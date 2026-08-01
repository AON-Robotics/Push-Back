#include "aon/shadow/player-pros.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#if defined(__arm__) || defined(__thumb__)
#include "aon/auton/actions.hpp"
#include "aon/constants.hpp"
#include "aon/core/hardware.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/shadow/mechanisms.hpp"
#include "lemlib/asset.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#endif

namespace aon::shadow {
namespace {

constexpr char kRuntimePathMetadata[] =
    "endData\n"
    "#PATH.JERRYIO-DATA {\"appVersion\":\"0.11.0\",\"format\":\"LemLib "
    "v0.5\",\"name\":\"Shadow Runtime\"}\n";

bool append(RuntimePath& output, const char* data, std::size_t size) {
  if (size > output.bytes.size() - output.used) return false;
  std::memcpy(output.bytes.data() + output.used, data, size);
  output.used += size;
  return true;
}

#if defined(__arm__) || defined(__thumb__)
RuntimePath runtimePath;
const ProcessedRoute* activeRoute = nullptr;
const RouteSegment* activeSegment = nullptr;
float previousProgress = 0.0F;
std::atomic<bool> playbackCancelled{false};

bool hardwareCancelled() {
  return playbackCancelled.load() || pros::competition::is_disabled() ||
         aon::core::hardware().mainController.get_digital(
             pros::E_CONTROLLER_DIGITAL_X);
}

ResultCode motionResult(const aon::auton::MotionResult& result) {
  if (result.succeeded) return ResultCode::Ok;
  if (result.reason == aon::auton::MotionFailureReason::Cancelled) {
    return ResultCode::Cancelled;
  }
  switch (result.reason) {
    case aon::auton::MotionFailureReason::DeviceInvalid:
    case aon::auton::MotionFailureReason::NonFinitePose:
    case aon::auton::MotionFailureReason::ImpossiblePoseJump:
    case aon::auton::MotionFailureReason::FrozenTracking:
      return ResultCode::OdometryFailure;
    default:
      return ResultCode::MotionFailure;
  }
}
#endif

}  // namespace

ResultCode serializeRuntimePath(const ProcessedRoute& route,
                                const RouteSegment& segment,
                                RuntimePath& output) {
  output.used = 0;
  if (!validMotionSegment(route, segment)) return ResultCode::CorruptFile;

  char line[192];
  for (std::size_t offset = 0; offset < segment.pointCount; ++offset) {
    const auto& point = route.points[segment.firstPoint + offset];
    const float speed = offset + 1 == segment.pointCount ? 0.0F : point.speed;
    const int written = std::snprintf(line, sizeof(line), "%f, %f, %f\n",
                                      static_cast<double>(point.x),
                                      static_cast<double>(point.y),
                                      static_cast<double>(speed));
    if (written < 0 || static_cast<std::size_t>(written) >= sizeof(line) ||
        !append(output, line, static_cast<std::size_t>(written))) {
      output.used = 0;
      return ResultCode::CapacityReached;
    }
  }
  if (!append(output, kRuntimePathMetadata,
              sizeof(kRuntimePathMetadata) - 1)) {
    output.used = 0;
    return ResultCode::CapacityReached;
  }
  return ResultCode::Ok;
}

ResultCode validateRuntimePaths(const ProcessedRoute& route,
                                RuntimePath& scratch) {
  if (route.segmentCount == 0 || route.segmentCount > kMaximumSegments) {
    return ResultCode::CorruptFile;
  }
  for (std::size_t index = 0; index < route.segmentCount; ++index) {
    if (route.segments[index].kind != SegmentKind::Motion) continue;
    const ResultCode result =
        serializeRuntimePath(route, route.segments[index], scratch);
    if (result != ResultCode::Ok) return result;
  }
  scratch.used = 0;
  return ResultCode::Ok;
}

#if defined(__arm__) || defined(__thumb__)
ResultCode playOnRobot(const DecodedRecording& recording,
                       const PlaybackPolicy& policy) {
#if USING_BIG_ROBOT
  (void)recording;
  (void)policy;
  return ResultCode::UnsupportedRobot;
#else
  const ResultCode validation = validatePlayback(recording, policy);
  if (validation != ResultCode::Ok) {
    cancelRobotPlayback();
    return validation;
  }
  const ResultCode runtimeValidation =
      validateRuntimePaths(recording.route, runtimePath);
  if (runtimeValidation != ResultCode::Ok) {
    cancelRobotPlayback();
    return runtimeValidation;
  }
  ResultCode observerFailure = ResultCode::Ok;
  PlaybackCallbacks callbacks{
      [&](const ProcessedRoute& route, const RouteSegment& segment,
          const MotionProgress& progress) {
        activeRoute = &route;
        activeSegment = &segment;
        previousProgress = 0.0F;
        observerFailure = serializeRuntimePath(route, segment, runtimePath);
        if (observerFailure != ResultCode::Ok) return observerFailure;
        asset runtimeAsset{runtimePath.bytes.data(), runtimePath.used};
        const auto poll = [&] {
          if (observerFailure != ResultCode::Ok) return;
          if (hardwareCancelled()) {
            observerFailure = ResultCode::Cancelled;
          } else {
            const auto pose = aon::lemlib_integration::chassis().getPose();
            if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
                !std::isfinite(pose.theta)) {
              observerFailure = ResultCode::OdometryFailure;
            } else {
              previousProgress = monotonicPolylineProgress(
                  activeRoute->points.data() + activeSegment->firstPoint,
                  activeSegment->pointCount, static_cast<float>(pose.x),
                  static_cast<float>(pose.y), previousProgress);
              observerFailure = progress(previousProgress);
            }
          }
          if (observerFailure != ResultCode::Ok) {
            aon::auton::actions().cancelMotion();
          }
        };
        const auto result = aon::auton::actions().followPath(
            "Shadow segment", runtimeAsset, 6.0F,
            playbackTimeoutMs(segment.durationMs),
            segment.direction == Direction::Forward, poll,
            aon::auton::OdometryMonitoring::FailClosed);
        if (observerFailure != ResultCode::Ok) return observerFailure;
        return motionResult(result);
      },
      [&](std::uint32_t durationMs, const DwellProgress& progress) {
        const std::uint32_t startedAt = pros::millis();
        while (true) {
          if (hardwareCancelled()) return ResultCode::Cancelled;
          const std::uint32_t elapsed = pros::millis() - startedAt;
          const ResultCode result = progress(elapsed);
          if (result != ResultCode::Ok) return result;
          if (elapsed >= durationMs) return ResultCode::Ok;
          pros::delay(20);
        }
      },
      [](const MechanismEvent& event) { return applyMechanism(event); },
      [] { return hardwareCancelled(); },
      [] {
        aon::auton::actions().cancelMotion();
        aon::auton::actions().stop();
        stopAllMechanisms();
      }};
  return playRecording(recording, policy, callbacks);
#endif
}

ResultCode prepareRobotPlayback() {
  playbackCancelled.store(false);
  stopAllMechanisms();
  return aon::auton::actions().prepareMotion() ? ResultCode::Ok
                                               : ResultCode::Cancelled;
}

void cancelRobotPlayback() {
  playbackCancelled.store(true);
  aon::auton::actions().cancelMotion();
  aon::auton::actions().stop();
  stopAllMechanisms();
}
#else
ResultCode playOnRobot(const DecodedRecording&, const PlaybackPolicy&) {
  return ResultCode::UnsupportedRobot;
}

ResultCode prepareRobotPlayback() { return ResultCode::UnsupportedRobot; }

void cancelRobotPlayback() {}
#endif

}  // namespace aon::shadow
