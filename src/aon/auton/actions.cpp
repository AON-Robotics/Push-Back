#include "aon/auton/actions.hpp"

#include "aon/auton/encoder-motion.hpp"
#include "aon/auton/fallback-geometry.hpp"
#include "aon/auton/motion-control.hpp"
#include "aon/auton/motion-health.hpp"
#include "aon/config/robot-config.hpp"
#include "aon/core/hardware.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/lemlib/drive-io.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

namespace aon::auton {
namespace {

enum class TargetKind { Point, Pose, Heading, Unsupported };

struct ActionTarget {
  TargetKind kind;
  double x;
  double y;
  double heading;
  bool forwards;
};

MotionControl motionControl;
EncoderMotionController encoderController;

class MotionLease {
 public:
  explicit MotionLease(MotionControl& control)
      : control_(control), acquired_(control_.tryBegin()) {}
  ~MotionLease() noexcept {
    if (acquired_) control_.finish();
  }
  MotionLease(const MotionLease&) = delete;
  MotionLease& operator=(const MotionLease&) = delete;
  MotionLease(MotionLease&&) = delete;
  MotionLease& operator=(MotionLease&&) = delete;

  bool acquired() const { return acquired_; }

 private:
  MotionControl& control_;
  bool acquired_;
};

MotionSample motionSample() {
  const auto pose = lemlib_integration::chassis().getPose();
  const auto drive = lemlib_integration::sampleDriveSensors();
  MotionSample sample;
  sample.timeMs = pros::millis();
  sample.poseX = pose.x;
  sample.poseY = pose.y;
  sample.poseHeading = pose.theta;
  sample.poseValid = std::isfinite(pose.x) && std::isfinite(pose.y) &&
                     std::isfinite(pose.theta);
  sample.leftMotorDegrees = drive.leftMotorDegrees;
  sample.rightMotorDegrees = drive.rightMotorDegrees;
  sample.leftTrackingInches = drive.leftTrackingInches;
  sample.rightTrackingInches = drive.rightTrackingInches;
  sample.backTrackingInches = drive.backTrackingInches;
  sample.imuDegrees = drive.imuDegrees;
  sample.leftMotorValid = drive.leftMotorValid;
  sample.rightMotorValid = drive.rightMotorValid;
  sample.leftTrackingValid = drive.leftTrackingValid;
  sample.rightTrackingValid = drive.rightTrackingValid;
  sample.backTrackingValid = drive.backTrackingValid;
  sample.imuValid = drive.imuValid;
  return sample;
}

MotionFailureReason invalidReason(const MotionSample& sample) {
  if (!sample.poseValid) return MotionFailureReason::NonFinitePose;
  if (!sample.leftMotorValid || !sample.rightMotorValid ||
      !sample.leftTrackingValid || !sample.rightTrackingValid ||
      !sample.backTrackingValid || !sample.imuValid) {
    return MotionFailureReason::DeviceInvalid;
  }
  return MotionFailureReason::None;
}

void logStart(const char* operation, const char* name) {
  std::printf("AUTON_START operation=%s name=%s time=%lu\n", operation, name,
              static_cast<unsigned long>(pros::millis()));
}

void logFinish(const char* operation, const char* name,
               const MotionResult& result) {
  const lemlib::Pose pose = lemlib_integration::chassis().getPose();
  std::printf(
      "AUTON_FINISH operation=%s name=%s succeeded=%d mode=%s fallback=%d "
      "reason=%s x=%.3f y=%.3f heading=%.3f time=%lu\n",
      operation, name, result.succeeded ? 1 : 0,
      motionModeName(result.mode), result.fallbackUsed ? 1 : 0,
      motionFailureName(result.reason), pose.x, pose.y, pose.theta,
      static_cast<unsigned long>(pros::millis()));
}

FallbackGeometry fallbackGeometry(const MotionSample& trusted,
                                  const ActionTarget& target) {
  const TrustedPose start{trusted.poseX, trusted.poseY,
                          trusted.poseHeading};
  switch (target.kind) {
    case TargetKind::Point:
      return withTravelDirection(pointFallback(start, target.x, target.y),
                                 target.forwards);
    case TargetKind::Pose:
      return withTravelDirection(
          poseFallback(start, target.x, target.y, target.heading),
          target.forwards);
    case TargetKind::Heading:
      return {headingFallback(start.heading, target.heading), 0.0, 0.0};
    case TargetKind::Unsupported:
      return {0.0, 0.0, 0.0};
  }
  return {0.0, 0.0, 0.0};
}

void logFallback(const char* name, MotionFailureReason reason,
                 const MotionSample& trusted, const MotionSample& failed,
                 const ActionTarget& target) {
  std::printf(
      "ODOM_FALLBACK name=%s reason=%s trusted_x=%.3f trusted_y=%.3f "
      "trusted_h=%.3f target_x=%.3f target_y=%.3f target_h=%.3f "
      "track_l=%.3f track_r=%.3f track_b=%.3f motor_l=%.3f "
      "motor_r=%.3f time=%lu\n",
      name, motionFailureName(reason), trusted.poseX, trusted.poseY,
      trusted.poseHeading, target.x, target.y, target.heading,
      failed.leftTrackingInches, failed.rightTrackingInches,
      failed.backTrackingInches, failed.leftMotorDegrees,
      failed.rightMotorDegrees, static_cast<unsigned long>(pros::millis()));
}

bool stopAndSettle() {
  using namespace lemlib_integration;
  auto& robotChassis = chassis();
  const auto& fallback = config::activeRobotConfig().lemlib.fallback;
  robotChassis.cancelAllMotions();
  stopDrive();
  setDriveBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  const DriveSensorSample start = sampleDriveSensors();
  pros::delay(fallback.settleMs / 2);
  const DriveSensorSample middle = sampleDriveSensors();
  pros::delay(fallback.settleMs - fallback.settleMs / 2);
  const DriveSensorSample finish = sampleDriveSensors();
  if (!start.leftMotorValid || !start.rightMotorValid ||
      !middle.leftMotorValid || !middle.rightMotorValid ||
      !finish.leftMotorValid || !finish.rightMotorValid) {
    return false;
  }

  const double firstMovement =
      std::abs(middle.leftMotorDegrees - start.leftMotorDegrees) +
      std::abs(middle.rightMotorDegrees - start.rightMotorDegrees);
  const double finalMovement =
      std::abs(finish.leftMotorDegrees - middle.leftMotorDegrees) +
      std::abs(finish.rightMotorDegrees - middle.rightMotorDegrees);
  return finalMovement <= firstMovement + 2.0 && finalMovement <= 3.0;
}

MotionResult runEncoder(const char* name, const ActionTarget& target,
                        const MotionSample& trusted, int requestedMaxOutput,
                        std::uint32_t timeoutMs, MotionMode mode,
                        MotionFailureReason transitionReason) {
  if (target.kind == TargetKind::Unsupported) {
    lemlib_integration::stopDrive();
    return {false, mode, true, MotionFailureReason::Unsupported};
  }
  const EncoderMotionResult encoderResult = encoderController.execute(
      {name, fallbackGeometry(trusted, target), requestedMaxOutput, timeoutMs,
       trusted.imuValid},
      motionControl);
  return {encoderResult.succeeded, mode, true,
          encoderResult.succeeded ? transitionReason : encoderResult.reason};
}

MotionResult runMonitored(const char* operation, const char* name,
                          const ActionTarget& target, MotionIntent intent,
                          int requestedMaxOutput, std::uint32_t timeoutMs,
                          const std::function<void()>& startMotion,
                          const std::function<void()>& onPoll,
                          OdometryMonitoring monitoring =
                              OdometryMonitoring::Configured) {
  const FallbackStatusSnapshot status = fallbackStatus();
  logStart(operation, name);
  MotionLease lease(motionControl);
  if (!lease.acquired()) {
    MotionResult result{false, status.mode, false, MotionFailureReason::Busy};
    logFinish(operation, name, result);
    return result;
  }

  MotionSample initial = motionSample();

  if (status.mode != MotionMode::Tracking) {
    if (monitoring == OdometryMonitoring::FailClosed) {
      lemlib_integration::stopDrive();
      const MotionFailureReason reason =
          status.reason == MotionFailureReason::None
              ? MotionFailureReason::DeviceInvalid
              : status.reason;
      MotionResult result{false, status.mode, false, reason};
      logFinish(operation, name, result);
      return result;
    }
    MotionResult result =
        runEncoder(name, target, initial, requestedMaxOutput, timeoutMs,
                   status.mode, status.reason);
    logFinish(operation, name, result);
    return result;
  }

  MotionHealthMonitor monitor(
      config::activeRobotConfig().lemlib.fallback.health);
  monitor.reset(initial);
  const bool failClosedMonitoring =
      monitoring == OdometryMonitoring::FailClosed;
  const bool healthMonitoring =
      failClosedMonitoring ||
      shouldMonitorAutomatically(
          status.mode == MotionMode::Tracking,
          config::activeRobotConfig()
              .lemlib.fallback.automaticFallbackAuthorized);
  if (failClosedMonitoring) {
    const MotionFailureReason reason = invalidReason(initial);
    if (reason != MotionFailureReason::None) {
      lemlib_integration::stopDrive();
      MotionResult result{false, MotionMode::Tracking, false, reason};
      logFinish(operation, name, result);
      return result;
    }
  }
  const std::uint32_t startedAt = pros::millis();
  if (!motionControl.runIfActive([&] { startMotion(); })) {
    MotionResult result{false, status.mode, false,
                        MotionFailureReason::Cancelled};
    logFinish(operation, name, result);
    return result;
  }

  auto& robotChassis = lemlib_integration::chassis();
  while (robotChassis.isInMotion()) {
    if (motionControl.isCancelled()) {
      robotChassis.cancelAllMotions();
      lemlib_integration::stopDrive();
      MotionResult result{false, MotionMode::Tracking, false,
                          MotionFailureReason::Cancelled};
      logFinish(operation, name, result);
      return result;
    }
    if (pros::millis() - startedAt >= timeoutMs) {
      robotChassis.cancelAllMotions();
      lemlib_integration::stopDrive();
      MotionResult result{false, MotionMode::Tracking, false,
                          MotionFailureReason::Timeout};
      logFinish(operation, name, result);
      return result;
    }

    const MotionSample sample = motionSample();
    const MotionFailureReason immediateReason =
        failClosedMonitoring ? invalidReason(sample)
                             : MotionFailureReason::None;
    const MotionFailureReason reason =
        immediateReason != MotionFailureReason::None
            ? immediateReason
            : (healthMonitoring ? monitor.observe(sample, intent)
                                : MotionFailureReason::None);
    if (reason != MotionFailureReason::None) {
      if (failClosedMonitoring) {
        robotChassis.cancelAllMotions();
        lemlib_integration::stopDrive();
        latchFallbackFault(reason, name);
        MotionResult result{false, MotionMode::Tracking, false, reason};
        logFinish(operation, name, result);
        return result;
      }
      const MotionSample trusted = monitor.lastTrustedSample();
      logFallback(name, reason, trusted, sample, target);
      if (!stopAndSettle()) {
        latchFallbackFault(reason, name);
        MotionResult result{false, MotionMode::FaultedEncoder, true,
                            MotionFailureReason::RetryFailed};
        logFinish(operation, name, result);
        return result;
      }
      if (motionControl.isCancelled()) {
        MotionResult result{false, MotionMode::Tracking, false,
                            MotionFailureReason::Cancelled};
        logFinish(operation, name, result);
        return result;
      }

      latchFallbackFault(reason, name);
      const auto& fallback = config::activeRobotConfig().lemlib.fallback;
      const std::uint32_t retryBudget = fallbackBudget(
          startedAt, pros::millis(), timeoutMs,
          fallback.transitionAllowanceMs);
      MotionResult result =
          runEncoder(name, target, trusted, requestedMaxOutput, retryBudget,
                     MotionMode::FaultedEncoder, reason);
      logFinish(operation, name, result);
      return result;
    }
    if (onPoll) onPoll();
    pros::delay(20);
  }

  if (motionControl.isCancelled()) {
    lemlib_integration::stopDrive();
    MotionResult result{false, MotionMode::Tracking, false,
                        MotionFailureReason::Cancelled};
    logFinish(operation, name, result);
    return result;
  }

  if (pros::millis() - startedAt >= timeoutMs) {
    lemlib_integration::stopDrive();
    MotionResult result{false, MotionMode::Tracking, false,
                        MotionFailureReason::Timeout};
    logFinish(operation, name, result);
    return result;
  }

  MotionResult result{true, MotionMode::Tracking, false,
                      MotionFailureReason::None};
  logFinish(operation, name, result);
  return result;
}

}  // namespace

void Actions::setPose(double x, double y, double heading) {
  if (config::activeRobotConfig().localization.fusedLemLibAuthorized) {
    Odometry& odometry = core::hardware().odometry;
    if (!odometry.resetPose(x, y, heading)) {
      cancelMotion();
      return;
    }
    const Pose fused = odometry.getPose();
    lemlib_integration::chassis().setPose(fused.x, fused.y, fused.theta);
    return;
  }
  lemlib_integration::chassis().setPose(x, y, heading);
}

MotionResult Actions::moveToPoint(const char* name, double x, double y,
                                  int timeout,
                                  lemlib::MoveToPointParams params,
                                  OdometryMonitoring monitoring) {
  return runMonitored(
      "moveToPoint", name, {TargetKind::Point, x, y, 0.0, params.forwards},
      MotionIntent::Linear, static_cast<int>(params.maxSpeed), timeout,
      [=] { lemlib_integration::chassis().moveToPoint(x, y, timeout, params); },
      {}, monitoring);
}

MotionResult Actions::moveToPose(const char* name, double x, double y,
                                 double heading, int timeout,
                                 lemlib::MoveToPoseParams params,
                                 OdometryMonitoring monitoring) {
  return runMonitored(
      "moveToPose", name,
      {TargetKind::Pose, x, y, heading, params.forwards},
      MotionIntent::Linear, static_cast<int>(params.maxSpeed), timeout, [=] {
        lemlib_integration::chassis().moveToPose(x, y, heading, timeout, params);
      }, {}, monitoring);
}

MotionResult Actions::turnToHeading(const char* name, double heading,
                                    int timeout,
                                    lemlib::TurnToHeadingParams params) {
  return runMonitored(
      "turnToHeading", name,
      {TargetKind::Heading, 0.0, 0.0, heading, true}, MotionIntent::Turn,
      params.maxSpeed, timeout, [=] {
        lemlib_integration::chassis().turnToHeading(heading, timeout, params);
      }, {});
}

MotionResult Actions::arcadeFor(const char* name, int throttle, int turn,
                                std::uint32_t durationMs,
                                OdometryMonitoring monitoring) {
  logStart("arcadeFor", name);
  const auto mode = fallbackStatus().mode;
  MotionLease lease(motionControl);
  if (!lease.acquired()) {
    MotionResult result{false, mode, false, MotionFailureReason::Busy};
    logFinish("arcadeFor", name, result);
    return result;
  }

  const bool failClosedMonitoring =
      monitoring == OdometryMonitoring::FailClosed;
  if (failClosedMonitoring && mode != MotionMode::Tracking) {
    lemlib_integration::stopDrive();
    MotionResult result{false, mode, false, MotionFailureReason::DeviceInvalid};
    logFinish("arcadeFor", name, result);
    return result;
  }
  if (failClosedMonitoring) {
    const MotionFailureReason initialReason = invalidReason(motionSample());
    if (initialReason != MotionFailureReason::None) {
      lemlib_integration::stopDrive();
      MotionResult result{false, mode, false, initialReason};
      logFinish("arcadeFor", name, result);
      return result;
    }
  }

  auto& robotChassis = lemlib_integration::chassis();
  if (!motionControl.runIfActive(
          [&] { robotChassis.arcade(throttle, turn, true); })) {
    MotionResult result{false, mode, mode != MotionMode::Tracking,
                        MotionFailureReason::Cancelled};
    logFinish("arcadeFor", name, result);
    return result;
  }

  const std::uint32_t startedAt = pros::millis();
  MotionFailureReason reason = MotionFailureReason::None;
  while (pros::millis() - startedAt < durationMs) {
    if (motionControl.isCancelled()) {
      reason = MotionFailureReason::Cancelled;
      break;
    }
    if (failClosedMonitoring) {
      reason = invalidReason(motionSample());
      if (reason != MotionFailureReason::None) break;
    } else {
      const auto drive = lemlib_integration::sampleDriveSensors();
      if (!driveFeedbackValid(drive.leftMotorValid, drive.rightMotorValid)) {
        reason = MotionFailureReason::DeviceInvalid;
        break;
      }
    }
    pros::delay(20);
  }
  motionControl.runIfActive([&] { robotChassis.arcade(0, 0, true); });
  lemlib_integration::stopDrive();
  if (failClosedMonitoring && reason != MotionFailureReason::None &&
      reason != MotionFailureReason::Cancelled) {
    latchFallbackFault(reason, name);
  }
  MotionResult result{reason == MotionFailureReason::None, mode,
                      mode != MotionMode::Tracking, reason};
  logFinish("arcadeFor", name, result);
  return result;
}

MotionResult Actions::followPath(const char* name, const asset& path,
                                 float lookahead, int timeout, bool forwards,
                                 const std::function<void()>& onPoll,
                                 OdometryMonitoring monitoring) {
  return runMonitored(
      "followPath", name,
      {TargetKind::Unsupported, 0.0, 0.0, 0.0, forwards},
      MotionIntent::Linear, 127, timeout, [=, &path] {
        lemlib_integration::chassis().follow(path, lookahead, timeout, forwards);
      }, onPoll, monitoring);
}

void Actions::cancelMotion() {
  motionControl.cancelAndRun([] {
    lemlib_integration::chassis().cancelAllMotions();
    lemlib_integration::stopDrive();
  });
}

bool Actions::prepareMotion() {
  return motionControl.stopAndPrepare([] {
    lemlib_integration::chassis().cancelAllMotions();
    lemlib_integration::stopDrive();
  });
}

void Actions::resetCancellation() { motionControl.resetCancellation(); }

bool Actions::isCancellationLatched() const {
  return motionControl.isCancelled();
}

void Actions::stop(pros::motor_brake_mode_e brakeMode) {
  cancelMotion();
  lemlib_integration::setDriveBrakeMode(brakeMode);
}

Actions& actions() {
  static Actions instance;
  return instance;
}

}  // namespace aon::auton
