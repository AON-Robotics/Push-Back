#include "aon/shadow/player.hpp"

#include "aon/shadow/mechanisms.hpp"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdint>
#include <limits>

namespace aon::shadow {
namespace {

bool finitePoint(const PathPoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.speed);
}

bool validDwellSegment(const RouteSegment& segment) {
  if (segment.durationMs == 0) return false;
  return segment.kind == SegmentKind::Dwell &&
         segment.direction == Direction::Stopped && segment.firstPoint == 0 &&
         segment.pointCount == 0;
}

}  // namespace

bool validMotionSegment(const ProcessedRoute& route,
                        const RouteSegment& segment) {
  if (segment.kind != SegmentKind::Motion || segment.durationMs == 0 ||
      (segment.direction != Direction::Forward &&
       segment.direction != Direction::Reverse) ||
      segment.pointCount < 2 || segment.firstPoint > route.pointCount ||
      segment.pointCount > route.pointCount - segment.firstPoint) {
    return false;
  }
  for (std::size_t offset = 0; offset < segment.pointCount; ++offset) {
    const auto& point = route.points[segment.firstPoint + offset];
    if (!finitePoint(point)) return false;
    if (offset + 1 < segment.pointCount &&
        (point.speed < 1.0F || point.speed > 100.0F)) {
      return false;
    }
    if (offset > 0) {
      const auto& previous = route.points[segment.firstPoint + offset - 1];
      if (point.x == previous.x && point.y == previous.y) return false;
    }
  }
  return true;
}

ResultCode validatePlayback(const DecodedRecording& recording,
                            const PlaybackPolicy& policy) {
  if (!policy.authorized || !policy.armed) return ResultCode::PlayLocked;
  if (policy.activeRobot != RobotIdentity::Small) {
    return ResultCode::UnsupportedRobot;
  }
  if (recording.capture.robot != policy.activeRobot) {
    return ResultCode::WrongRobot;
  }

  const auto& route = recording.route;
  if (route.result != ResultCode::Ok || route.segmentCount == 0 ||
      route.segmentCount > kMaximumSegments ||
      route.pointCount > kMaximumPathPoints ||
      route.eventCount > kMaximumEvents) {
    return ResultCode::CorruptFile;
  }

  for (std::size_t index = 0; index < route.segmentCount; ++index) {
    const auto& segment = route.segments[index];
    if (segment.kind == SegmentKind::Motion
            ? !validMotionSegment(route, segment)
            : !validDwellSegment(segment)) {
      return ResultCode::CorruptFile;
    }
  }

  std::uint16_t previousSegment = 0;
  float previousProgress = 0.0F;
  std::uint32_t previousOffset = 0;
  bool havePrevious = false;
  for (std::size_t index = 0; index < route.eventCount; ++index) {
    const auto& anchored = route.events[index];
    if (anchored.segmentIndex >= route.segmentCount) {
      return ResultCode::CorruptFile;
    }
    const auto& segment = route.segments[anchored.segmentIndex];
    if (havePrevious && anchored.segmentIndex < previousSegment) {
      return ResultCode::CorruptFile;
    }
    if (segment.kind == SegmentKind::Motion) {
      if (!std::isfinite(anchored.progress) || anchored.progress < 0.0F ||
          anchored.progress > 1.0F || anchored.offsetMs != 0) {
        return ResultCode::CorruptFile;
      }
      if (havePrevious && anchored.segmentIndex == previousSegment &&
          anchored.progress < previousProgress) {
        return ResultCode::CorruptFile;
      }
    } else {
      if (anchored.offsetMs > segment.durationMs || anchored.progress != 0.0F) {
        return ResultCode::CorruptFile;
      }
      if (havePrevious && anchored.segmentIndex == previousSegment &&
          anchored.offsetMs < previousOffset) {
        return ResultCode::CorruptFile;
      }
    }
    if (validateMechanism(policy.activeRobot, anchored.event) !=
        ResultCode::Ok) {
      return ResultCode::CorruptFile;
    }
    previousSegment = anchored.segmentIndex;
    previousProgress = anchored.progress;
    previousOffset = anchored.offsetMs;
    havePrevious = true;
  }
  return ResultCode::Ok;
}

ResultCode playRecording(const DecodedRecording& recording,
                         const PlaybackPolicy& policy,
                         PlaybackCallbacks& callbacks) {
  const auto finish = [&](ResultCode result) {
    if (callbacks.stopAll) callbacks.stopAll();
    return result;
  };

  const ResultCode validation = validatePlayback(recording, policy);
  if (validation != ResultCode::Ok) return finish(validation);
  if (!callbacks.follow || !callbacks.dwell || !callbacks.mechanism ||
      !callbacks.cancelled || !callbacks.stopAll) {
    return finish(ResultCode::CorruptFile);
  }

  const auto& route = recording.route;
  std::size_t nextEvent = 0;
  for (std::size_t segmentIndex = 0; segmentIndex < route.segmentCount;
       ++segmentIndex) {
    if (callbacks.cancelled()) return finish(ResultCode::Cancelled);
    const auto& segment = route.segments[segmentIndex];

    const auto dispatchReached = [&](auto reached) {
      while (nextEvent < route.eventCount &&
             route.events[nextEvent].segmentIndex == segmentIndex &&
             reached(route.events[nextEvent])) {
        const ResultCode result =
            callbacks.mechanism(route.events[nextEvent].event);
        if (result != ResultCode::Ok) return result;
        ++nextEvent;
      }
      return ResultCode::Ok;
    };

    ResultCode result = ResultCode::Ok;
    if (segment.kind == SegmentKind::Motion) {
      const MotionProgress observer = [&](float progress) {
        if (callbacks.cancelled()) return ResultCode::Cancelled;
        if (!std::isfinite(progress)) return ResultCode::CorruptFile;
        const float clamped = std::clamp(progress, 0.0F, 1.0F);
        return dispatchReached([&](const AnchoredEvent& event) {
          return event.progress <= clamped;
        });
      };
      result = observer(0.0F);
      if (result == ResultCode::Ok) {
        result = callbacks.follow(route, segment, observer);
      }
      if (result == ResultCode::Ok) result = observer(1.0F);
    } else {
      const DwellProgress observer = [&](std::uint32_t elapsedMs) {
        if (callbacks.cancelled()) return ResultCode::Cancelled;
        const std::uint32_t clamped = std::min(elapsedMs, segment.durationMs);
        return dispatchReached([&](const AnchoredEvent& event) {
          return event.offsetMs <= clamped;
        });
      };
      result = observer(0);
      if (result == ResultCode::Ok) {
        result = callbacks.dwell(segment.durationMs, observer);
      }
      if (result == ResultCode::Ok) result = observer(segment.durationMs);
    }
    if (result != ResultCode::Ok) return finish(result);
    if (callbacks.cancelled()) return finish(ResultCode::Cancelled);
  }

  if (nextEvent != route.eventCount) return finish(ResultCode::CorruptFile);
  return finish(ResultCode::Ok);
}

int playbackTimeoutMs(std::uint32_t durationMs) {
  constexpr std::uint64_t kMinimum = 2000;
  constexpr std::uint64_t kMaximum = static_cast<std::uint64_t>(INT_MAX);
  const auto scaled = static_cast<std::uint64_t>(durationMs) * 2U + 1000U;
  return static_cast<int>(std::min(kMaximum, std::max(kMinimum, scaled)));
}

float monotonicPolylineProgress(const PathPoint* points, std::size_t count,
                                float x, float y, float previous) {
  if (points == nullptr || count < 2 || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(previous)) {
    return previous;
  }

  float totalLength = 0.0F;
  for (std::size_t index = 0; index < count; ++index) {
    if (!finitePoint(points[index])) return previous;
    if (index > 0) {
      const float dx = points[index].x - points[index - 1].x;
      const float dy = points[index].y - points[index - 1].y;
      const float length = std::hypot(dx, dy);
      if (!(length > 0.0F) || !std::isfinite(length)) return previous;
      totalLength += length;
    }
  }
  if (!(totalLength > 0.0F) || !std::isfinite(totalLength)) return previous;

  const float previousLength =
      std::clamp(previous, 0.0F, 1.0F) * totalLength;
  std::size_t currentEdge = 1;
  float currentEdgeStartLength = 0.0F;
  float traversedLength = 0.0F;
  for (std::size_t index = 1; index < count; ++index) {
    const float length = std::hypot(points[index].x - points[index - 1].x,
                                    points[index].y - points[index - 1].y);
    if (previousLength < traversedLength + length || index + 1 == count) {
      currentEdge = index;
      currentEdgeStartLength = traversedLength;
      break;
    }
    traversedLength += length;
  }

  float nearestDistanceSquared = std::numeric_limits<float>::infinity();
  float nearestProgress = previous;
  float completedLength = currentEdgeStartLength;
  const std::size_t lastCandidateEdge =
      std::min(count - 1, currentEdge + 1);
  for (std::size_t index = currentEdge; index <= lastCandidateEdge; ++index) {
    const auto& start = points[index - 1];
    const auto& end = points[index];
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float lengthSquared = dx * dx + dy * dy;
    const float length = std::sqrt(lengthSquared);
    const float local = std::clamp(
        ((x - start.x) * dx + (y - start.y) * dy) / lengthSquared,
        0.0F, 1.0F);
    const float projectedX = start.x + local * dx;
    const float projectedY = start.y + local * dy;
    const float offsetX = x - projectedX;
    const float offsetY = y - projectedY;
    const float distanceSquared = offsetX * offsetX + offsetY * offsetY;
    if (distanceSquared < nearestDistanceSquared) {
      nearestDistanceSquared = distanceSquared;
      nearestProgress = (completedLength + local * length) / totalLength;
    }
    completedLength += length;
  }
  return std::max(previous, std::clamp(nearestProgress, 0.0F, 1.0F));
}

}  // namespace aon::shadow
