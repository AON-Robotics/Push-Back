#pragma once

#include "aon/shadow/codec.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace aon::shadow {

struct PlaybackPolicy {
  bool authorized = false;
  bool armed = false;
  RobotIdentity activeRobot = RobotIdentity::Small;
};

using MotionProgress = std::function<ResultCode(float progress)>;
using DwellProgress = std::function<ResultCode(std::uint32_t elapsedMs)>;

struct PlaybackCallbacks {
  std::function<ResultCode(const ProcessedRoute&, const RouteSegment&,
                           const MotionProgress&)> follow;
  std::function<ResultCode(std::uint32_t durationMs,
                           const DwellProgress&)> dwell;
  std::function<ResultCode(const MechanismEvent&)> mechanism;
  std::function<bool()> cancelled;
  std::function<void()> stopAll;
  /**
   * Initializes odometry from the recording's confirmed starting pose.
   * Called exactly once before any segment or mechanism callback.
   */
  std::function<ResultCode(const RawSample& start)> initializePose;
};

bool validMotionSegment(const ProcessedRoute& route,
                        const RouteSegment& segment);
ResultCode validatePlayback(const DecodedRecording& recording,
                            const PlaybackPolicy& policy);
ResultCode playRecording(const DecodedRecording& recording,
                         const PlaybackPolicy& policy,
                         PlaybackCallbacks& callbacks);
int playbackTimeoutMs(std::uint32_t durationMs);
float monotonicPolylineProgress(const PathPoint* points, std::size_t count,
                                float x, float y, float previous);

}  // namespace aon::shadow
