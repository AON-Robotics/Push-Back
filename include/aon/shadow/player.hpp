#pragma once

#include "aon/shadow/codec.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>

namespace aon::shadow {

/** Runtime authorization and robot-identity requirements for playback. */
struct PlaybackPolicy {
  bool authorized = false;
  bool armed = false;
  RobotIdentity activeRobot = RobotIdentity::Small;
};

/** Reports motion-segment progress in the inclusive range [0, 1]. */
using MotionProgress = std::function<ResultCode(float progress)>;
/** Reports dwell progress in milliseconds since the dwell began. */
using DwellProgress = std::function<ResultCode(std::uint32_t elapsedMs)>;

/**
 * @brief Hardware boundary used by the deterministic playback scheduler.
 *
 * Callbacks are borrowed for the duration of playRecording and must remain
 * valid until it returns. All callbacks run synchronously on the caller's
 * task. stopAll is called on every success and failure exit.
 */
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

/** Validates a motion segment's bounds, geometry, speed, and duration. */
[[nodiscard]] bool validMotionSegment(const ProcessedRoute& route,
                                      const RouteSegment& segment);
/** Validates a recording without issuing hardware commands. */
[[nodiscard]] ResultCode validatePlayback(const DecodedRecording& recording,
                                          const PlaybackPolicy& policy);
/** Runs validated playback synchronously and always invokes stopAll. */
ResultCode playRecording(const DecodedRecording& recording,
                         const PlaybackPolicy& policy,
                         PlaybackCallbacks& callbacks);
/** Derives a bounded LemLib timeout in milliseconds from recorded duration. */
[[nodiscard]] int playbackTimeoutMs(std::uint32_t durationMs);
/** Projects an inch-based pose onto a path without allowing progress reversal. */
[[nodiscard]] float monotonicPolylineProgress(
    const PathPoint* points, std::size_t count, float x, float y,
    float previous);

}  // namespace aon::shadow
