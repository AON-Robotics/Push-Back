#pragma once

#include "aon/auton/motion-health.hpp"

#include <array>
#include <cstdint>

namespace aon::auton {

/** Active source used to close the drivetrain motion loop. */
enum class MotionMode { Tracking, ForcedEncoder, FaultedEncoder };

/** Atomic-style value snapshot of the current fallback decision. */
struct FallbackStatusSnapshot {
  MotionMode mode = MotionMode::Tracking;
  MotionFailureReason reason = MotionFailureReason::None;
  bool selectionLocked = false;
  std::array<char, 48> actionName{};
  std::uint32_t changedAt = 0;
};

/** Requests forced encoder mode before selection is locked for a routine. */
[[nodiscard]] bool selectForcedEncoder(bool forced);
/** Prevents fallback selection changes until the next routine reset. */
void lockFallbackSelection();
/** Records a terminal fallback failure and the action that detected it. */
void latchFallbackFault(MotionFailureReason reason, const char* actionName);
/** Returns a by-value snapshot safe to retain after the call. */
[[nodiscard]] FallbackStatusSnapshot fallbackStatus();
/** Returns a process-lifetime display name for a motion mode. */
[[nodiscard]] const char* motionModeName(MotionMode mode);

}  // namespace aon::auton
