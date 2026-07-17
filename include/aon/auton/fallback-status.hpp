#pragma once

#include "aon/auton/motion-health.hpp"

#include <array>
#include <cstdint>

namespace aon::auton {

enum class MotionMode { Tracking, ForcedEncoder, FaultedEncoder };

struct FallbackStatusSnapshot {
  MotionMode mode = MotionMode::Tracking;
  MotionFailureReason reason = MotionFailureReason::None;
  bool selectionLocked = false;
  std::array<char, 48> actionName{};
  std::uint32_t changedAt = 0;
};

bool selectForcedEncoder(bool forced);
void lockFallbackSelection();
void latchFallbackFault(MotionFailureReason reason, const char* actionName);
FallbackStatusSnapshot fallbackStatus();
const char* motionModeName(MotionMode mode);

}  // namespace aon::auton
