#include "aon/auton/fallback-status.hpp"

#include "pros/rtos.hpp"

#include <cstdio>

namespace aon::auton {
namespace {

pros::Mutex statusMutex;
FallbackStatusSnapshot status;

void copyActionName(const char* name) {
  std::snprintf(status.actionName.data(), status.actionName.size(), "%s",
                name == nullptr ? "" : name);
}

}  // namespace

bool selectForcedEncoder(bool forced) {
  statusMutex.take();
  if (status.selectionLocked || status.mode == MotionMode::FaultedEncoder) {
    statusMutex.give();
    return false;
  }
  status.mode =
      forced ? MotionMode::ForcedEncoder : MotionMode::Tracking;
  status.reason = MotionFailureReason::None;
  copyActionName("");
  status.changedAt = pros::millis();
  statusMutex.give();
  return true;
}

void lockFallbackSelection() {
  statusMutex.take();
  status.selectionLocked = true;
  statusMutex.give();
}

void latchFallbackFault(MotionFailureReason reason, const char* actionName) {
  statusMutex.take();
  status.mode = MotionMode::FaultedEncoder;
  status.reason = reason;
  status.selectionLocked = true;
  copyActionName(actionName);
  status.changedAt = pros::millis();
  statusMutex.give();
}

FallbackStatusSnapshot fallbackStatus() {
  statusMutex.take();
  const FallbackStatusSnapshot snapshot = status;
  statusMutex.give();
  return snapshot;
}

const char* motionModeName(MotionMode mode) {
  switch (mode) {
    case MotionMode::Tracking:
      return "tracking";
    case MotionMode::ForcedEncoder:
      return "forced-encoder";
    case MotionMode::FaultedEncoder:
      return "faulted-encoder";
  }
  return "unknown";
}

}  // namespace aon::auton
