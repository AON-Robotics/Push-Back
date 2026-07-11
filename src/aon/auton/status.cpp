#include "aon/auton/status.hpp"

#include "pros/rtos.hpp"

#include <cstdio>

namespace aon::auton {
namespace {

pros::Mutex statusMutex;
RoutineStatus status{"None", RoutineState::Idle, 0, 0};

const char* stateName(RoutineState state) {
  switch (state) {
    case RoutineState::Idle:
      return "idle";
    case RoutineState::Selected:
      return "selected";
    case RoutineState::Running:
      return "running";
    case RoutineState::Completed:
      return "completed";
    case RoutineState::Failed:
      return "failed";
  }
  return "unknown";
}

void logStatus(const RoutineStatus& snapshot) {
  std::printf("AUTON_STATE name=%s state=%s time=%lu\n", snapshot.name,
              stateName(snapshot.state),
              static_cast<unsigned long>(pros::millis()));
}

}  // namespace

void selectRoutine(const char* name) {
  statusMutex.take();
  status = {name, RoutineState::Selected, 0, 0};
  const RoutineStatus snapshot = status;
  statusMutex.give();
  logStatus(snapshot);
}

void startRoutine(const char* name) {
  const std::uint32_t now = pros::millis();
  statusMutex.take();
  status = {name, RoutineState::Running, now, 0};
  const RoutineStatus snapshot = status;
  statusMutex.give();
  logStatus(snapshot);
}

void finishRoutine(bool succeeded) {
  statusMutex.take();
  status.state = succeeded ? RoutineState::Completed : RoutineState::Failed;
  status.finishedAt = pros::millis();
  const RoutineStatus snapshot = status;
  statusMutex.give();
  logStatus(snapshot);
}

RoutineStatus routineStatus() {
  statusMutex.take();
  const RoutineStatus snapshot = status;
  statusMutex.give();
  return snapshot;
}

}  // namespace aon::auton
