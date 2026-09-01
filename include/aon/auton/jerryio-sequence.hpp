#pragma once

#include <cstdint>
#include <functional>

namespace aon::auton {

enum class JerryIoPhase : std::uint8_t {
  FollowToIntake,
  Intake,
  FollowToOuttake,
  Outtake,
  FollowToPistons,
  PulsePistons,
};

struct JerryIoSequenceResult {
  bool succeeded;
  JerryIoPhase failedPhase;
};

struct JerryIoCallbacks {
  std::function<bool(JerryIoPhase)> run;
  std::function<void()> stopAll;
};

/** Runs each path leg and its checkpoint action, then stops all outputs. */
JerryIoSequenceResult runJerryIoSequence(JerryIoCallbacks& callbacks);

}  // namespace aon::auton
