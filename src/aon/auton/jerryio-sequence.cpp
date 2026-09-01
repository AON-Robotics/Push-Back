#include "aon/auton/jerryio-sequence.hpp"

#include <array>

namespace aon::auton {
namespace {

constexpr std::array<JerryIoPhase, 6> kJerryIoOrder{{
    JerryIoPhase::FollowToIntake,
    JerryIoPhase::Intake,
    JerryIoPhase::FollowToOuttake,
    JerryIoPhase::Outtake,
    JerryIoPhase::FollowToPistons,
    JerryIoPhase::PulsePistons,
}};

}  // namespace

JerryIoSequenceResult runJerryIoSequence(JerryIoCallbacks& callbacks) {
  for (const JerryIoPhase phase : kJerryIoOrder) {
    if (!callbacks.run(phase)) {
      callbacks.stopAll();
      return {false, phase};
    }
  }
  callbacks.stopAll();
  return {true, JerryIoPhase::PulsePistons};
}

}  // namespace aon::auton
