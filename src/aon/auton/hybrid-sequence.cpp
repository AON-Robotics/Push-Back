#include "aon/auton/hybrid-sequence.hpp"

#include <array>

namespace aon::auton {
namespace {

constexpr std::array<RedSixPhase, 8> kRedSixOrder{{
    RedSixPhase::LoaderPursuit,
    RedSixPhase::LoaderContact,
    RedSixPhase::CollectSix,
    RedSixPhase::ReverseClearance,
    RedSixPhase::ReverseAlignment,
    RedSixPhase::GoalPursuit,
    RedSixPhase::GoalContact,
    RedSixPhase::ScoreSix,
}};

bool isValidPhase(RedSixPhase phase) {
  for (const RedSixPhase candidate : kRedSixOrder) {
    if (candidate == phase) return true;
  }
  return false;
}

}  // namespace

RedSixPhaseResult runRedSixSequence(RedSixCallbacks& callbacks,
                                    RedSixPhase stopAfter) {
  if (!isValidPhase(stopAfter)) {
    callbacks.stopAll();
    return {false, stopAfter};
  }

  for (const RedSixPhase phase : kRedSixOrder) {
    if (!callbacks.run(phase)) {
      callbacks.stopAll();
      return {false, phase};
    }
    if (phase == stopAfter) {
      callbacks.stopAll();
      return {true, phase};
    }
  }

  callbacks.stopAll();
  return {true, RedSixPhase::ScoreSix};
}

}  // namespace aon::auton
