#pragma once

#include <cstdint>
#include <functional>

namespace aon::auton {

enum class RedSixPhase : std::uint8_t {
  LoaderPursuit,
  LoaderContact,
  CollectSix,
  ReverseClearance,
  ReverseAlignment,
  GoalPursuit,
  GoalContact,
  ScoreSix,
};

struct RedSixPhaseResult {
  bool succeeded;
  RedSixPhase failedPhase;
};

struct RedSixCallbacks {
  std::function<bool(RedSixPhase)> run;
  std::function<void()> stopAll;
};

/** Runs the red six-block phases in order and always stops all outputs. */
RedSixPhaseResult runRedSixSequence(
    RedSixCallbacks& callbacks,
    RedSixPhase stopAfter = RedSixPhase::ScoreSix);

}  // namespace aon::auton
