#include "aon/drivetrain/legacy-motion-safety.hpp"

#include <cassert>

int main() {
  using aon::legacy_motion::shouldContinue;

  assert(shouldContinue(false, false, false));
  assert(!shouldContinue(true, false, false));
  assert(!shouldContinue(false, true, false));
  assert(!shouldContinue(false, false, true));
  assert(!shouldContinue(true, true, true));
  return 0;
}
