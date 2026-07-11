#include "aon/auton/mechanism-actions.hpp"

#include "aon/constants.hpp"
#include "aon/globals.hpp"

namespace aon::auton::mechanisms {

void beginLoaderCollection() {
  intake.activateScan();
}

void finishLoaderCollection() {
  intake.stopScan();
}

void prepareLoaderCart() {
  intake.dropCart();
}

void resetLoaderCart() {
  intake.raiseCart();
}

void prepareTopScorer() {
#if !USING_BIG_ROBOT
  intake.raiseScorer();
#endif
}

void scoreTopBlocks(std::uint32_t durationMs) {
  intake.score(Intake::TOP, static_cast<int>(durationMs));
}

void deployParkMechanism() {
  brooks.activate();
}

void stopAll() {
  intake.stop();
}

}  // namespace aon::auton::mechanisms
