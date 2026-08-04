#include "aon/auton/mechanism-actions.hpp"

#include "aon/constants.hpp"
#include "aon/core/hardware.hpp"

namespace aon::auton::mechanisms {

void beginLoaderCollection(aon::core::Hardware& hardware) {
  hardware.intake.activateScan();
}

void beginLoaderCollection() {
  beginLoaderCollection(aon::core::hardware());
}

void finishLoaderCollection(aon::core::Hardware& hardware) {
  hardware.intake.stopScan();
}

void finishLoaderCollection() {
  finishLoaderCollection(aon::core::hardware());
}

void prepareLoaderCart(aon::core::Hardware& hardware) {
  hardware.intake.dropCart();
}

void prepareLoaderCart() {
  prepareLoaderCart(aon::core::hardware());
}

void resetLoaderCart(aon::core::Hardware& hardware) {
  hardware.intake.raiseCart();
}

void resetLoaderCart() {
  resetLoaderCart(aon::core::hardware());
}

void prepareTopScorer(aon::core::Hardware& hardware) {
#if !USING_BIG_ROBOT
  hardware.intake.raiseScorer();
#else
  static_cast<void>(hardware);
#endif
}

void prepareTopScorer() {
  prepareTopScorer(aon::core::hardware());
}

void scoreTopBlocks(aon::core::Hardware& hardware,
                    std::uint32_t durationMs) {
  hardware.intake.score(Intake::TOP, static_cast<int>(durationMs));
}

void scoreTopBlocks(std::uint32_t durationMs) {
  scoreTopBlocks(aon::core::hardware(), durationMs);
}

void deployParkMechanism(aon::core::Hardware& hardware) {
  hardware.brooks.activate();
}

void deployParkMechanism() {
  deployParkMechanism(aon::core::hardware());
}

void stopAll(aon::core::Hardware& hardware) {
  hardware.intake.stop();
}

void stopAll() {
  stopAll(aon::core::hardware());
}

}  // namespace aon::auton::mechanisms
