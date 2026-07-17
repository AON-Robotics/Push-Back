#include "aon/auton/motion-health.hpp"
#include "aon/auton/fallback-geometry.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

using aon::auton::MotionFailureReason;
using aon::auton::MotionHealthMonitor;
using aon::auton::MotionIntent;
using aon::auton::MotionSample;
using aon::auton::TrustedPose;
using aon::auton::cappedFallbackOutput;
using aon::auton::fallbackBudget;
using aon::auton::headingFallback;
using aon::auton::motorDegreesForDistance;
using aon::auton::pointFallback;
using aon::auton::poseFallback;

namespace {

MotionSample healthy(std::uint32_t timeMs) {
  MotionSample sample{};
  sample.timeMs = timeMs;
  sample.poseValid = true;
  sample.leftMotorValid = true;
  sample.rightMotorValid = true;
  sample.leftTrackingValid = true;
  sample.rightTrackingValid = true;
  sample.backTrackingValid = true;
  sample.imuValid = true;
  return sample;
}

void checkNear(double actual, double expected, double tolerance = 1e-6) {
  CHECK(std::abs(actual - expected) <= tolerance);
}

void testFallbackGeometryUsesPositiveYAsHeadingZero() {
  const auto forward = pointFallback({0.0, 0.0, 0.0}, 0.0, 12.0);
  checkNear(forward.turnDegrees, 0.0);
  checkNear(forward.distanceInches, 12.0);

  const auto right = pointFallback({0.0, 0.0, 0.0}, 12.0, 0.0);
  checkNear(right.turnDegrees, 90.0);

  const auto behind = pointFallback({0.0, 0.0, 180.0}, 0.0, 12.0);
  checkNear(behind.turnDegrees, -180.0);
}

void testFallbackHeadingTakesShortestTurn() {
  checkNear(headingFallback(350.0, 10.0), 20.0);
  checkNear(headingFallback(10.0, 350.0), -20.0);

  const auto pose = poseFallback({0.0, 0.0, 0.0}, 0.0, 12.0, 90.0);
  checkNear(pose.finalTurnDegrees, 90.0);
}

void testMotorDegreesIncludeExternalGearRatio() {
  constexpr double kPi = 3.14159265358979323846;
  const double degrees = motorDegreesForDistance(12.0, 2.75, 0.75);
  checkNear(degrees, 12.0 / (kPi * 2.75 * 0.75) * 360.0);
}

void testFallbackOutputIsReducedFromRequestedMaximum() {
  CHECK(cappedFallbackOutput(100, 60) == 60);
  CHECK(cappedFallbackOutput(-100, 60) == -60);
  CHECK(cappedFallbackOutput(35, 60) == 21);
}

void testFallbackBudgetIsBoundedByRemainingTimeAndAllowance() {
  CHECK(fallbackBudget(1000, 1700, 1000, 250) == 550);
  CHECK(fallbackBudget(1000, 2500, 1000, 250) == 250);
}

void testInvalidSamplesRequireConfirmation() {
  MotionHealthMonitor monitor({});
  auto sample = healthy(0);
  monitor.reset(sample);

  sample = healthy(20);
  sample.leftTrackingValid = false;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);

  sample = healthy(40);
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);

  for (std::uint32_t timeMs : {60U, 80U}) {
    sample = healthy(timeMs);
    sample.leftTrackingValid = false;
    CHECK(monitor.observe(sample, MotionIntent::Linear) ==
          MotionFailureReason::None);
  }

  sample = healthy(100);
  sample.leftTrackingValid = false;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::DeviceInvalid);
}

void testFrozenTrackingRequiresMotorMovementAndDwell() {
  MotionHealthMonitor monitor({});
  monitor.reset(healthy(0));

  for (std::uint32_t timeMs = 20; timeMs < 300; timeMs += 20) {
    auto sample = healthy(timeMs);
    sample.leftMotorDegrees = static_cast<double>(timeMs) / 10.0;
    sample.rightMotorDegrees = static_cast<double>(timeMs) / 10.0;
    CHECK(monitor.observe(sample, MotionIntent::Linear) ==
          MotionFailureReason::None);
  }

  auto sample = healthy(300);
  sample.leftMotorDegrees = 30.0;
  sample.rightMotorDegrees = 30.0;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::FrozenTracking);
}

void testBlockedDriveDoesNotLookLikeFrozenTracking() {
  MotionHealthMonitor monitor({});
  monitor.reset(healthy(0));

  for (std::uint32_t timeMs = 20; timeMs <= 500; timeMs += 20) {
    CHECK(monitor.observe(healthy(timeMs), MotionIntent::Linear) ==
          MotionFailureReason::None);
  }
}

void testImpossiblePoseJumpRequiresConfirmation() {
  MotionHealthMonitor monitor({});
  monitor.reset(healthy(0));

  auto sample = healthy(20);
  sample.poseX = 9.0;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);

  sample = healthy(40);
  sample.poseX = 18.0;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::ImpossiblePoseJump);
}

}  // namespace

int main() {
  testFallbackGeometryUsesPositiveYAsHeadingZero();
  testFallbackHeadingTakesShortestTurn();
  testMotorDegreesIncludeExternalGearRatio();
  testFallbackOutputIsReducedFromRequestedMaximum();
  testFallbackBudgetIsBoundedByRemainingTimeAndAllowance();
  testInvalidSamplesRequireConfirmation();
  testFrozenTrackingRequiresMotorMovementAndDwell();
  testBlockedDriveDoesNotLookLikeFrozenTracking();
  testImpossiblePoseJumpRequiresConfirmation();
  std::cout << "motion fallback tests passed\n";
  return 0;
}
