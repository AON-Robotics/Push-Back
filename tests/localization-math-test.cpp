#include <cmath>
#include <cstdlib>
#include <iostream>

#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

constexpr double kTolerance = 1e-9;

void checkNear(double actual, double expected,
               double tolerance = kTolerance) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

void angleConversionsAndWrappingAreConsistent() {
  using namespace aon::localization;

  checkNear(wrapRadians(radians(359.0)), radians(-1.0));
  checkNear(shortestAngleDelta(radians(359.0), radians(1.0)),
            radians(2.0));
  checkNear(shortestAngleDelta(radians(-179.0), radians(179.0)),
            radians(-2.0));
  checkNear(degrees(radians(90.0)), 90.0);
}

void sincIsStableNearZero() {
  using namespace aon::localization;

  checkNear(sinc(0.0), 1.0);
  checkNear(sinc(1e-9), 1.0, 1e-12);
}

void measurementTypesCarryValuesAndValidity() {
  using namespace aon::localization;

  const WheelDistances wheels{1.0, 2.0, 3.0, true, true, true};
  const ImuMeasurement imu{radians(10.0), true};
  const GpsMeasurement gps{12.0, 24.0, radians(90.0), 1.5,
                           true, true, true, 100U};

  CHECK(wheels.leftInches == 1.0);
  CHECK(wheels.backValid && imu.valid && gps.fresh);
  CHECK(gps.timestampMs == 100U);
}

void checkMotion(const aon::localization::LocalMotion& actual,
                 double expectedRight, double expectedForward,
                 double expectedHeading) {
  checkNear(actual.rightInches, expectedRight);
  checkNear(actual.forwardInches, expectedForward);
  checkNear(actual.headingRadians, expectedHeading);
}

void checkPose(const aon::localization::EstimatorPose& actual,
               double expectedX, double expectedY, double expectedHeading) {
  checkNear(actual.xInches, expectedX);
  checkNear(actual.yInches, expectedY);
  checkNear(actual.headingRadians, expectedHeading);
}

void threeWheelMotionUsesSignedOffsets() {
  using namespace aon::localization;

  const TrackingGeometry geometry{-4.0, 4.0, -3.0};
  checkMotion(localMotion({0.0, 0.0, 0.0, true, true, true}, geometry),
              0.0, 0.0, 0.0);
  checkMotion(localMotion({12.0, 12.0, 0.0, true, true, true}, geometry),
              0.0, 12.0, 0.0);
  checkMotion(localMotion({-12.0, -12.0, 0.0, true, true, true}, geometry),
              0.0, -12.0, 0.0);

  const double quarterTurn = kPi / 2.0;
  checkMotion(localMotion({4.0 * quarterTurn, -4.0 * quarterTurn,
                           -3.0 * quarterTurn, true, true, true},
                          geometry),
              0.0, 0.0, quarterTurn);
  checkMotion(localMotion({0.0, 0.0, 5.0, true, true, true}, geometry),
              5.0, 0.0, 0.0);

  const LocalMotion withoutBack =
      localMotion({3.0, 3.0, 100.0, true, true, false}, geometry);
  checkMotion(withoutBack, 0.0, 3.0, 0.0);
  CHECK(!withoutBack.lateralValid);

  const LocalMotion invalidGeometry =
      localMotion({1.0, 1.0, 0.0, true, true, true},
                  {2.0, 2.0, 1.0});
  CHECK(!std::isfinite(invalidGeometry.headingRadians));
}

void posePropagationMatchesTheFieldConvention() {
  using namespace aon::localization;

  checkPose(propagatePose({0.0, 0.0, 0.0},
                          {0.0, 12.0, 0.0, true}),
            0.0, 12.0, 0.0);
  checkPose(propagatePose({0.0, 0.0, kPi / 2.0},
                          {0.0, 12.0, 0.0, true}),
            12.0, 0.0, kPi / 2.0);
  checkPose(propagatePose({0.0, 0.0, 0.0},
                          {6.0, 0.0, 0.0, true}),
            6.0, 0.0, 0.0);

  constexpr double radius = 10.0;
  const double quarterTurn = kPi / 2.0;
  checkPose(propagatePose({0.0, 0.0, 0.0},
                          {0.0, radius * quarterTurn, quarterTurn, true}),
            radius, radius, quarterTurn);

  const LocalMotion combined{2.0, 6.0, kPi / 3.0, true};
  const double halfTurn = combined.headingRadians / 2.0;
  const double scale = sinc(halfTurn);
  checkPose(propagatePose({0.0, 0.0, 0.0}, combined),
            scale * (combined.rightInches * std::cos(halfTurn) +
                     combined.forwardInches * std::sin(halfTurn)),
            scale * (-combined.rightInches * std::sin(halfTurn) +
                     combined.forwardInches * std::cos(halfTurn)),
            combined.headingRadians);
}

}  // namespace

int main() {
  angleConversionsAndWrappingAreConsistent();
  sincIsStableNearZero();
  measurementTypesCarryValuesAndValidity();
  threeWheelMotionUsesSignedOffsets();
  posePropagationMatchesTheFieldConvention();
  std::cout << "localization math tests passed\n";
  return 0;
}
