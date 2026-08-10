#include <cmath>
#include <cstdlib>
#include <iostream>

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

}  // namespace

int main() {
  angleConversionsAndWrappingAreConsistent();
  sincIsStableNearZero();
  measurementTypesCarryValuesAndValidity();
  std::cout << "localization math tests passed\n";
  return 0;
}
