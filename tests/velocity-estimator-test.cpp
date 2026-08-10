#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "aon/localization/velocity-estimator.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

void bodyMotionBecomesFieldVelocityAtMidpointHeading() {
  using namespace aon::localization;
  VelocityEstimator estimator({1.0, 0.001, 0.2});
  CHECK(estimator.update({0.0, 10.0, 0.0, true}, 0.0, 0.1) ==
        VelocityUpdateResult::Accepted);
  CHECK(std::abs(estimator.velocity().xInchesPerSecond) < 1e-9);
  CHECK(std::abs(estimator.velocity().yInchesPerSecond - 100.0) < 1e-9);

  estimator.reset();
  CHECK(estimator.update({0.0, 10.0, 0.0, true}, kPi / 2.0, 0.1) ==
        VelocityUpdateResult::Accepted);
  CHECK(std::abs(estimator.velocity().xInchesPerSecond - 100.0) < 1e-9);
  CHECK(std::abs(estimator.velocity().yInchesPerSecond) < 1e-9);
}

void angularVelocityAndSmoothingRemainExplicit() {
  using namespace aon::localization;
  VelocityEstimator estimator({0.5, 0.001, 0.2});
  CHECK(estimator.update({0.0, 2.0, 0.2, true}, 0.0, 0.1) ==
        VelocityUpdateResult::Accepted);
  CHECK(std::abs(estimator.velocity().angularRadiansPerSecond - 2.0) < 1e-9);
  CHECK(estimator.update({0.0, 0.0, 0.0, true}, 0.2, 0.1) ==
        VelocityUpdateResult::Accepted);
  CHECK(std::abs(estimator.velocity().angularRadiansPerSecond - 1.0) < 1e-9);
}

void invalidTimingAndMotionDoNotChangeTheEstimate() {
  using namespace aon::localization;
  VelocityEstimator estimator({1.0, 0.01, 0.2});
  CHECK(estimator.update({0.0, 1.0, 0.0, true}, 0.0, 0.1) ==
        VelocityUpdateResult::Accepted);
  const Velocity2D before = estimator.velocity();
  CHECK(estimator.update({0.0, 1.0, 0.0, true}, 0.0, 0.0) ==
        VelocityUpdateResult::InvalidTiming);
  CHECK(estimator.update(
            {0.0, std::numeric_limits<double>::quiet_NaN(), 0.0, true},
            0.0, 0.1) == VelocityUpdateResult::InvalidMotion);
  CHECK(estimator.velocity().yInchesPerSecond == before.yInchesPerSecond);
}

}  // namespace

int main() {
  bodyMotionBecomesFieldVelocityAtMidpointHeading();
  angularVelocityAndSmoothingRemainExplicit();
  invalidTimingAndMotionDoNotChangeTheEstimate();
  std::cout << "velocity estimator tests passed\n";
  return 0;
}
