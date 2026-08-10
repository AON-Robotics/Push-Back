#include "aon/odometry/ekf.hpp"
#include "aon/localization/wall-observation.hpp"

#include <cmath>
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

namespace {

aon::localization::EkfConfig config() {
  const double headingVariance =
      aon::localization::radians(5.0) * aon::localization::radians(5.0);
  return {4.0, headingVariance, 1e-6, 1e-8, 0.01, 0.01,
          headingVariance, 4.0, headingVariance, 1e-12};
}

void wallAxisCorrectionContractsOnlyObservedPositionUncertainty() {
  using namespace aon::localization;

  Ekf ekf(config());
  ekf.reset({0.0, 0.0, 0.0});
  const CovarianceDiagonal before = ekf.covarianceDiagonal();
  CHECK(ekf.updateAxisPosition(PositionAxis::X, 1.0, 0.25, 10.0));
  CHECK(ekf.pose().xInches > 0.0 && ekf.pose().xInches < 1.0);
  CHECK(std::abs(ekf.pose().yInches) < 1e-12);
  const CovarianceDiagonal after = ekf.covarianceDiagonal();
  CHECK(after.xVariance < before.xVariance);
  CHECK(std::abs(after.yVariance - before.yVariance) < 1e-12);

  const EstimatorPose accepted = ekf.pose();
  CHECK(!ekf.updateAxisPosition(PositionAxis::Y, 100.0, 0.25, 1.0));
  CHECK(ekf.pose().xInches == accepted.xInches);
  CHECK(ekf.pose().yInches == accepted.yInches);
}

void lidarWallObservationUsesTheStandardEstimatorGate() {
  using namespace aon::localization;

  Ekf ekf(config());
  ekf.reset({0.0, 0.0, 0.0});
  WallObservationAdapter adapter(50);
  const aon::lidar::WallObservation observation{
      aon::lidar::WallAxis::Y, 1.0, 0.25, 0.1, 6, 100};
  const WallCorrectionResult result =
      adapter.apply(ekf, observation, 10.0, 100);
  CHECK(result == WallCorrectionResult::Accepted);
  CHECK(ekf.pose().yInches > 0.0);
  CHECK(adapter.apply(ekf, observation, 10.0, 100) ==
        WallCorrectionResult::Duplicate);
  aon::lidar::WallObservation stale = observation;
  stale.captureTimestampMs = 120;
  CHECK(adapter.apply(ekf, stale, 10.0, 200) ==
        WallCorrectionResult::Stale);
}

}  // namespace

int main() {
  wallAxisCorrectionContractsOnlyObservedPositionUncertainty();
  lidarWallObservationUsesTheStandardEstimatorGate();
  std::cout << "wall observation tests passed\n";
  return 0;
}
