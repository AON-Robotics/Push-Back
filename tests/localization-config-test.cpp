#include <cmath>
#include <cstdlib>
#include <iostream>

#include "aon/config/robot-config.hpp"
#include "aon/odometry/diagnostics.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

void localizationPolicyStartsSafeAndExplicit() {
  const aon::config::RobotConfig& config =
      aon::config::activeRobotConfig();

  CHECK(config.localization.loopPeriodMs == 10U);
  CHECK(config.localization.geometry.leftOffsetInches < 0.0);
  CHECK(config.localization.geometry.rightOffsetInches > 0.0);
  CHECK(config.localization.geometry.backOffsetInches < 0.0);
  CHECK(config.localization.trackingWheelDiameterInches > 0.0);
  CHECK(!config.localization.fusedLemLibAuthorized);
  CHECK(!config.localization.gps.enabled);
  CHECK(config.localization.gps.port == 0);
  CHECK(!config.localization.gps.headingUpdateEnabled);
  CHECK(config.localization.ekf.imuHeadingVariance > 0.0);

  CHECK(std::abs(config.lemlib.leftTrackingOffset -
                 config.localization.geometry.leftOffsetInches) < 1e-6);
  CHECK(std::abs(config.lemlib.rightTrackingOffset -
                 config.localization.geometry.rightOffsetInches) < 1e-6);
  CHECK(std::abs(config.lemlib.backTrackingOffset -
                 config.localization.geometry.backOffsetInches) < 1e-6);
}

void diagnosticsAreFixedValueState() {
  const aon::localization::LocalizationDiagnostics diagnostics{};
  CHECK(diagnostics.gpsRejectionReason ==
        aon::localization::GpsRejectionReason::None);
  CHECK(diagnostics.deadlineMisses == 0U);
  CHECK(diagnostics.numericalRejections == 0U);
  CHECK(std::isfinite(diagnostics.dtSeconds));
}

void knownBigRobotReversalMismatchRemainsVisible() {
  CHECK(aon::config::validateHardwareMap(
            aon::config::bigRobotHardwareMap) ==
        aon::config::HardwareMapIssue::RightTrackingReversalMismatch);
}

}  // namespace

int main() {
  localizationPolicyStartsSafeAndExplicit();
  diagnosticsAreFixedValueState();
  knownBigRobotReversalMismatchRemainsVisible();
  std::cout << "localization config tests passed\n";
  return 0;
}
