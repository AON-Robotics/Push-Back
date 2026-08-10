#include <cmath>
#include <cstdlib>
#include <iostream>

#include "aon/config/robot-config.hpp"
#include "aon/odometry/diagnostics.hpp"
#include "aon/tools/logging.hpp"

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
  CHECK(!config.localization.fusedNavigationAuthorized);
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

void diagnosticsCanBeFormattedOnlyWhenRequested() {
  aon::localization::LocalizationDiagnostics diagnostics{};
  diagnostics.timestampMs = 42U;
  diagnostics.rawPose = {1.0, 2.0, 0.25};
  diagnostics.fusedPose = {1.5, 2.5, 0.2};
  diagnostics.gpsPositionAccepted = true;

  constexpr const char* kOutputPath =
      "bin/host-tests/localization-diagnostics-test.csv";
  std::FILE* output = std::fopen(kOutputPath, "w+");
  CHECK(output != nullptr);
  aon::logging::WriteLocalizationCsvHeader(output);
  aon::logging::WriteLocalizationCsvRow(output, diagnostics);
  std::rewind(output);

  char line[512]{};
  CHECK(std::fgets(line, sizeof(line), output) != nullptr);
  CHECK(std::string(line).find("time_ms,raw_x,raw_y,raw_theta") == 0U);
  CHECK(std::fgets(line, sizeof(line), output) != nullptr);
  CHECK(std::string(line).find("42,1.000000,2.000000") == 0U);
  std::fclose(output);
  CHECK(std::remove(kOutputPath) == 0);
}

}  // namespace

int main() {
  localizationPolicyStartsSafeAndExplicit();
  diagnosticsAreFixedValueState();
  knownBigRobotReversalMismatchRemainsVisible();
  diagnosticsCanBeFormattedOnlyWhenRequested();
  std::cout << "localization config tests passed\n";
  return 0;
}
