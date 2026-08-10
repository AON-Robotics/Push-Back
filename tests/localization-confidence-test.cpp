#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "aon/localization/confidence.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

void covarianceMapsToExplicitDriveSafetyLevels() {
  using namespace aon::localization;
  const ConfidencePolicy policy{2.0, 6.0, radians(5.0), radians(20.0)};

  CHECK(classifyConfidence({1.0, 1.0, radians(2.0) * radians(2.0)}, policy) ==
        ConfidenceLevel::Normal);
  CHECK(classifyConfidence({9.0, 4.0, radians(10.0) * radians(10.0)}, policy) ==
        ConfidenceLevel::Degraded);
  CHECK(classifyConfidence({49.0, 1.0, radians(2.0) * radians(2.0)}, policy) ==
        ConfidenceLevel::StopRequired);
  CHECK(classifyConfidence(
            {std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0}, policy) ==
        ConfidenceLevel::StopRequired);
}

void largeCorrectionsNeedRepeatedIndependentAgreement() {
  using namespace aon::localization;
  RecoveryMonitor monitor({3, 2.0, radians(5.0), 300});

  CHECK(!monitor.observe({30.0, 20.0, radians(10.0), 100}).allowCorrection);
  CHECK(!monitor.observe({30.5, 19.5, radians(11.0), 150}).allowCorrection);
  const RecoveryDecision accepted =
      monitor.observe({29.5, 20.25, radians(9.0), 200});
  CHECK(accepted.allowCorrection);
  CHECK(accepted.consistentObservations == 3);

  monitor.reset();
  CHECK(!monitor.observe({30.0, 20.0, 0.0, 100}).allowCorrection);
  CHECK(!monitor.observe({50.0, 20.0, 0.0, 150}).allowCorrection);
  CHECK(monitor.observe({50.5, 20.0, 0.0, 200}).consistentObservations == 2);
  CHECK(!monitor.observe({50.5, 20.0, 0.0, 600}).allowCorrection);
}

void invalidAndOutOfOrderCandidatesFailClosed() {
  using namespace aon::localization;
  RecoveryMonitor monitor({2, 2.0, radians(5.0), 300});
  CHECK(monitor.observe({0.0, 0.0, 0.0, 100}).status ==
        RecoveryStatus::Collecting);
  CHECK(monitor.observe({0.0, 0.0, 0.0, 99}).status ==
        RecoveryStatus::OutOfOrder);
  CHECK(monitor.observe(
            {std::numeric_limits<double>::infinity(), 0.0, 0.0, 110})
            .status == RecoveryStatus::Invalid);
}

}  // namespace

int main() {
  covarianceMapsToExplicitDriveSafetyLevels();
  largeCorrectionsNeedRepeatedIndependentAgreement();
  invalidAndOutOfOrderCandidatesFailClosed();
  std::cout << "localization confidence tests passed\n";
  return 0;
}
