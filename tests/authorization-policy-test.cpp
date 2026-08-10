#include "aon/config/robot-config.hpp"

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

void onlyAnEntirelyLockedSnapshotIsSafeBeforeValidation() {
  using aon::config::safeForUnvalidatedBaseline;

  const aon::config::AuthorizationSnapshot locked{};
  CHECK(safeForUnvalidatedBaseline(locked));

  auto enabled = locked;
  enabled.automaticEncoderFallback = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.forcedEncoderTesting = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.shadowPlayback = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.redSixBlock = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.jerryIoPath = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.gpsHardware = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.gpsHeadingFusion = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.fusedLemLib = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
  enabled = locked;
  enabled.fusedNavigation = true;
  CHECK(!safeForUnvalidatedBaseline(enabled));
}

void bothRobotBaselinesKeepEveryPhysicalGateLocked() {
  using aon::config::RobotIdentity;

  const auto small = aon::config::baselineAuthorizations(RobotIdentity::Small);
  const auto big = aon::config::baselineAuthorizations(RobotIdentity::Big);
  CHECK(!small.shadowPlayback);
  CHECK(!small.gpsHardware);
  CHECK(!small.gpsHeadingFusion);
  CHECK(!small.fusedLemLib);
  CHECK(!small.fusedNavigation);
  CHECK(aon::config::safeForUnvalidatedBaseline(small));
  CHECK(aon::config::safeForUnvalidatedBaseline(big));
}

void activeConfigurationIsSafeBeforePhysicalValidation() {
  const auto& config = aon::config::activeRobotConfig();
  const auto authorizations = aon::config::authorizationSnapshot(config);
  CHECK(aon::config::safeForUnvalidatedBaseline(authorizations));
}

}  // namespace

int main() {
  onlyAnEntirelyLockedSnapshotIsSafeBeforeValidation();
  bothRobotBaselinesKeepEveryPhysicalGateLocked();
  activeConfigurationIsSafeBeforePhysicalValidation();
  std::cout << "authorization policy tests passed\n";
  return 0;
}
