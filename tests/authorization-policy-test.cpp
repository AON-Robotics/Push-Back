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

  CHECK(safeForUnvalidatedBaseline({false, false, false, false, false}));
  CHECK(!safeForUnvalidatedBaseline({true, false, false, false, false}));
  CHECK(!safeForUnvalidatedBaseline({false, true, false, false, false}));
  CHECK(!safeForUnvalidatedBaseline({false, false, true, false, false}));
  CHECK(!safeForUnvalidatedBaseline({false, false, false, true, false}));
  CHECK(!safeForUnvalidatedBaseline({false, false, false, false, true}));
}

void activeConfigurationIsSafeBeforePhysicalValidation() {
  const auto& config = aon::config::activeRobotConfig();
  const auto authorizations = aon::config::authorizationSnapshot(config);
  CHECK(aon::config::safeForUnvalidatedBaseline(authorizations));
}

}  // namespace

int main() {
  onlyAnEntirelyLockedSnapshotIsSafeBeforeValidation();
  activeConfigurationIsSafeBeforePhysicalValidation();
  std::cout << "authorization policy tests passed\n";
  return 0;
}
