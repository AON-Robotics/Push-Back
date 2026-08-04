#include "aon/config/hardware-map.hpp"
#include "aon/config/robot-config.hpp"

#include <array>
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

namespace {

using aon::config::HardwareMapIssue;
using aon::config::RobotHardwareMap;

void smallRobotValuesRemainUnchanged() {
  constexpr std::array<std::int8_t, 4> expectedLeft{11, -12, 13, -14};
  constexpr std::array<std::int8_t, 4> expectedRight{1, -2, 3, -4};
  const RobotHardwareMap& map = aon::config::smallRobotHardwareMap;

  CHECK(map.drive.left == expectedLeft);
  CHECK(map.drive.right == expectedRight);
  CHECK(map.legacyTracking.left == 19);
  CHECK(map.legacyTracking.right == -18);
  CHECK(map.legacyTracking.back == 5);
  CHECK(map.legacyTracking.imu == 16);
  CHECK(map.lemlibTracking.left == 19);
  CHECK(map.lemlibTracking.right == 18);
  CHECK(map.lemlibTracking.back == 5);
  CHECK(map.lemlibTracking.imu == 16);
  CHECK(!map.lemlibTracking.leftReversed);
  CHECK(map.lemlibTracking.rightReversed);
  CHECK(!map.lemlibTracking.backReversed);
  CHECK(aon::config::validateHardwareMap(map) == HardwareMapIssue::None);
}

void bigRobotValuesAndKnownMismatchRemainVisible() {
  constexpr std::array<std::int8_t, 4> expectedLeft{12, -13, -18, 19};
  constexpr std::array<std::int8_t, 4> expectedRight{-1, 2, 3, -4};
  const RobotHardwareMap& map = aon::config::bigRobotHardwareMap;

  CHECK(map.drive.left == expectedLeft);
  CHECK(map.drive.right == expectedRight);
  CHECK(map.legacyTracking.left == 5);
  CHECK(map.legacyTracking.right == -6);
  CHECK(map.legacyTracking.back == 7);
  CHECK(map.legacyTracking.imu == 14);
  CHECK(map.lemlibTracking.left == 5);
  CHECK(map.lemlibTracking.right == -6);
  CHECK(map.lemlibTracking.back == 7);
  CHECK(map.lemlibTracking.imu == 14);
  CHECK(!map.lemlibTracking.leftReversed);
  CHECK(!map.lemlibTracking.rightReversed);
  CHECK(!map.lemlibTracking.backReversed);
  CHECK(aon::config::validateHardwareMap(map) ==
        HardwareMapIssue::RightTrackingReversalMismatch);
}

void validatorDistinguishesPortAndReversalFailures() {
  RobotHardwareMap wrongPort = aon::config::smallRobotHardwareMap;
  wrongPort.lemlibTracking.right = 17;
  CHECK(aon::config::validateHardwareMap(wrongPort) ==
        HardwareMapIssue::RightTrackingPortMismatch);

  RobotHardwareMap wrongReversal = aon::config::smallRobotHardwareMap;
  wrongReversal.lemlibTracking.rightReversed = false;
  CHECK(aon::config::validateHardwareMap(wrongReversal) ==
        HardwareMapIssue::RightTrackingReversalMismatch);
}

void validatorRejectsPortsOutsideTheV5Range() {
  RobotHardwareMap zeroPort = aon::config::smallRobotHardwareMap;
  zeroPort.lemlibTracking.left = 0;
  CHECK(aon::config::validateHardwareMap(zeroPort) ==
        HardwareMapIssue::InvalidPort);

  RobotHardwareMap highPort = aon::config::smallRobotHardwareMap;
  highPort.legacyTracking.back = -22;
  CHECK(aon::config::validateHardwareMap(highPort) ==
        HardwareMapIssue::InvalidPort);
}

void experimentalRoutesRequireIndividualAuthorization() {
  using aon::config::AutonomousAuthorizations;
  using aon::config::ExperimentalRoute;

  const AutonomousAuthorizations locked{false, false};
  CHECK(!locked.allows(ExperimentalRoute::RedSixBlock));
  CHECK(!locked.allows(ExperimentalRoute::JerryIoPath));

  const AutonomousAuthorizations redOnly{true, false};
  CHECK(redOnly.allows(ExperimentalRoute::RedSixBlock));
  CHECK(!redOnly.allows(ExperimentalRoute::JerryIoPath));

  const AutonomousAuthorizations jerryOnly{false, true};
  CHECK(!jerryOnly.allows(ExperimentalRoute::RedSixBlock));
  CHECK(jerryOnly.allows(ExperimentalRoute::JerryIoPath));
}

}  // namespace

int main() {
  smallRobotValuesRemainUnchanged();
  bigRobotValuesAndKnownMismatchRemainVisible();
  validatorDistinguishesPortAndReversalFailures();
  validatorRejectsPortsOutsideTheV5Range();
  experimentalRoutesRequireIndividualAuthorization();
  std::cout << "hardware map tests passed\n";
  return 0;
}
