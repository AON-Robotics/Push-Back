#include "aon/config/hardware-map.hpp"

#include <array>

namespace aon::config {
namespace {

int physicalPort(std::int8_t port) noexcept {
  const int value = port;
  return value < 0 ? -value : value;
}

bool isValidPort(std::int8_t port) noexcept {
  const int value = physicalPort(port);
  return value >= 1 && value <= 21;
}

bool allPortsAreValid(const RobotHardwareMap& map) noexcept {
  for (const std::int8_t port : map.drive.left) {
    if (!isValidPort(port)) return false;
  }
  for (const std::int8_t port : map.drive.right) {
    if (!isValidPort(port)) return false;
  }
  const std::array<std::int8_t, 8> trackingPorts{
      map.legacyTracking.left, map.legacyTracking.right,
      map.legacyTracking.back, map.legacyTracking.imu,
      map.lemlibTracking.left, map.lemlibTracking.right,
      map.lemlibTracking.back, map.lemlibTracking.imu,
  };
  for (const std::int8_t port : trackingPorts) {
    if (!isValidPort(port)) return false;
  }
  return true;
}

}  // namespace

HardwareMapIssue validateHardwareMap(const RobotHardwareMap& map) noexcept {
  if (!allPortsAreValid(map)) return HardwareMapIssue::InvalidPort;

  if (physicalPort(map.legacyTracking.left) !=
      physicalPort(map.lemlibTracking.left)) {
    return HardwareMapIssue::LeftTrackingPortMismatch;
  }
  if (physicalPort(map.legacyTracking.right) !=
      physicalPort(map.lemlibTracking.right)) {
    return HardwareMapIssue::RightTrackingPortMismatch;
  }
  if (physicalPort(map.legacyTracking.back) !=
      physicalPort(map.lemlibTracking.back)) {
    return HardwareMapIssue::BackTrackingPortMismatch;
  }
  if (physicalPort(map.legacyTracking.imu) !=
      physicalPort(map.lemlibTracking.imu)) {
    return HardwareMapIssue::ImuPortMismatch;
  }
  if ((map.legacyTracking.left < 0) !=
      map.lemlibTracking.leftReversed) {
    return HardwareMapIssue::LeftTrackingReversalMismatch;
  }
  if ((map.legacyTracking.right < 0) !=
      map.lemlibTracking.rightReversed) {
    return HardwareMapIssue::RightTrackingReversalMismatch;
  }
  if ((map.legacyTracking.back < 0) !=
      map.lemlibTracking.backReversed) {
    return HardwareMapIssue::BackTrackingReversalMismatch;
  }
  return HardwareMapIssue::None;
}

}  // namespace aon::config
