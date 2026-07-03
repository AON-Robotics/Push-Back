#pragma once

#include <array>
#include <cstdint>

namespace aon::config {

enum class RobotIdentity {
  Small,
  Big,
};

struct DrivePorts {
  std::array<std::int8_t, 4> left;
  std::array<std::int8_t, 4> right;
};

struct TrackingPorts {
  std::int8_t left;
  std::int8_t right;
  std::int8_t back;
  std::int8_t imu;
  bool leftReversed;
  bool rightReversed;
  bool backReversed;
};

struct LemLibDriveConfig {
  DrivePorts motors;
  TrackingPorts trackingPorts;
  float driveWheelDiameter;
  float trackingWheelDiameter;
  float trackWidth;
  float drivetrainRpm;
  float horizontalDrift;
  float leftTrackingOffset;
  float rightTrackingOffset;
  float backTrackingOffset;
};

struct RobotConfig {
  RobotIdentity identity;
  LemLibDriveConfig lemlib;
};

/// Configuration selected by USING_BIG_ROBOT.
///
/// These values mirror the existing constants and ports. Moving them here does
/// not mean they are calibrated; tracking offsets and controller gains still
/// require physical validation.
const RobotConfig& activeRobotConfig();

}  // namespace aon::config
