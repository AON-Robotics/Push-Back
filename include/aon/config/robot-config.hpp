#pragma once

#include <cstdint>

#include "aon/auton/motion-health.hpp"
#include "aon/config/hardware-map.hpp"

namespace aon::config {

enum class RobotIdentity {
  Small,
  Big,
};

struct FallbackConfig {
  double wheelRevolutionsPerMotorRevolution;
  double distanceKp;
  double turnKp;
  int minimumOutput;
  int maximumOutputPercent;
  std::uint32_t settleMs;
  std::uint32_t transitionAllowanceMs;
  aon::auton::HealthThresholds health;
  bool automaticFallbackAuthorized;
  bool forcedEncoderTestingAuthorized;
};

struct LemLibDriveConfig {
  DrivePorts motors;
  TrackingPorts trackingPorts;
  float driveWheelDiameter;
  float trackingWheelDiameter;
  float trackWidth;
  float drivetrainRpm;
  float horizontalDrift;
  float lateralSlew;
  float angularSlew;
  float leftTrackingOffset;
  float rightTrackingOffset;
  float backTrackingOffset;
  FallbackConfig fallback;
};

struct RobotConfig {
  RobotIdentity identity;
  LemLibDriveConfig lemlib;
  bool shadowPlaybackAuthorized;
};

/// Configuration selected by USING_BIG_ROBOT.
///
/// These values mirror the existing constants and ports. Moving them here does
/// not mean they are calibrated; tracking offsets and controller gains still
/// require physical validation.
const RobotConfig& activeRobotConfig();

}  // namespace aon::config
