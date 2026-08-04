#pragma once

#include <cstdint>

#include "aon/auton/motion-health.hpp"
#include "aon/config/hardware-map.hpp"

namespace aon::config {

enum class RobotIdentity {
  Small,
  Big,
};

/** Experimental autonomous routes that require individual field validation. */
enum class ExperimentalRoute {
  RedSixBlock,
  JerryIoPath,
};

/**
 * @brief Explicit execution gates for autonomous routes not yet field-proven.
 *
 * A route must remain disabled until its physical checklist has passed. These
 * flags affect autonomous execution only; they do not enable hardware or
 * bypass the robot identity checks performed by each routine.
 */
struct AutonomousAuthorizations {
  bool redSixBlock;
  bool jerryIoPath;

  /** Returns whether the specified route may issue autonomous commands. */
  [[nodiscard]] constexpr bool allows(ExperimentalRoute route) const noexcept {
    switch (route) {
      case ExperimentalRoute::RedSixBlock:
        return redSixBlock;
      case ExperimentalRoute::JerryIoPath:
        return jerryIoPath;
    }
    return false;
  }
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
  AutonomousAuthorizations autonomousAuthorizations;
};

/// Configuration selected by USING_BIG_ROBOT.
///
/// These values mirror the existing constants and ports. Moving them here does
/// not mean they are calibrated; tracking offsets and controller gains still
/// require physical validation.
const RobotConfig& activeRobotConfig();

}  // namespace aon::config
