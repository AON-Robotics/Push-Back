#pragma once

#include <cstdint>

#include "aon/auton/motion-health.hpp"
#include "aon/config/hardware-map.hpp"
#include "aon/odometry/ekf.hpp"
#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"

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

/** Snapshot of every behavior that remains gated by physical validation. */
struct AuthorizationSnapshot {
  bool automaticEncoderFallback;
  bool forcedEncoderTesting;
  bool shadowPlayback;
  bool redSixBlock;
  bool jerryIoPath;
};

/** Returns the locked authorization baseline for either physical robot. */
[[nodiscard]] constexpr AuthorizationSnapshot baselineAuthorizations(
    RobotIdentity) noexcept {
  return {};
}

/**
 * @brief Reports whether no unvalidated behavior is enabled.
 *
 * This pure policy is shared by configuration construction and host checks. It
 * does not grant authorization; it only proves that the enumerated gates are
 * shut.
 */
[[nodiscard]] constexpr bool safeForUnvalidatedBaseline(
    const AuthorizationSnapshot& value) noexcept {
  return !value.automaticEncoderFallback && !value.forcedEncoderTesting &&
         !value.shadowPlayback && !value.redSixBlock && !value.jerryIoPath;
}

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

struct GpsHardwareConfig {
  bool enabled;
  std::int8_t port;
  double xOffsetMeters;
  double yOffsetMeters;
  double headingOffsetDegrees;
  bool headingUpdateEnabled;
  aon::localization::GpsValidationConfig validation;
};

struct LocalizationConfig {
  aon::localization::TrackingGeometry geometry;
  double trackingWheelDiameterInches;
  std::uint32_t loopPeriodMs;
  aon::localization::EkfConfig ekf;
  GpsHardwareConfig gps;
  bool fusedLemLibAuthorized;
  bool fusedNavigationAuthorized;
};

struct RobotConfig {
  RobotIdentity identity;
  LemLibDriveConfig lemlib;
  LocalizationConfig localization;
  bool shadowPlaybackAuthorized;
  AutonomousAuthorizations autonomousAuthorizations;
};

/** Returns the physical-validation gates currently active in a robot config. */
[[nodiscard]] constexpr AuthorizationSnapshot authorizationSnapshot(
    const RobotConfig& config) noexcept {
  return {
      config.lemlib.fallback.automaticFallbackAuthorized,
      config.lemlib.fallback.forcedEncoderTestingAuthorized,
      config.shadowPlaybackAuthorized,
      config.autonomousAuthorizations.allows(ExperimentalRoute::RedSixBlock),
      config.autonomousAuthorizations.allows(ExperimentalRoute::JerryIoPath),
  };
}

/// Configuration selected by USING_BIG_ROBOT.
///
/// These values mirror the existing constants and ports. Moving them here does
/// not mean they are calibrated; tracking offsets and controller gains still
/// require physical validation.
const RobotConfig& activeRobotConfig();

}  // namespace aon::config
