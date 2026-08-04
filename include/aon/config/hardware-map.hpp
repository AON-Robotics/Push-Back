#pragma once

#include <array>
#include <cstdint>

namespace aon::config {

/**
 * @brief Signed smart-port assignments for the left and right drive motors.
 *
 * @details Each absolute value is a V5 smart port in the range [1, 21]. A
 * negative value preserves the existing motor-reversal convention. This is
 * non-owning configuration data and does not initialize or command motors.
 */
struct DrivePorts {
  std::array<std::int8_t, 4> left;
  std::array<std::int8_t, 4> right;
};

/**
 * @brief Tracking-device ports and final LemLib reversal states.
 *
 * @details Port signs are preserved as supplied to PROS constructors. The
 * reversal booleans are the final states applied by `chassis.cpp` after
 * construction. All absolute port values must be in the range [1, 21].
 */
struct TrackingPorts {
  std::int8_t left;
  std::int8_t right;
  std::int8_t back;
  std::int8_t imu;
  bool leftReversed;
  bool rightReversed;
  bool backReversed;
};

/**
 * @brief Signed tracking ports passed to the legacy odometry constructor.
 *
 * @details Legacy odometry uses the absolute value as the physical V5 port
 * and derives encoder reversal from a negative sign. Values are non-owning
 * configuration with no device-lifetime implications.
 */
struct LegacyTrackingPorts {
  std::int8_t left;
  std::int8_t right;
  std::int8_t back;
  std::int8_t imu;
};

/**
 * @brief Hardware port values shared by legacy and LemLib construction.
 *
 * @details The drive map has one representation because both consumers use
 * the same signed-port convention. Tracking representations remain distinct
 * so existing behavior can be preserved and semantic disagreement reported.
 */
struct RobotHardwareMap {
  DrivePorts drive;
  LegacyTrackingPorts legacyTracking;
  TrackingPorts lemlibTracking;
};

/** @brief Result of validating legacy and LemLib hardware-map agreement. */
enum class HardwareMapIssue : std::uint8_t {
  None,                              ///< All checked values agree.
  InvalidPort,                       ///< An absolute V5 port is outside [1, 21].
  LeftTrackingPortMismatch,          ///< Left physical ports differ.
  RightTrackingPortMismatch,         ///< Right physical ports differ.
  BackTrackingPortMismatch,          ///< Back physical ports differ.
  ImuPortMismatch,                   ///< IMU physical ports differ.
  LeftTrackingReversalMismatch,      ///< Left final directions differ.
  RightTrackingReversalMismatch,     ///< Right final directions differ.
  BackTrackingReversalMismatch,      ///< Back final directions differ.
};

/**
 * @brief Current small-robot drivetrain and tracking-device port map.
 *
 * @warning These values describe existing wiring and reversal behavior; they
 * are not calibration results or authorization to command hardware.
 */
inline constexpr RobotHardwareMap smallRobotHardwareMap{
    {{{11, -12, 13, -14}}, {{1, -2, 3, -4}}},
    {19, -18, 5, 16},
    {19, 18, 5, 16, false, true, false},
};

/**
 * @brief Current big-robot drivetrain and tracking-device port map.
 *
 * @warning The legacy and LemLib right-tracker reversal values intentionally
 * preserve a known mismatch pending physical validation. Do not treat this
 * configuration value as authorization to run a calibration or route.
 */
inline constexpr RobotHardwareMap bigRobotHardwareMap{
    {{{12, -13, -18, 19}}, {{-1, 2, 3, -4}}},
    {5, -6, 7, 14},
    {5, -6, 7, 14, false, false, false},
};

/**
 * @brief Validates V5 port ranges and tracking semantics without using hardware.
 *
 * @param map Required configuration value; the function does not retain it.
 * @return The first detected issue, or HardwareMapIssue::None.
 *
 * @pre `map` remains alive for the duration of the call.
 * @post `map` is unchanged.
 * @note This function is deterministic, thread-safe, allocation-free, and has
 * no hardware, logging, timing, or authorization side effects.
 * @note Failure is reported only through the return value; no exception is
 * thrown and no correction is applied.
 */
[[nodiscard]] HardwareMapIssue validateHardwareMap(
    const RobotHardwareMap& map) noexcept;

}  // namespace aon::config
