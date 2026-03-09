#ifndef AON_CONTROLS_POSE_LOCK_HPP__
#define AON_CONTROLS_POSE_LOCK_HPP__

#include <cmath>
#include "./pid/pid.hpp"
#include "../drivetrain.hpp"
#include "../odometry/odometry.hpp"
#include "../tools/vector.hpp"

namespace aon {

// ============================================================================
//    ____                 _                _
//   |  _ \ ___  ___  ___| |    ___   ___| | __
//   | |_) / _ \/ __|/ _ \ |   / _ \ / __| |/ /
//   |  __/ (_) \__ \  __/ |__| (_) | (__|   <
//   |_|   \___/|___/\___|_____\___/ \___|_|\_\          .
//
// ============================================================================

// ============================= Enums ========================================

/// @brief The internal state of the PoseLock state machine
enum class PoseLockState {
  IDLE,       ///< Not active, waiting for setTarget()
  SETTLING,   ///< Actively correcting pose toward the target
  FINISHED    ///< Within tolerance for the required number of cycles
};

/// @brief Sequential settling stages for the tank drive state machine
///
/// @details Tank drives cannot strafe, so precision alignment is performed
///          in three sequential phases: face the target, drive to it, then
///          correct the final heading.
enum class TankStage {
  IDLE,             ///< Not actively settling
  ALIGN_HEADING,    ///< Turn to face the target position
  DRIVE_TO_TARGET,  ///< Drive toward the target while maintaining heading
  FINAL_HEADING,    ///< Correct to the desired final heading
  DONE              ///< All stages complete
};

// ============================= Structs ======================================

/// @brief Configuration for PoseLock tolerances and motor limits
///
/// @details Tune these values per-robot to match physical characteristics.
///          `minVelocity` is critical — set it to the minimum RPM that
///          reliably overcomes static friction on your drivetrain.
struct PoseLockConfig {
  /// Position tolerance in \b inches — below this, position is "settled"
  double linearTolerance = 0.5;

  /// Angular tolerance in \b radians — below this, heading is "settled"
  double angularTolerance = 0.03;  // ~1.7 degrees

  /// Consecutive `update()` cycles within tolerance before declaring finished
  int settledCycles = 10;

  /// Minimum motor velocity in \b RPM to overcome static friction
  /// @note Set this to the lowest RPM that reliably moves your specific robot
  double minVelocity = 15.0;

  /// Maximum motor velocity in \b RPM to clamp PID output
  double maxVelocity = 200.0;

  /// PID output below this threshold (in RPM) is treated as zero
  double deadband = 3.0;

  /// Maximum time in \b milliseconds before auto-finishing (safety timeout)
  double timeout = 3000.0;
};

// ============================= PoseLock Class ===============================

/// @brief Precision alignment controller for the final stage of a motion
///        profile. Locks the robot onto a target pose using PID.
///
/// @details Supports both holonomic (X-Drive) and tank drive configurations
///          selected via the DriveType enum. Designed to be invoked at the
///          final stage of a High Precision Profile (HPP) for sub-inch
///          accuracy positioning.
///
/// @par Tank Drive Behavior (Sequential State Machine):
///   1. **ALIGN_HEADING** — Turn to face the target position
///   2. **DRIVE_TO_TARGET** — Drive forward while correcting heading
///   3. **FINAL_HEADING** — Rotate to achieve the desired final heading
///
/// @par Holonomic Drive Behavior (Simultaneous 3-DOF):
///   Corrects forward, strafe, and heading errors all at once, leveraging
///   the omnidirectional capability of the X-Drive.
///
/// @par Usage Example:
/// @code{.cpp}
/// // === Tank Drive ===
/// aon::PoseLock lock(drivetrain,
///                    aon::PID(2.0, 0, 0.1),   // xPid
///                    aon::PID(1.5, 0, 0.05));  // thetaPid
/// lock.setTarget(aon::Pose(24.0, 48.0, M_PI / 2));
/// while (!lock.isFinished()) {
///   lock.update();
///   pros::delay(10);
/// }
///
/// // === Holonomic (X-Drive) ===
/// aon::PoseLock lock(xdrive,
///                    aon::PID(2.0, 0, 0.1),   // xPid
///                    aon::PID(2.0, 0, 0.1),   // yPid
///                    aon::PID(1.5, 0, 0.05)); // thetaPid
/// lock.setTarget(aon::Pose(24.0, 48.0, M_PI / 2));
/// while (!lock.isFinished()) {
///   lock.update();
///   pros::delay(10);
/// }
/// @endcode
class PoseLock {
 private:
  // === Configuration ===
  PoseLockState state;
  PoseLockConfig config;

  // === PID Controllers ===
  PID xPid;       ///< Distance (tank) or forward-axis (holonomic)
  PID yPid;       ///< Strafe-axis (holonomic only, unused in tank)
  PID thetaPid;   ///< Angular correction

  // === Target ===
  Pose targetPose;

  // === Drive Reference ===
  Drivetrain* drive = nullptr;

  // === State Tracking ===
  int settledCounter = 0;
  uint32_t startTime = 0;
  TankStage tankStage = TankStage::IDLE;

  // === Internal Helpers ===

  /// @brief Apply minimum velocity threshold to overcome motor static friction
  /// @param velocity Raw PID output in RPM
  /// @return Adjusted velocity — zero if within deadband, bumped to minVelocity
  ///         if below the friction threshold, otherwise unchanged
  double applyMinVelocity(double velocity) const;

  /// @brief Clamp velocity magnitude to the configured max velocity
  /// @param velocity Raw or adjusted PID output in RPM
  /// @return Velocity clamped to [-maxVelocity, maxVelocity]
  double clampVelocity(double velocity) const;

  /// @brief Compute and apply motor commands for tank drive (sequential FSM)
  void updateTank();

  /// @brief Compute and apply motor commands for holonomic drive (simultaneous)
  void updateHolonomic();

  /// @brief Check if all relevant axes are within tolerance
  /// @return true if both position and heading errors are below thresholds
  bool checkTolerance() const;

  /// @brief Stop all drive motors via the active drivetrain
  void stopMotors();

 public:
  // === Constructors ===

  /// @brief Construct a PoseLock for a **tank drive** (non-holonomic)
  /// @param drive      Reference to the non-holonomic Drivetrain instance
  /// @param xPid      PID controller for linear distance correction
  /// @param thetaPid  PID controller for heading correction
  /// @param config    Configuration for tolerances and motor limits
  PoseLock(Drivetrain& drive, PID xPid, PID thetaPid,
           PoseLockConfig config = {});

  /// @brief Construct a PoseLock for a **holonomic** drive
  /// @param drive      Reference to the holonomic Drivetrain instance
  /// @param xPid      PID controller for the forward axis
  /// @param yPid      PID controller for the strafe axis
  /// @param thetaPid  PID controller for heading correction
  /// @param config    Configuration for tolerances and motor limits
  PoseLock(Drivetrain& drive, PID xPid, PID yPid, PID thetaPid,
           PoseLockConfig config = {});

  // === Core Interface ===

  /// @brief Set the target pose for the PoseLock to achieve
  /// @param target The desired pose (x in inches, y in inches, theta in radians)
  /// @note Resets all internal state and PID controllers automatically
  void setTarget(const Pose& target);

  /// @brief Update the PoseLock — call once per control loop iteration
  /// @details Computes PID outputs, applies motor commands,
  ///          and advances the state machine.
  void update();

  /// @brief Check if the PoseLock has finished settling
  /// @return true if the robot is within tolerance for the required number
  ///         of cycles, or the safety timeout has elapsed
  bool isFinished() const;

  /// @brief Reset the PoseLock to IDLE, clearing all state and stopping motors
  void reset();

  // === Accessors ===

  /// @brief Get the current state of the outer state machine
  PoseLockState getState() const;

  /// @brief Get the current tank settling stage (only meaningful for non-holonomic drives)
  TankStage getTankStage() const;
};

}  // namespace aon

#endif  // AON_CONTROLS_POSE_LOCK_HPP__
