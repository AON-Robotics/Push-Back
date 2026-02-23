#ifndef AON_CONTROLS_POSE_LOCK_HPP__
#define AON_CONTROLS_POSE_LOCK_HPP__

#include <cmath>
#include "./pid/pid.hpp"
#include "../drivetrain.hpp"
#include "../tank-drive/tank-drive.hpp"
#include "../x-drive/x-drive.hpp"
#include "../odometry/odometry.hpp"

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

/// @brief The type of drivetrain the PoseLock will control
enum class DriveType {
  TANK,       ///< Differential (tank) drive — 2 DOF: forward + turn
  HOLONOMIC   ///< Holonomic (X-drive) — 3 DOF: forward + strafe + turn
};

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
  ALIGN_HEADING,    ///< Turn to face the target position
  DRIVE_TO_TARGET,  ///< Drive toward the target while maintaining heading
  FINAL_HEADING,    ///< Correct to the desired final heading
  DONE              ///< All stages complete
};

// ============================= Structs ======================================

/// @brief Configuration for PoseLock tolerances and motor limits
///
/// @details Tune these values per-robot to match physical characteristics.
///          `minPower` is critical — set it to the minimum RPM that
///          reliably overcomes static friction on your drivetrain.
struct PoseLockConfig {
  /// Position tolerance in \b inches — below this, position is "settled"
  double linearTolerance = 0.5;

  /// Angular tolerance in \b radians — below this, heading is "settled"
  double angularTolerance = 0.03;  // ~1.7 degrees

  /// Consecutive `update()` cycles within tolerance before declaring finished
  int settledCycles = 10;

  /// Minimum motor power in \b RPM to overcome static friction
  /// @note Set this to the lowest RPM that reliably moves your specific robot
  double minPower = 15.0;

  /// Maximum motor power in \b RPM to clamp PID output
  double maxPower = 200.0;

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
///                    aon::PID(2.0, 0, 0.1),   // linear PID
///                    aon::PID(1.5, 0, 0.05));  // heading PID
/// lock.setTarget(aon::Pose(24.0, 48.0, M_PI / 2));
/// while (!lock.isFinished()) {
///   lock.update();
///   pros::delay(10);
/// }
///
/// // === Holonomic (X-Drive) ===
/// aon::PoseLock lock(xdrive,
///                    aon::PID(2.0, 0, 0.1),   // forward PID
///                    aon::PID(2.0, 0, 0.1),   // strafe PID
///                    aon::PID(1.5, 0, 0.05)); // heading PID
/// lock.setTarget(aon::Pose(24.0, 48.0, M_PI / 2));
/// while (!lock.isFinished()) {
///   lock.update();
///   pros::delay(10);
/// }
/// @endcode
class PoseLock {
 private:
  // === Configuration ===
  DriveType driveType;
  PoseLockState state;
  PoseLockConfig config;

  // === PID Controllers ===
  PID linearPid;    ///< Distance (tank) or forward-axis (holonomic)
  PID strafePid;    ///< Strafe-axis (holonomic only, unused in tank)
  PID headingPid;   ///< Angular correction

  // === Target ===
  Pose targetPose;

  // === Drive References (one is active based on DriveType) ===
  TankDrive* tankDrive = nullptr;
  XDrive* holonomicDrive = nullptr;

  // === Odometry Reference (obtained from the active drive) ===
  Odometry* odometry = nullptr;

  // === State Tracking ===
  int settledCounter = 0;
  uint32_t startTime = 0;
  TankStage tankStage = TankStage::ALIGN_HEADING;

  // === Internal Helpers ===

  /// @brief Apply minimum power threshold to overcome motor static friction
  /// @param output Raw PID output in RPM
  /// @return Adjusted output — zero if within deadband, bumped to minPower
  ///         if below the friction threshold, otherwise unchanged
  double applyMinPower(double output) const;

  /// @brief Clamp output magnitude to the configured max power
  /// @param output Raw or adjusted PID output in RPM
  /// @return Output clamped to [-maxPower, maxPower]
  double clampOutput(double output) const;

  /// @brief Normalize an angle to the range [-PI, PI]
  /// @param angle Input angle in radians
  /// @return Equivalent angle in [-PI, PI]
  static double normalizeAngle(double angle);

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

  /// @brief Construct a PoseLock for a **tank drive**
  /// @param drive   Reference to the TankDrive instance
  /// @param linearPid  PID controller for linear distance correction
  /// @param headingPid PID controller for heading correction
  /// @param config  Configuration for tolerances and motor limits
  PoseLock(TankDrive& drive, PID linearPid, PID headingPid,
           PoseLockConfig config = {});

  /// @brief Construct a PoseLock for a **holonomic (X-Drive)** drive
  /// @param drive      Reference to the XDrive instance
  /// @param linearPid  PID controller for the forward axis
  /// @param strafePid  PID controller for the strafe axis
  /// @param headingPid PID controller for heading correction
  /// @param config     Configuration for tolerances and motor limits
  PoseLock(XDrive& drive, PID linearPid, PID strafePid, PID headingPid,
           PoseLockConfig config = {});

  // === Core Interface ===

  /// @brief Set the target pose for the PoseLock to achieve
  /// @param target The desired pose (x in inches, y in inches, theta in radians)
  /// @note Resets all internal state and PID controllers automatically
  void setTarget(const Pose& target);

  /// @brief Update the PoseLock — call once per control loop iteration
  /// @details Reads odometry, computes PID outputs, applies motor commands,
  ///          and advances the state machine. Also calls `odometry::Update()`.
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

  /// @brief Get the current tank settling stage (only meaningful for tank drive)
  TankStage getTankStage() const;

  /// @brief Get the active drive type
  DriveType getDriveType() const;
};

}  // namespace aon

#endif  // AON_CONTROLS_POSE_LOCK_HPP__
