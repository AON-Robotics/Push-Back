#include "../include/aon/controls/pose-lock.hpp"

namespace aon {

// ============================================================================
//     ___              _                  _
//    / __|___ _ _  ___| |_ _ _ _  _ __| |_ ___ _ _ ___
//   | (__/ _ \ ' \(_-<  _| '_| || / _|  _/ _ \ '_(_-<
//    \___\___/_||_/__/\__|_|  \_,_\__|\__\___/_| /__/
//
// ============================================================================

PoseLock::PoseLock(TankDrive& drive, PID linearPid, PID headingPid,
                   PoseLockConfig config)
    : driveType(DriveType::TANK),
      state(PoseLockState::IDLE),
      config(config),
      linearPid(linearPid),
      strafePid(0, 0, 0),  // unused for tank drive
      headingPid(headingPid),
      targetPose(),
      tankDrive(&drive),
      holonomicDrive(nullptr),
      odometry(&drive.odom) {}

PoseLock::PoseLock(XDrive& drive, PID linearPid, PID strafePid,
                   PID headingPid, PoseLockConfig config)
    : driveType(DriveType::HOLONOMIC),
      state(PoseLockState::IDLE),
      config(config),
      linearPid(linearPid),
      strafePid(strafePid),
      headingPid(headingPid),
      targetPose(),
      tankDrive(nullptr),
      holonomicDrive(&drive),
      odometry(&drive.odom) {}

// ============================================================================
//    ___       _    _ _        __  __     _   _            _
//   | _ \_  _| |__| (_)__   |  \/  |___| |_| |_  ___  __| |___
//   |  _/ || | '_ \ | / _|  | |\/| / -_)  _| ' \/ _ \/ _` (_-<
//   |_|  \_,_|_.__/_|_\__|  |_|  |_\___|\__|_||_\___/\__,_/__/
//
// ============================================================================

void PoseLock::setTarget(const Pose& target) {
  targetPose = target;
  state = PoseLockState::SETTLING;
  settledCounter = 0;
  startTime = pros::millis();
  tankStage = TankStage::ALIGN_HEADING;

  // Reset all PID controllers for a clean start
  linearPid.Reset();
  strafePid.Reset();
  headingPid.Reset();
}

void PoseLock::update() {
  if (state != PoseLockState::SETTLING) return;

  // Safety timeout — finish even if not fully settled
  if ((pros::millis() - startTime) >= static_cast<uint32_t>(config.timeout)) {
    stopMotors();
    state = PoseLockState::FINISHED;
    return;
  }

  // Refresh odometry before computing errors
  odometry->update();

  // Dispatch to drive-specific update logic
  switch (driveType) {
    case DriveType::TANK:
      updateTank();
      break;
    case DriveType::HOLONOMIC:
      updateHolonomic();
      break;
  }
}

bool PoseLock::isFinished() const {
  return state == PoseLockState::FINISHED;
}

void PoseLock::reset() {
  stopMotors();
  state = PoseLockState::IDLE;
  settledCounter = 0;
  tankStage = TankStage::ALIGN_HEADING;
  linearPid.Reset();
  strafePid.Reset();
  headingPid.Reset();
}

PoseLockState PoseLock::getState() const { return state; }

TankStage PoseLock::getTankStage() const { return tankStage; }

DriveType PoseLock::getDriveType() const { return driveType; }

// ============================================================================
//    ___     _                  _   _  _     _
//   |_ _|_ _| |_ ___ _ _ _ _ | | | || |___| |_ __  ___ _ _ ___
//    | || ' \  _/ -_) '_| ' \| | | __ / -_) | '_ \/ -_) '_(_-<
//   |___|_||_\__\___|_| |_||_|_| |_||_\___|_| .__/\___|_| /__/
//                                            |_|
// ============================================================================

double PoseLock::applyMinPower(double output) const {
  // Below the deadband — motor wouldn't meaningfully move, treat as zero
  if (std::fabs(output) < config.deadband) {
    return 0.0;
  }

  // Between deadband and minPower — bump up to overcome static friction
  if (std::fabs(output) < config.minPower) {
    return std::copysign(config.minPower, output);
  }

  return output;
}

double PoseLock::clampOutput(double output) const {
  if (output > config.maxPower) return config.maxPower;
  if (output < -config.maxPower) return -config.maxPower;
  return output;
}

double PoseLock::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void PoseLock::stopMotors() {
  if (driveType == DriveType::TANK && tankDrive != nullptr) {
    tankDrive->stop();
  } else if (driveType == DriveType::HOLONOMIC && holonomicDrive != nullptr) {
    holonomicDrive->stop();
  }
}

bool PoseLock::checkTolerance() const {
  const double currentX = odometry->getX();
  const double currentY = odometry->getY();
  const double currentTheta = odometry->getRadians();

  const double dx = targetPose.x - currentX;
  const double dy = targetPose.y - currentY;
  const double distError = std::hypot(dx, dy);
  const double headingError =
      std::fabs(normalizeAngle(targetPose.theta - currentTheta));

  return (distError < config.linearTolerance) &&
         (headingError < config.angularTolerance);
}

// ============================================================================
//    _____          _      ___       _           _   _          _        _
//   |_   _|_ _ _ _ | |__  |   \ _ _(_)_ _____  | | | |_ __  __| |__ _ | |_ ___
//     | |/ _` | ' \| / /  | |) | '_| \ V / -_) | |_| | '_ \/ _` / _` ||  _/ -_)
//     |_|\__,_|_||_|_\_\  |___/|_| |_|\_/\___|  \___/| .__/\__,_\__,_| \__\___|
//                                                     |_|
//   Sequential State Machine: ALIGN_HEADING -> DRIVE_TO_TARGET -> FINAL_HEADING
// ============================================================================

void PoseLock::updateTank() {
  // --- Read current pose from odometry ---
  const double currentX = odometry->getX();
  const double currentY = odometry->getY();
  const double currentTheta = odometry->getRadians();

  // --- Compute field-frame errors ---
  const double dx = targetPose.x - currentX;
  const double dy = targetPose.y - currentY;
  const double distToTarget = std::hypot(dx, dy);

  // Heading required to face the target position
  // atan2(dy, dx) matches the odometry rotation convention
  const double angleToTarget = std::atan2(dy, dx);

  switch (tankStage) {
    // -----------------------------------------------------------------------
    //  Stage 1: Turn to face the target position
    // -----------------------------------------------------------------------
    case TankStage::ALIGN_HEADING: {
      const double headingError =
          normalizeAngle(angleToTarget - currentTheta);

      double turnOutput = headingPid.Output(headingError, 0.0);
      turnOutput = clampOutput(applyMinPower(turnOutput));

      tankDrive->driveWhileTurning(0.0, turnOutput);

      // Transition: heading is aligned OR already close enough to skip
      if (std::fabs(headingError) < config.angularTolerance ||
          distToTarget < config.linearTolerance) {
        headingPid.Reset();
        tankStage = TankStage::DRIVE_TO_TARGET;
      }
      break;
    }

    // -----------------------------------------------------------------------
    //  Stage 2: Drive forward while maintaining heading toward the target
    // -----------------------------------------------------------------------
    case TankStage::DRIVE_TO_TARGET: {
      // Signed forward distance — negative means the robot overshot
      const double forwardDist =
          dx * std::cos(currentTheta) + dy * std::sin(currentTheta);

      double forwardOutput = linearPid.Output(forwardDist, 0.0);
      forwardOutput = clampOutput(applyMinPower(forwardOutput));

      // Keep heading pointed at the target while driving
      const double headingError =
          normalizeAngle(angleToTarget - currentTheta);
      double turnOutput = headingPid.Output(headingError, 0.0);
      turnOutput = clampOutput(applyMinPower(turnOutput));

      tankDrive->driveWhileTurning(forwardOutput, turnOutput);

      // Transition: close enough to target position
      if (distToTarget < config.linearTolerance) {
        linearPid.Reset();
        headingPid.Reset();
        tankStage = TankStage::FINAL_HEADING;
      }
      break;
    }

    // -----------------------------------------------------------------------
    //  Stage 3: Correct to the desired final heading
    // -----------------------------------------------------------------------
    case TankStage::FINAL_HEADING: {
      const double headingError =
          normalizeAngle(targetPose.theta - currentTheta);

      double turnOutput = headingPid.Output(headingError, 0.0);
      turnOutput = clampOutput(applyMinPower(turnOutput));

      tankDrive->driveWhileTurning(0.0, turnOutput);

      // Check if fully settled across ALL axes
      if (checkTolerance()) {
        settledCounter++;
        if (settledCounter >= config.settledCycles) {
          tankStage = TankStage::DONE;
          stopMotors();
          state = PoseLockState::FINISHED;
        }
      } else {
        settledCounter = 0;
      }
      break;
    }

    case TankStage::DONE:
      break;
  }
}

// ============================================================================
//    _  _     _                        _       ___       _           _   _
//   | || |___| |___ _ _  ___ _ __ (_)__  |   \ _ _(_)_ _____  | | | _ __
//   | __ / _ \ / _ \ ' \/ _ \ '  \| / _| | |) | '_| \ V / -_) | |_| | '_ \
//   |_||_\___/_\___/_||_\___/_|_|_|_\__| |___/|_| |_|\_/\___| \___/| .__/
//                                                                    |_|
//   Simultaneous 3-DOF Correction: forward + strafe + heading
// ============================================================================

void PoseLock::updateHolonomic() {
  // goToPose is a blocking call that handles its own 3-DOF control loop
  // (forward + strafe + turn) using motion profiles internally.
  holonomicDrive->goToPose(targetPose);

  // goToPose blocks until settled, so we can immediately mark as finished
  stopMotors();
  state = PoseLockState::FINISHED;
}

}  // namespace aon
