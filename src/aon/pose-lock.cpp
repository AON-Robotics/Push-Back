#include "../include/aon/controls/pose-lock.hpp"

namespace aon {

// ============================================================================
//     ___              _                  _
//    / __|___ _ _  ___| |_ _ _ _  _ __| |_ ___ _ _ ___
//   | (__/ _ \ ' \(_-<  _| '_| || / _|  _/ _ \ '_(_-<
//    \___\___/_||_/__/\__|_|  \_,_\__|\__\___/_| /__/
//
// ============================================================================

PoseLock::PoseLock(Drivetrain& drive, PID xPid, PID thetaPid,
                   PoseLockConfig config)
    : state(PoseLockState::IDLE),
      config(config),
      xPid(xPid),
      yPid(0, 0, 0),  // unused for non-holonomic drives
      thetaPid(thetaPid),
      targetPose(),
      drive(&drive),
      holonomic(false) {}

PoseLock::PoseLock(Drivetrain& drive, PID xPid, PID yPid,
                   PID thetaPid, PoseLockConfig config)
    : state(PoseLockState::IDLE),
      config(config),
      xPid(xPid),
      yPid(yPid),
      thetaPid(thetaPid),
      targetPose(),
      drive(&drive),
      holonomic(true) {}

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
  if (!holonomic) {
    tankStage = TankStage::ALIGN_HEADING;
  }

  // Reset all PID controllers for a clean start
  xPid.Reset();
  yPid.Reset();
  thetaPid.Reset();
}

void PoseLock::update() {
  if (state != PoseLockState::SETTLING) return;

  // Safety timeout — finish even if not fully settled
  if ((pros::millis() - startTime) >= static_cast<uint32_t>(config.timeout)) {
    stopMotors();
    state = PoseLockState::FINISHED;
    return;
  }

  // Dispatch to drive-specific update logic
  if (holonomic) {
    updateHolonomic();
  } else {
    updateTank();
  }
}

bool PoseLock::isFinished() const {
  return state == PoseLockState::FINISHED;
}

void PoseLock::reset() {
  stopMotors();
  state = PoseLockState::IDLE;
  settledCounter = 0;
  tankStage = TankStage::IDLE;
  xPid.Reset();
  yPid.Reset();
  thetaPid.Reset();
}

PoseLockState PoseLock::getState() const { return state; }

TankStage PoseLock::getTankStage() const { return tankStage; }

// ============================================================================
//    ___     _                  _   _  _     _
//   |_ _|_ _| |_ ___ _ _ _ _ | | | || |___| |_ __  ___ _ _ ___
//    | || ' \  _/ -_) '_| ' \| | | __ / -_) | '_ \/ -_) '_(_-<
//   |___|_||_\__\___|_| |_||_|_| |_||_\___|_| .__/\___|_| /__/
//                                            |_|
// ============================================================================

double PoseLock::applyMinVelocity(double velocity) const {
  // Below the deadband — motor wouldn't meaningfully move, treat as zero
  if (std::fabs(velocity) < config.deadband) {
    return 0.0;
  }

  // Between deadband and minVelocity — bump up to overcome static friction
  if (std::fabs(velocity) < config.minVelocity) {
    return std::copysign(config.minVelocity, velocity);
  }

  return velocity;
}

double PoseLock::clampVelocity(double velocity) const {
  if (velocity > config.maxVelocity) return config.maxVelocity;
  if (velocity < -config.maxVelocity) return -config.maxVelocity;
  return velocity;
}


void PoseLock::stopMotors() {
  drive->stop();
}

bool PoseLock::checkTolerance() const {
  const double currentX = drive->getX();
  const double currentY = drive->getY();
  const double currentTheta = drive->getThetaRadians();

  const double dx = targetPose.x - currentX;
  const double dy = targetPose.y - currentY;
  const double distError = std::hypot(dx, dy);
  const double headingError =
      std::fabs((Angle().SetRadians(targetPose.theta) - Angle().SetRadians(currentTheta)).normalize().GetRadians());

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
  // --- Read current pose from drivetrain ---
  const double currentX = drive->getX();
  const double currentY = drive->getY();
  const double currentTheta = drive->getThetaRadians();

  // --- Compute field-frame errors ---
  const double dx = targetPose.x - currentX;
  const double dy = targetPose.y - currentY;
  const double distToTarget = std::hypot(dx, dy);

  // Heading required to face the target position
  Angle angleToTarget = Angle().SetPos(dx, dy);

  switch (tankStage) {
    case TankStage::IDLE:
      break;

    // -----------------------------------------------------------------------
    //  Stage 1: Turn to face the target position
    // -----------------------------------------------------------------------
    case TankStage::ALIGN_HEADING: {
      const double headingError =
          (angleToTarget - Angle().SetRadians(currentTheta)).normalize().GetRadians();

      double turnVelocity = thetaPid.Output(headingError, 0.0);
      turnVelocity = clampVelocity(applyMinVelocity(turnVelocity));

      drive->driveWhileTurning(0.0, turnVelocity);

      // Transition: heading is aligned OR already close enough to skip
      if (std::fabs(headingError) < config.angularTolerance ||
          distToTarget < config.linearTolerance) {
        thetaPid.Reset();
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

      double forwardVelocity = xPid.Output(forwardDist, 0.0);
      forwardVelocity = clampVelocity(applyMinVelocity(forwardVelocity));

      // Keep heading pointed at the target while driving
      const double headingError =
          (angleToTarget - Angle().SetRadians(currentTheta)).normalize().GetRadians();
      double turnVelocity = thetaPid.Output(headingError, 0.0);
      turnVelocity = clampVelocity(applyMinVelocity(turnVelocity));

      drive->driveWhileTurning(forwardVelocity, turnVelocity);

      // Transition: close enough to target position
      if (distToTarget < config.linearTolerance) {
        xPid.Reset();
        thetaPid.Reset();
        tankStage = TankStage::FINAL_HEADING;
      }
      break;
    }

    // -----------------------------------------------------------------------
    //  Stage 3: Correct to the desired final heading
    // -----------------------------------------------------------------------
    case TankStage::FINAL_HEADING: {
      const double headingError =
          (Angle().SetRadians(targetPose.theta) - Angle().SetRadians(currentTheta)).normalize().GetRadians();

      double turnVelocity = thetaPid.Output(headingError, 0.0);
      turnVelocity = clampVelocity(applyMinVelocity(turnVelocity));

      drive->driveWhileTurning(0.0, turnVelocity);

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
  drive->goToPose(targetPose);

  // goToPose blocks until settled, so we can immediately mark as finished
  stopMotors();
  state = PoseLockState::FINISHED;
}

}  // namespace aon
