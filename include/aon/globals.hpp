#pragma once

#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "./constants.hpp"
#include "../api.h"
#include "../okapi/api.hpp"
#include "./controls/pid/pid.hpp"
#include "./tools/vector.hpp"
#include "./x-drive/x-drive.hpp"
#include "./intake/intake.hpp"
#include "./tank-drive/tank-drive.hpp"

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================


// Drivetrain
inline aon::XDrive drivetrain = aon::XDrive();
inline aon::TankDrive drivetrainTank = aon::TankDrive();

// Intake
inline aon::Intake intake = aon::Intake({-16, 17}, {17}, {-16}, 3);
inline okapi::MotorGroup bottom = okapi::MotorGroup({1});
inline okapi::MotorGroup top = okapi::MotorGroup({-2});

// Misc
inline okapi::Motor arm = okapi::Motor(11);
inline okapi::Motor turret = okapi::Motor(-15);

// TriPort
inline pros::ADIDigitalOut indexer('G');
inline bool indexerOut = false;
inline pros::ADIDigitalOut claw('H');
inline bool clawOn = false;

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================

// Encoders
inline pros::Rotation turretEncoder(14, true);
inline pros::ADIEncoder opticalEncoder('A', 'B');

// Vision

// Colors
enum Colors {
  RED = 1,
  BLUE,
  STAKE
};

inline Colors COLOR = RED;
inline pros::Vision vision_sensor(12);
inline volatile bool turretFollowing = false;
inline volatile bool turretBraking = true;
inline volatile bool turretScanning = false;

inline pros::vision_signature_s_t RED_SIG =
    pros::Vision::signature_from_utility(RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
inline pros::vision_signature_s_t BLUE_SIG =
    pros::Vision::signature_from_utility(BLUE, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);
inline pros::vision_signature_s_t STAKE_SIG =
    pros::Vision::signature_from_utility(STAKE, -2247, -1833, -2040, -5427, -4727, -5077, 4.600, 0);

// Distance
inline pros::Distance distanceSensor(3);
inline volatile bool intakeScanning = false; // TODO: remove this

// Potentiometer
inline pros::ADIPotentiometer potentiometer('F');

// PIDs
inline aon::PID drivePID = aon::PID(0.02, 0, 0);
inline aon::PID turnPID = aon::PID(0.002, 0, 0);
inline aon::PID fastPID = aon::PID(1, 0, 0);
inline aon::PID turretPID = aon::PID(0.25, 0, 0);

// Controller
inline pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);

namespace aon::operator_control {
/// Driver profiles for all robots
enum Drivers {
  IAN,
  DAVID,
  DEFAULT,
};
}  // namespace aon::operator_control

// ============================================================================
//   ___ _   _ _  _  ___ _____ ___ ___  _  _ ___ 
//  | __| | | | \| |/ __|_   _|_ _/ _ \| \| / __|
//  | _|| |_| | .` | (__  | |  | | (_) | .` \__ \
//  |_|  \___/|_|\_|\___| |_| |___\___/|_|\_|___/
//
// ============================================================================

namespace aon {

inline void ConfigureMotors(const bool opcontrol = true) {
  // HOLD for AUTONOMOUS ||| BRAKE for OPERATOR CONTROL
  okapi::AbstractMotor::brakeMode brakeMode =
      opcontrol ? okapi::AbstractMotor::brakeMode::brake
                : okapi::AbstractMotor::brakeMode::hold;

  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue);

  arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  arm.setGearing(okapi::AbstractMotor::gearset::red);
  arm.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  arm.tarePosition();

  turret.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  turret.setGearing(okapi::AbstractMotor::gearset::green);
  turret.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  turret.tarePosition();
}

/**
 * \brief Adds the colors to the vision sensor
*/
inline void ConfigureColors() {
  vision_sensor.set_signature(RED, &RED_SIG);
  vision_sensor.set_signature(BLUE, &BLUE_SIG);
  vision_sensor.set_signature(STAKE, &STAKE_SIG);
}

/**
 * \brief Stops movement from robot
 */
inline void STOP() {
  drivetrain.stop();
  intake.stop();
  arm.moveVelocity(0);
  turret.moveVelocity(0);
}

/**
 * \brief Toggles the value of a bool
 */
inline bool toggle(bool &boolean) {
  boolean = !boolean;
  return boolean;
}

/**
 * \brief Used to make sure a condition is being met or a block of code is being run
 */
inline void testEndpoint(int speed = 100) {
  STOP();
  intake.move(speed);
  pros::delay(1000);
  intake.stop();
}

/**
 * \brief Task to stop all motors during auton testing if something goes wrong
 */
inline void autonSafety() {
  while (true) {
    while (mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      STOP();
    }
    pros::delay(50);
  }
}

/// @brief Begins ORBIT following cycle
inline void activateORBITFollow() {
  turretFollowing = true;
  turretBraking = true;
  turretScanning = false;
}

/// @brief Ends ORBIT following cycle
inline void deactivateORBITFollow() { turretFollowing = false; }

/// @brief Begins ORBIT scanning cycle
inline void activateORBITScan() {
  turretFollowing = false;
  turretBraking = false;
  turretScanning = true;
}

/// @brief Ends ORBIT scanning cycle
inline void deactivateORBITScan() { turretScanning = false; }

/// @brief Sets the ORBIT to brake if not scanning
inline void brakeORBIT() { turretBraking = true; }

/// @brief Releases the ORBIT from braking to allow other functions to use it
inline void releaseORBIT() { turretBraking = false; }

}  // namespace aon

#endif  // AON_GLOBALS_HPP_