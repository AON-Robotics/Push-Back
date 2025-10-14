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
#include "./orbit/orbit.hpp"

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================

aon::Orbit orbit(1,true,1,1);

// Drivetrain
aon::XDrive drivetrain = aon::XDrive();
aon::TankDrive drivetrainTank = aon::TankDrive();


//Intake:
aon::Intake intake = aon::Intake({-16, 17}, {17}, {-16}, 3);

okapi::MotorGroup bottom = okapi::MotorGroup({1});
okapi::MotorGroup top = okapi::MotorGroup({-2});

// Misc

okapi::Motor arm = okapi::Motor(11);
okapi::Motor turret = okapi::Motor(-15);

// TriPort

pros::ADIDigitalOut indexer ('G');
bool indexerOut = false;
pros::ADIDigitalOut claw ('H');
bool clawOn = false;

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================

// Encoders

pros::Rotation encoderRight(5, true);
pros::Rotation encoderLeft(4, false);
pros::Rotation encoderBack(11, false);

pros::ADIEncoder opticalEncoder('A', 'B');

pros::Gps gps(13, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);

// Distance

pros::Distance distanceSensor(3);
volatile bool intakeScanning = false; // TODO: remove this

// Potentiometer

pros::ADIPotentiometer potentiometer('F');

/// PIDs

aon::PID drivePID = aon::PID(0.02, 0, 0);
aon::PID turnPID = aon::PID(0.002, 0, 0);
aon::PID fastPID = aon::PID(1, 0, 0);


/// Controller
pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);

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
  okapi::AbstractMotor::brakeMode brakeMode = opcontrol ? okapi::AbstractMotor::brakeMode::brake : okapi::AbstractMotor::brakeMode::hold;

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
 * \brief Stops movement from robot
 */
void STOP(){
  drivetrain.stop();
  intake.stop();
  arm.moveVelocity(0);
  turret.moveVelocity(0);
}

/**
 * \brief Toggles the value of a bool
 * 
 * \param boolean The variable to be toggled
 * 
 * \returns The updated boolean
 */
inline bool toggle(bool &boolean) {
  boolean = !boolean;
  return boolean;
}

/**
 * \brief Used to make sure a condition is being met or a block of code is being run
 * 
 * \param speed The speed with which to spin the intake to differentiate between multiple tests
 * 
 * \note `speed` should vary if running multiple tests in one same run to be able to tell apart between them
*/
void testEndpoint(int speed = 100){
  STOP(); 
  intake.move(speed);
  pros::delay(1000);
  intake.stop();
}

/**
 * \brief Task to stop all motors during auton testing if something goes wrong
 */
void autonSafety(){
  while(true){
    while(mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      STOP();
    }
    pros::delay(50);
  }
}

/// @brief Starts intake scanning cycle
void activateIntakeScan(){
  intakeScanning = true;
}

/// @brief Ends intake scanning cycle
void deactivateIntakeScan(){
  intakeScanning = false;
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
