#pragma once

#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"
#include "./controls/pid/pid.hpp"
#include "./tools/vector.hpp"

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================


// Drivetrain

okapi::MotorGroup driveLeft = okapi::MotorGroup({-20, 19, -18});
okapi::MotorGroup driveRight = okapi::MotorGroup({9, -8, 7});
okapi::MotorGroup driveFull = okapi::MotorGroup({-20, 19, -18, 9, -8, 7});
#include "./controls/s-curve-profile.hpp" //! Change this, I dont like doing the include this far down and after ive done other stuff
MotionProfile forwardProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);

// Intake

okapi::MotorGroup intake = okapi::MotorGroup({-16, 17});
okapi::Motor rail = okapi::Motor(17);
okapi::Motor gate = okapi::Motor(-16);

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
pros::Rotation turretEncoder(14, true);

pros::ADIEncoder opticalEncoder('A', 'B');

// Vision

// Colors
enum Colors {
  RED = 1,
  BLUE,
  STAKE
};

Colors COLOR = RED;

pros::Vision vision_sensor(12);
volatile bool turretFollowing = false;
volatile bool turretBraking = true;
volatile bool turretScanning = false;
pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(BLUE, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);
pros::vision_signature_s_t STAKE_SIG = pros::Vision::signature_from_utility(STAKE, -2247, -1833, -2040, -5427, -4727, -5077, 4.600, 0); // RGB 4.600
pros::Gps gps(13, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);

// Distance

pros::Distance distanceSensor(3);
volatile bool intakeScanning = false;


// Gyro/Accelerometer

#if GYRO_ENABLED
pros::Imu gyroscope(6);
#endif

// Potentiometer

pros::ADIPotentiometer potentiometer('F');

/// PIDs

aon::PID drivePID = aon::PID(0.02, 0, 0);
aon::PID turnPID = aon::PID(0.002, 0, 0);
aon::PID fastPID = aon::PID(1, 0, 0);
aon::PID turretPID = aon::PID(0.25, 0, 0);

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

  driveLeft.setBrakeMode(brakeMode); 
  driveLeft.setGearing(okapi::AbstractMotor::gearset::blue);
  driveLeft.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveLeft.tarePosition();

  driveRight.setBrakeMode(brakeMode);
  driveRight.setGearing(okapi::AbstractMotor::gearset::blue);
  driveRight.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveRight.tarePosition();

  driveFull.setBrakeMode(brakeMode);
  driveFull.setGearing(okapi::AbstractMotor::gearset::blue);
  driveFull.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  driveFull.tarePosition();

  intake.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake.setGearing(okapi::AbstractMotor::gearset::green);
  intake.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intake.tarePosition();

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
inline void ConfigureColors(){
  vision_sensor.set_signature(RED, &RED_SIG);
  vision_sensor.set_signature(BLUE, &BLUE_SIG);
  vision_sensor.set_signature(STAKE, &STAKE_SIG);
}

/**
 * \brief Stops movement from robot
 */
void STOP(){
  driveFull.moveVelocity(0);
  intake.moveVelocity(0);
  arm.moveVelocity(0);
  turret.moveVelocity(0);
}

/**
 * \brief Returns position of the robot in the field
 *
 * \returns The GPS coordinates as a `Vector`
 */
Vector position(){
  STOP();
  pros::delay(2000);
  pros::c::gps_status_s_t status = gps.get_status();
  Vector current = Vector().SetPosition(status.x, status.y);

  return current;
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
  intake.moveVelocity(speed);
  pros::delay(1000);
  intake.moveVelocity(0);
}

/**
 * \brief Makes the rail go slightly back
 */
void kickBackRail(){
  rail.moveVelocity(-100);
  pros::delay(150);
  rail.moveVelocity(0);
}

/**
 * \brief Task to stop all motors during auton testing if something goes wrong
 */
void autonSafety(){
  while(true){
    while(mainController.get_digital(DIGITAL_X)){
      STOP();
    }
    pros::delay(50);
  }
}

/// @brief Begins ORBIT following cycle
void activateORBITFollow(){
  turretFollowing = true;
  turretBraking = true;
  turretScanning = false;
}

/// @brief Ends ORBIT following cycle
void deactivateORBITFollow(){
  turretFollowing = false;
}

/// @brief Begins ORBIT scanning cycle
void activateORBITScan(){
  turretFollowing = false;
  turretBraking = false;
  turretScanning = true;
}

/// @brief Ends ORBIT scanning cycle
void deactivateORBITScan(){
  turretScanning = false;
}

/// @brief Sets the ORBIT to brake if not scanning
void brakeORBIT(){
  turretBraking = true;
}

/// @brief Releases the ORBIT from braking to allow other functions to use it
void releaseORBIT() {
  turretBraking = false;
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
