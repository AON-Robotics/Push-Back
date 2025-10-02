#pragma once
#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "../api.h"
#include "../okapi/api.hpp"
#include "./constants.hpp"
#include "./controls/pid/pid.hpp"
#include "./tools/vector.hpp"
#include "./controls/TankDrive/tankDrive.hpp"
// ============================================================================
//   _____  _    _  _ _  __  ___  ___ _____   _____ 
//  |_   _|/_\  | \| | |/ / |   \| _ \_ _\ \ / / __|
//    | | / _ \ | .` | ' <  | |) |   /| | \ V /| _| 
//    |_|/_/ \_\|_|\_|_|\_\ |___/|_|_|___| \_/ |___|
//
// ============================================================================

// PIDs
inline aon::PID drivePID = aon::PID(TANK_DRIVE_PID_KP, TANK_DRIVE_PID_KI, TANK_DRIVE_PID_KD);
inline aon::PID turnPID = aon::PID(TANK_TURN_PID_KP, TANK_TURN_PID_KI, TANK_TURN_PID_KD);

inline aon::TankDrive tankDrive();

// Intake

inline okapi::MotorGroup intake = okapi::MotorGroup({-16, 17});
inline okapi::Motor rail = okapi::Motor(17);
inline okapi::Motor gate = okapi::Motor(-16);

// Misc

inline okapi::Motor arm = okapi::Motor(11);
inline okapi::Motor turret = okapi::Motor(-15);

// TriPort

static pros::ADIDigitalOut indexer ('G');
static bool indexerOut = false;
static pros::ADIDigitalOut claw ('H');
static bool clawOn = false;

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================

// Encoders

static pros::Rotation encoderRight(5, true);
static pros::Rotation encoderLeft(4, false);
static pros::Rotation encoderBack(11, false);
static pros::Rotation turretEncoder(14, true);

static pros::ADIEncoder opticalEncoder('A', 'B');

// Vision

// Colors
enum Colors {
  RED = 1,
  BLUE,
  STAKE
};

static Colors COLOR = RED;

static pros::Vision vision_sensor(12);
static volatile bool turretFollowing = false;
static volatile bool turretBraking = true;
static volatile bool turretScanning = false;
static pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
static pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(BLUE, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);
static pros::vision_signature_s_t STAKE_SIG = pros::Vision::signature_from_utility(STAKE, -2247, -1833, -2040, -5427, -4727, -5077, 4.600, 0); // RGB 4.600
static pros::Gps gps(13, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);

// Distance

static pros::Distance distanceSensor(3);
static volatile bool intakeScanning = false;


// Gyro/Accelerometer

#if GYRO_ENABLED
static pros::Imu gyroscope(6);
#endif

// Potentiometer

static pros::ADIPotentiometer potentiometer('F');

static aon::PID fastPID = aon::PID(1, 0, 0);
static aon::PID turretPID = aon::PID(0.25, 0, 0);

/// Controller
static pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);

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

  tankDrive.driveLeft.setBrakeMode(brakeMode); 
  tankDrive.driveLeft.setGearing(okapi::AbstractMotor::gearset::blue);
  tankDrive.driveLeft.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  tankDrive.driveLeft.tarePosition();

  tankDrive.driveRight.setBrakeMode(brakeMode);
  tankDrive.driveRight.setGearing(okapi::AbstractMotor::gearset::blue);
  tankDrive.driveRight.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  tankDrive.driveRight.tarePosition();

  tankDrive.driveFull.setBrakeMode(brakeMode);
  tankDrive.driveFull.setGearing(okapi::AbstractMotor::gearset::blue);
  tankDrive.driveFull.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  tankDrive.driveFull.tarePosition();

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
inline void STOP(){
  tankDrive.driveFull.moveVelocity(0);
  intake.moveVelocity(0);
  arm.moveVelocity(0);
  turret.moveVelocity(0);
}

/**
 * \brief Returns position of the robot in the field
 *
 * \returns The GPS coordinates as a `Vector`
 */
inline Vector position(){
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
inline void testEndpoint(int speed = 100){
  STOP();
  intake.moveVelocity(speed);
  pros::delay(1000);
  intake.moveVelocity(0);
}

/**
 * \brief Makes the rail go slightly back
 */
inline void kickBackRail(){
  rail.moveVelocity(-100);
  pros::delay(150);
  rail.moveVelocity(0);
}

/**
 * \brief Task to stop all motors during auton testing if something goes wrong
 */
inline void autonSafety(){
  while(true){
    while(mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      STOP();
    }
    pros::delay(50);
  }
}

/// @brief Begins ORBIT following cycle
inline void activateORBITFollow(){
  turretFollowing = true;
  turretBraking = true;
  turretScanning = false;
}

/// @brief Ends ORBIT following cycle
inline void deactivateORBITFollow(){
  turretFollowing = false;
}

/// @brief Begins ORBIT scanning cycle
inline void activateORBITScan(){
  turretFollowing = false;
  turretBraking = false;
  turretScanning = true;
}

/// @brief Ends ORBIT scanning cycle
inline void deactivateORBITScan(){
  turretScanning = false;
}

/// @brief Sets the ORBIT to brake if not scanning
inline void brakeORBIT(){
  turretBraking = true;
}

/// @brief Releases the ORBIT from braking to allow other functions to use it
inline void releaseORBIT() {
  turretBraking = false;
}

/// @brief Starts intake scanning cycle
inline void activateIntakeScan(){
  intakeScanning = true;
}

/// @brief Ends intake scanning cycle
inline void deactivateIntakeScan(){
  intakeScanning = false;
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
