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
aon::XDrive drivetrain = aon::XDrive();
aon::TankDrive drivetrainTank = aon::TankDrive();

// Intake
aon::Intake intake = aon::Intake({-1, 1}, {1}, {-1}, 1);

// Big Bot
okapi::MotorGroup elevatorB = okapi::MotorGroup({1});
okapi::MotorGroup hoarder = okapi::MotorGroup({-1});
okapi::MotorGroup scorerB = okapi::MotorGroup({-1});
// End Big Bot

// Small Bot
// Drivetrain
okapi::MotorGroup left({-7, -8, 9, 10});
okapi::MotorGroup right({14, 12, -13, -11});

// Intake
okapi::MotorGroup elevator({6, 3});
okapi::MotorGroup scorer({-4});
okapi::MotorGroup judge({2});
pros::ADIDigitalOut topScorer ('A');
pros::ADIDigitalOut puncher ('B');
// End Small Bot

// Misc

okapi::Motor arm = okapi::Motor(1);
okapi::Motor turret = okapi::Motor(-11);

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

pros::Rotation turretEncoder(1, true);

pros::ADIEncoder opticalEncoder('C', 'D');

// Vision

// Colors
enum Colors {
  RED = 1,
  BLUE,
  STAKE
};

Colors COLOR = RED;

pros::Vision vision_sensor(1);
volatile bool turretFollowing = false;
volatile bool turretBraking = true;
volatile bool turretScanning = false;
pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(BLUE, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);
pros::vision_signature_s_t STAKE_SIG = pros::Vision::signature_from_utility(STAKE, -2247, -1833, -2040, -5427, -4727, -5077, 4.600, 0); // RGB 4.600

// Distance

pros::Distance distanceSensor(1);
volatile bool intakeScanning = false; // TODO: remove this

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

  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue);

  elevator.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  elevator.setGearing(okapi::AbstractMotor::gearset::green);
  elevator.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  elevator.tarePosition();

  hoarder.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  hoarder.setGearing(okapi::AbstractMotor::gearset::green);
  hoarder.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  hoarder.tarePosition();

  scorer.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  scorer.setGearing(okapi::AbstractMotor::gearset::green);
  scorer.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  scorer.tarePosition();

  left.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  left.setGearing(okapi::AbstractMotor::gearset::blue);
  left.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  left.tarePosition();

  right.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  right.setGearing(okapi::AbstractMotor::gearset::blue);
  right.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  right.tarePosition();

  scorer.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  scorer.setGearing(okapi::AbstractMotor::gearset::blue);
  scorer.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  scorer.tarePosition();

  judge.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  judge.setGearing(okapi::AbstractMotor::gearset::blue);
  judge.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  judge.tarePosition();

  elevator.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  elevator.setGearing(okapi::AbstractMotor::gearset::blue);
  elevator.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  elevator.tarePosition();

  arm.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  arm.setGearing(okapi::AbstractMotor::gearset::green);
  arm.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  arm.tarePosition();

  turret.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  turret.setGearing(okapi::AbstractMotor::gearset::green);
  turret.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  turret.tarePosition();

}

/// @brief Adds the colors to the vision sensor
inline void ConfigureColors(){
  vision_sensor.set_signature(RED, &RED_SIG);
  vision_sensor.set_signature(BLUE, &BLUE_SIG);
  vision_sensor.set_signature(STAKE, &STAKE_SIG);
}

/// @brief Stops movement from robot
void STOP(){
  drivetrain.stop();
  intake.stop();
  arm.moveVelocity(0);
  turret.moveVelocity(0);
}

/// @brief Toggles the value of a bool
/// @param boolean The variable to be toggled
/// @returns The updated boolean
inline bool toggle(bool &boolean) {
  boolean = !boolean;
  return boolean;
}

/// @brief Used to make sure a condition is being met or a block of code is being run
/// @param speed The speed with which to spin the intake to differentiate between multiple tests
/// @note `speed` should vary if running multiple tests in one same run to be able to tell apart between them
void testEndpoint(int speed = 100){
  STOP(); 
  intake.move(speed);
  pros::delay(1000);
  intake.stop();
}

/// @brief Task to stop all motors during auton testing if something goes wrong
void autonSafety(){
  while(true){
    while(mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
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

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
