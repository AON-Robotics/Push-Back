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
#include "./h-drive/h-drive.hpp"
#include "./controls/s-curve-profile.hpp" //! Change this, I dont like doing the include this far down and after ive done other stuff

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================


#if USING_BIG_ROBOT

// Drivetrain
aon::HDrive drivetrain = aon::HDrive({-1, -2, 3}, {4, -5, 6}, 7);
// okapi::MotorGroup mid({17}); // Default make robot go right

pros::ADIDigitalOut semPiston('F'); // Shrek Ear Mechanism
pros::ADIDigitalOut brooksPiston('H');

aon::Intake intake = aon::Intake({1, 13, -10, 3, -6, -9}, {1}, {13}, {-10}, {3}, {-6}, {-9}, 'G', 8, 7);

pros::Vision vision_sensor(0);

#else

aon::TankDrive drivetrain = aon::TankDrive({-13, -12, 11, 14}, {16, -17, -19, 18});
aon::Intake intake = aon::Intake({6, -3, -2, -4, -7}, {6, -3}, {-2}, {-4, -7}, 'H', 'G', 'F', 5, 15);

okapi::Motor arrow(20);
pros::Vision vision_sensor(8);

#endif

// Misc
aon::Orbit orbit(1,true,1,1);

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================

// Encoders
pros::Rotation turretEncoder(0, true);

pros::ADIEncoder opticalEncoder('C', 'D');

// Vision

// Colors
enum Colors {
  RED = 1,
  BLUE,
  STAKE,
};

Colors COLOR = RED;

volatile bool turretFollowing = false;
volatile bool turretBraking = true;
volatile bool turretScanning = false;
pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(BLUE, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);
pros::vision_signature_s_t STAKE_SIG = pros::Vision::signature_from_utility(STAKE, -2247, -1833, -2040, -5427, -4727, -5077, 4.600, 0); // RGB 4.600

// Potentiometer
pros::ADIPotentiometer potentiometer('P');

/// PIDs
aon::PID drivePID = aon::PID(0.02, 0, 0);
aon::PID turnPID = aon::PID(0.002, 0, 0);
aon::PID fastPID = aon::PID(1, 0, 0);


/// Controller
pros::Controller mainController = pros::Controller(CONTROLLER_MASTER);

namespace aon::operator_control {

/// Driver profiles for all robots
enum Drivers {
  KEVIN,
  FABIAN,
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

inline void Configure(const bool opcontrol = true) {
  // HOLD for AUTONOMOUS ||| BRAKE for OPERATOR CONTROL
  okapi::AbstractMotor::brakeMode brakeMode = opcontrol ? okapi::AbstractMotor::brakeMode::brake : okapi::AbstractMotor::brakeMode::hold;

  #if USING_BIG_ROBOT
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue);
  
  intake.configure(okapi::AbstractMotor::brakeMode::brake, okapi::AbstractMotor::gearset::green);
  
  
  // mid.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  // mid.setGearing(okapi::AbstractMotor::gearset::green);
  // mid.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  // mid.tarePosition();
  
  #else
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue);
  
  intake.configure(okapi::AbstractMotor::brakeMode::coast, okapi::AbstractMotor::gearset::blue);
  
  arrow.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  arrow.setGearing(okapi::AbstractMotor::gearset::green);
  arrow.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  arrow.tarePosition();
  
  #endif
  orbit.configure();
}

/// @brief Stops movement from robot
void STOP(){
  drivetrain.stop();
  intake.stop();
  #if USING_BIG_ROBOT
  // mid.moveVelocity(0);
  #endif
  orbit.stop();
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

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
