#pragma once

#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "./constants.hpp"
#include "../api.h"
#include "../okapi/api.hpp"
#include "./controls/pid/pid.hpp"
#include "./tools/vector.hpp"
#include "./h-drive/h-drive.hpp"
#include "./intake/intake.hpp"
#include "./tank-drive/tank-drive.hpp"
#include "./orbit/orbit.hpp"
#include "./drivetrain.hpp"
#include "./odometry/odometry.hpp"

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================


#if USING_BIG_ROBOT

aon::Odometry odometry = aon::Odometry(5, -6, 7, 0, 14);

// Drivetrain
aon::HDrive drivetrain = aon::HDrive({-1, -2, 3, 4}, {12, -13, -18, 19}, {15}, std::make_unique<aon::Odometry>(odometry));

aon::Intake intake = aon::Intake({20, -11, -16}, {17}, 'Z', 0, 10);

pros::ADIDigitalOut semPiston('Z'); // Shrek Ear Mechanism
pros::ADIDigitalOut brooksPiston('Z');

void activateSEM() { semPiston.set_value(HIGH); }

void deactivateSEM() { semPiston.set_value(LOW); }

void activateBrooks() { brooksPiston.set_value(HIGH); }

void deactivateBrooks() { brooksPiston.set_value(LOW); }

#else

// aon::XDrive drivetrain = aon::XDrive({-13}, {11}, {-12}, {14});
aon::Odometry odometry = aon::Odometry(-5, 6, 14, 0, 7);

aon::TankDrive drivetrain = aon::TankDrive({1, 2, -3, -4}, {-16, -17, 18, 19}, std::make_unique<aon::Odometry>(odometry));

aon::Intake intake = aon::Intake({-11, 20}, {15}, {-12}, 'Z', 'Z', 0, 8);

pros::ADIDigitalOut arrowPiston('Z');

void activateArrow() { arrowPiston.set_value(HIGH); }

void deactivateArrow() { arrowPiston.set_value(LOW); }

#endif

// Misc
aon::Orbit orbit(0,true,0,0);

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================

// Encoders
pros::Rotation turretEncoder(0, true);

pros::ADIEncoder opticalEncoder('Z', 'Z');

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
pros::ADIPotentiometer potentiometer('Z');

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
  
  intake.configure(okapi::AbstractMotor::brakeMode::brake, okapi::AbstractMotor::gearset::blue);
  
  #else
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue);
  
  intake.configure(okapi::AbstractMotor::brakeMode::coast, okapi::AbstractMotor::gearset::blue);

  #endif
  orbit.configure();
}

/// @brief Stops movement from robot
void STOP(){
  drivetrain.stop();
  intake.stop();
  orbit.stop();
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
    while(mainController.get_digital(DIGITAL_X)){
      STOP();
    }
    pros::delay(50);
  }
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
