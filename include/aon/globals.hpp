#pragma once

#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "./constants.hpp"
#include "../api.h"
#include "../okapi/api.hpp"
#include "./controls/pid/pid.hpp"
#include "./tools/vector.hpp"
#include "./drivetrain/h-drive.hpp"
#include "./intake/intake.hpp"
#include "./drivetrain/differential-drive.hpp"
#include "./orbit/orbit.hpp"
#include "./drivetrain/drivetrain.hpp"
#include "./odometry/odometry.hpp"
#include "./piston/piston.hpp"
#include "./math/scaling/pilons-scaler.hpp"
#include "./math/scaling/exponential-scaler.hpp"
#include "./math/scaling/cubic-scaler.hpp"

namespace aon::operator_control {

/// Driver profiles for all robots
enum Driver {
  KEVIN,
  FABIAN,
  DEFAULT,
};
}  // namespace aon::operator_control

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================


#if USING_BIG_ROBOT

// The scaler choice and subsequently tuning should be done as per driver preference
aon::PilonsScaler scaler = aon::PilonsScaler(SENSITIVITY);
// TODO: get driver feedback from the following
// aon::ExponentialScaler scaler = aon::ExponentialScaler(SENSITIVITY);
// aon::CubicScaler scaler = aon::CubicScaler(1);

aon::operator_control::Driver driver = aon::operator_control::FABIAN;

aon::Pose startingPose = aon::Pose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
aon::Odometry odometry = aon::Odometry(5, -6, 7, 0, 14);

aon::Drivetrain::SpeedFactors speedFactors = aon::Drivetrain::SpeedFactors(0.6, 1.0, 0.6, 1.0, 1.0, 1.0);

aon::MotionProfile xProfile = aon::MotionProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
aon::MotionProfile yProfile = aon::MotionProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
aon::MotionProfile thetaProfile = aon::MotionProfile(MAX_RPM, MAX_ACCEL * 3, MAX_DECEL * 0.8, MAX_ACCEL * 3);

aon::HDrive drivetrain = aon::HDrive(
                          {12, -13, -18, 19},
                          {-1, 2, 3, -4},
                          {-15},
                          startingPose,
                          std::make_unique<aon::Odometry>(odometry),
                          speedFactors,
                          std::make_unique<aon::MotionProfile>(xProfile),
                          std::make_unique<aon::MotionProfile>(yProfile),
                          std::make_unique<aon::MotionProfile>(thetaProfile));
                          
                          aon::Intake intake = aon::Intake({20, -11, -10}, {17}, 'H', 9, 16, 'F', 'E');
                          
                          aon::Piston sem('G', aon::Piston::RETRACTED);
                          aon::Piston brooks('D', aon::Piston::RETRACTED);
  
#else

// The scaler choice and the subsequent tuning should be done as per driver preference

// The joystick scaler object to smoothen driver input
aon::PilonsScaler scaler = aon::PilonsScaler(SENSITIVITY);
// TODO: get driver feedback from the following
// aon::ExponentialScaler scaler = aon::ExponentialScaler(SENSITIVITY);
// aon::CubicScaler scaler = aon::CubicScaler(1);

aon::operator_control::Driver driver = aon::operator_control::KEVIN;

// aon::XDrive drivetrain = aon::XDrive({-13}, {11}, {-12}, {14});
aon::Pose startingPose = aon::Pose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
aon::Odometry odometry = aon::Odometry(19, -18, 5, 0, 16);

aon::Drivetrain::SpeedFactors speedFactors = aon::Drivetrain::SpeedFactors(0.6, 0.0, 0.6, 1.0, 0.0, 0.667);

aon::MotionProfile yProfile = aon::MotionProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
aon::MotionProfile thetaProfile = aon::MotionProfile(MAX_RPM, MAX_ACCEL * 3, MAX_DECEL * 0.8, MAX_ACCEL * 3);

aon::DifferentialDrive drivetrain = aon::DifferentialDrive(
                                    {11, -12, 13, -14},
                                    {1, -2, 3, -4},
                                    startingPose,
                                    std::make_unique<aon::Odometry>(odometry),
                                    speedFactors,
                                    std::make_unique<aon::MotionProfile>(yProfile),
                                    std::make_unique<aon::MotionProfile>(thetaProfile));

aon::Intake intake = aon::Intake({-9}, {-6}, {7}, {-8}, 'H', 'B', 'A', 20, 17);

aon::Piston arrow('C', aon::Piston::RETRACTED);
aon::Piston brooks('G', aon::Piston::RETRACTED);

#endif

// Misc
aon::Orbit orbit(0, true, 0, 0);

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================


/// Set by the GUI; drives color-sort accept/reject logic at runtime.
volatile Alliance ALLIANCE = Alliance::Red;

// Potentiometer
pros::ADIPotentiometer potentiometer('Z');

/// PIDs
aon::PID drivePID = aon::PID(0.02, 0, 0);
aon::PID turnPID = aon::PID(0.002, 0, 0);
aon::PID fastPID = aon::PID(1, 0, 0);


/// Controller
pros::Controller mainController = pros::Controller(CONTROLLER_MASTER);


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
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue, MAX_ACCEL * 0.4);
  
  intake.configure(okapi::AbstractMotor::brakeMode::brake, okapi::AbstractMotor::gearset::blue);
  
  #else
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue, MAX_ACCEL);
  
  intake.configure(okapi::AbstractMotor::brakeMode::coast, okapi::AbstractMotor::gearset::blue);

  intake.stopScan();

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
