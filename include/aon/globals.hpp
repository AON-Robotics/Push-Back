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
#include "./piston/piston.hpp"
#include "EKF/sensor_feeder.hpp"
#include "./controls/holonomic-pure-pursuit/hpp.hpp"
#include "aon/struct_master.hpp"   // PipelineConfig


aon::TankDrive drivetrainTank2(
  {kRobotConfig.lm1, kRobotConfig.lm2, kRobotConfig.lm3, kRobotConfig.lm4},
  {kRobotConfig.rm1, kRobotConfig.rm2, kRobotConfig.rm3, kRobotConfig.rm4}
);
aon::TankDriveConfig drivetrainTank2Cfg   = kRobotConfig.toTankDriveConfig();
aon::TankDriveFeeder drivetrainFeeder2(drivetrainTank2Cfg);

// Encoders

pros::Rotation encoderLeft (kRobotConfig.encoder_left_port,  kRobotConfig.encoder_left_reversed);
pros::Rotation encoderRight(kRobotConfig.encoder_right_port, kRobotConfig.encoder_right_reversed);
pros::Gps gps(12, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);
pros::Imu imu(kRobotConfig.imu_port);

// Sensor Feeder Instance
aon::EKFConfig  sensorFeederCfg = kRobotConfig.toEKFConfig();
EKF::TankConfig ekfCfg          = kRobotConfig.toEKFTankConfig();
EKF ekf(ekfCfg);

aon::SensorFeeder sensorFeeder(
  encoderLeft,
  encoderRight,
  nullptr,
  imu,
  &gps,
  sensorFeederCfg
);

// ============================================================
// HPP Controller
// ============================================================
// TankDrive
//Feeder.applyDriveDefaults() connects hppCfg.mp_cfg to TankDrive
//.
aon::hpp::Config     hppCfg = kRobotConfig.toHppConfig();
aon::hpp::Controller hpp(hppCfg);



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
aon::HDrive drivetrain = aon::HDrive({12, -13, -18, 19}, {-1, 2, 3, -4}, {-15}, std::make_unique<aon::Odometry>(odometry));

aon::Intake intake = aon::Intake({20, -11, -10}, {17}, 'H', 9, 16);

aon::Piston sem('G', aon::Piston::RETRACTED);
aon::Piston brooks('D', aon::Piston::RETRACTED);

#else

// aon::XDrive drivetrain = aon::XDrive({-13}, {11}, {-12}, {14});
aon::Odometry odometry = aon::Odometry(19, -18, 5, 0, 16);

aon::TankDrive drivetrain = aon::TankDrive({11, -12, 13, -14}, {1, -2, 3, -4}, std::make_unique<aon::Odometry>(odometry));

aon::Intake intake = aon::Intake({-9, -6}, {7}, {-8}, 'H', 'B', 'A', 20, 17);

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
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue, MAX_ACCEL * 0.4);
  
  intake.configure(okapi::AbstractMotor::brakeMode::brake, okapi::AbstractMotor::gearset::blue);
  
  #else
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue, MAX_ACCEL);
  
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
