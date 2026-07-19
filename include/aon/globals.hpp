#pragma once

#ifndef AON_GLOBALS_HPP_
#define AON_GLOBALS_HPP_

#include "./constants.hpp"
#include "./control/driver.hpp"
#include "./core/hardware.hpp"
#include "../api.h"
#include "./auton/actions.hpp"
#include "./compat/okapi.hpp"
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

// ============================================================================
//   __  __  ___ _____ ___  ___  ___ 
//  |  \/  |/ _ \_   _/ _ \| _ \/ __|
//  | |\/| | (_) || || (_) |   /\__ \
//  |_|  |_|\___/ |_| \___/|_|_\|___/
//
// ============================================================================


inline auto& scaler = aon::core::hardware().scaler;
inline auto& driver = aon::core::hardware().driver;
inline auto& startingPose = aon::core::hardware().startingPose;
inline auto& odometry = aon::core::hardware().odometry;
inline auto& speedFactors = aon::core::hardware().speedFactors;
#if USING_BIG_ROBOT
inline auto& xProfile = aon::core::hardware().xProfile;
#endif
inline auto& yProfile = aon::core::hardware().yProfile;
inline auto& thetaProfile = aon::core::hardware().thetaProfile;
inline auto& drivetrain = aon::core::hardware().drivetrain;
inline auto& intake = aon::core::hardware().intake;
#if USING_BIG_ROBOT
inline auto& sem = aon::core::hardware().sem;
#else
inline auto& arrow = aon::core::hardware().arrow;
#endif
inline auto& brooks = aon::core::hardware().brooks;
inline auto& orbit = aon::core::hardware().orbit;

// ============================================================================
//   ___ ___ _  _ ___  ___  ___  ___ 
//  / __| __| \| / __|/ _ \| _ \/ __|
//  \__ \ _|| .` \__ \ (_) |   /\__ \
//  |___/___|_|\_|___/\___/|_|_\|___/
//
// ============================================================================


/// Set by the GUI; drives color-sort accept/reject logic at runtime.
inline volatile Alliance& ALLIANCE = aon::core::hardware().alliance;

// Potentiometer
inline auto& potentiometer = aon::core::hardware().potentiometer;

/// PIDs
inline auto& drivePID = aon::core::hardware().drivePID;
inline auto& turnPID = aon::core::hardware().turnPID;
inline auto& fastPID = aon::core::hardware().fastPID;


/// Controller
inline auto& mainController = aon::core::hardware().mainController;


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
  
  intake.configure(pros::MotorBrake::brake, pros::MotorGears::blue);
  
  #else
  drivetrain.configure(brakeMode, okapi::AbstractMotor::gearset::blue, MAX_ACCEL);
  
  intake.configure(pros::MotorBrake::coast, pros::MotorGears::blue);

  intake.stopScan();

  #endif
  orbit.configure();
}

/// @brief Stops movement from robot
inline void STOP(){
  aon::auton::actions().cancelMotion();
  drivetrain.stop();
  intake.stop();
  orbit.stop();
}

/// @brief Used to make sure a condition is being met or a block of code is being run
/// @param speed The speed with which to spin the intake to differentiate between multiple tests
/// @note `speed` should vary if running multiple tests in one same run to be able to tell apart between them
inline void testEndpoint(int speed = 100){
  STOP(); 
  intake.move(speed);
  pros::delay(1000);
  intake.stop();
}

/// @brief Task to stop all motors during auton testing if something goes wrong
inline void autonSafety(){
  while(true){
    while(mainController.get_digital(DIGITAL_X)){
      STOP();
    }
    pros::delay(50);
  }
}

}  // namespace aon

#endif  // AON_GLOBALS_HPP_
