#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"

/**
 * \brief Encapsulates functions and state for operator control.
 *
 * \details Practically uses Singleton design pattern, but classes would have
 * made it more complicated for beginners to understand. Also makes extensive
 * use of USING_BLACK_ROBOT global constant and preprocessor directives to
 * make switching between robots not require separate branches, which could make
 * fixes and updates to one branch not apply to the other. Finally, it includes
 * tests for practically all of the fundamental functions except the driver
 * profiles and the Run function.
 *
 */
namespace aon::operator_control {

// ============================================================================
//    _  _     _                 ___             _   _
//   | || |___| |_ __  ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
//   | __ / -_) | '_ \/ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_||_\___|_| .__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
//              |_|
// ============================================================================

/**
 * \brief Scales analog joystick input for easier control.
 *
 * \details Fine joystick control can be difficult, specially for tasks like
 *     rotating. After researching the forums I found that teams scale their
 *     joystick inputs using an exponential function of sorts. This makes small
 *     inputs produce a smaller output and bigger inputs increase speed, so fine
 *     movements can be done without as much of a hassle.
 *
 * \param x The controller's user input between -1 and 1
 * \param t Sensitivity (higher is more sensible and vice-versa)
 *
 * <a href="https://www.desmos.com/calculator/uhjyivyj4r">Demonstration of
 * scaling function in Desmos.</a>
 *
 * \return double
 *
 * \warning Make sure that the input x is between -1 and 1!!!
 */
inline double AnalogInputScaling(const double x, const double t) {
  const double z = 127.0 * x;
  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(z) - 127.0) / 10.0);

  return (a + b * (1 - a)) * z / 127.0;
}

// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================


/// Default Operator Control configuration
inline void DriveDefault() { 
  //////////// DRIVE ////////////
  const double vertical = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, SENSITIVITY);
  const double turn = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY) * .8;
  
  driveLeft.moveVelocity(MAX_RPM * std::clamp(vertical + turn, -1.0, 1.0) * .9);
  driveRight.moveVelocity(MAX_RPM * std::clamp(vertical - turn, -1.0, 1.0) * .9);
  
  //////////// INTAKE ////////////
  
  if (mainController.get_digital(DIGITAL_R1)) {
    intake.moveVelocity(INTAKE_VELOCITY);
  } else if (mainController.get_digital(DIGITAL_R2)) {
    intake.moveVelocity(-INTAKE_VELOCITY);
  } else {
    intake.moveVelocity(0);
  }
  
  if (mainController.get_digital_new_press(DIGITAL_A)) 
  { 
    claw.set_value(toggle(clawOn));
  }
  
  if (mainController.get_digital_new_press(DIGITAL_B)) 
  { 
    kickBackRail();
  }
  
  if(mainController.get_digital(DIGITAL_Y)){
    indexer.set_value(true);
  } 
  else {
    indexer.set_value(false);
  }
  
  if (mainController.get_digital(DIGITAL_L1)) {
    arm.moveVelocity(INTAKE_VELOCITY);
  } else if (mainController.get_digital(DIGITAL_L2)) {
    arm.moveVelocity(-INTAKE_VELOCITY);
  } else {
    arm.moveVelocity(0);
  }  
}

/// Ian's Operator Control configuration
inline void DriveIan() { DriveDefault(); }

/// David's Operator Control configuration
inline void DriveDavid() { DriveDefault(); }

// ============================================================================
//    __  __      _        ___             _   _
//   |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _
//   | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \
//   |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_|
//
// ============================================================================

/**
 *\brief Main function for operator control.
 *
 * \details Control configurations for the different drivers are manipulated
 * here.
 *
 * \param driver the name of the person driving the robot
 *
 * \see aon::operator_control::Drivers
 *
 */
inline void Run(const Drivers driver) {
  switch (driver) {
    case IAN:
      DriveIan();
      break;

    case DAVID:
      DriveDavid();
      break;

    default:
      DriveDefault();
      break;
  }
}

// ============================================================================
//    _____       _
//   |_   _|__ __| |_ ___
//     | |/ -_|_-<  _(_-<
//     |_|\___/__/\__/__/
//
// ============================================================================

/**
 * \brief Tests for the operator_control namespace
 *
 * \details Tests helper methods and input scaling. These tests are pretty
 * manual for now, but hopefully next year we'll have automated tests with a
 * solid framework.
 *
 */

namespace test {

}  // namespace test

}  // namespace aon::operator_control
