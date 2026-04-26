#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"

/// @brief Encapsulates functions and state for operator control.
/// @details Practically uses Singleton design pattern, but classes would have
///          made it more complicated for beginners to understand. Also makes extensive
///          use of USING_BIG_ROBOT global constant and preprocessor directives to
///          make switching between robots not require separate branches, which could make
///          fixes and updates to one branch not apply to the other. Finally, it includes
///          tests for practically all of the fundamental functions except the driver
///          profiles and the Run function.
namespace aon::operator_control {

// ============================================================================
//    _  _     _                 ___             _   _
//   | || |___| |_ __  ___ _ _  | __|  _ _ _  __| |_(_)___ _ _  ___
//   | __ / -_) | '_ \/ -_) '_| | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_||_\___|_| .__/\___|_|   |_| \_,_|_||_\__|\__|_\___/_||_/__/
//              |_|
// ============================================================================

/// @brief Scales analog joystick input for easier control.
/// @details Fine joystick control can be difficult, specially for tasks like
///          rotating. After researching the forums I found that teams scale their
///          joystick inputs using an exponential function of sorts. This makes small
///          inputs produce a smaller output and bigger inputs increase speed, so fine
///          movements can be done without as much of a hassle.
/// @param x The controller's user input between -127 and 127
/// @param t Sensitivity (higher is a steeper curve and vice-versa)
/// @return double between -1 and 1
///
/// @see Demonstration of scaling function in Desmos. https://www.desmos.com/calculator/kq9hgbxbwp
/// @warning Make sure that the input `x` is between -127 and 127!!!
inline double AnalogInputScaling(const double& x, const double& t) {
  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(x) - 127.0) / 10.0);

  return (a + b * (1 - a)) * x / 127.0;
}

// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================

#if USING_BIG_BOT
#else
size_t lastR1PressTime = 0;
size_t lastR2PressTime = 0;
const int DOUBLE_TAP_TIME = 250;
bool mergeCorridorAndElevator = true;
#endif

bool sortActive = false;
bool sortEnabled = true;

/// Default Operator Control configuration
inline void DriveDefault() { 
  //////////// DRIVE ////////////
  
  #if USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = -AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_X), SENSITIVITY);
  double leftY = -AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_Y), SENSITIVITY);
  double rightX = -AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_X), SENSITIVITY);
  double rightY = -AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_Y), SENSITIVITY);
  drivetrain.drive(leftX, leftY, rightX, rightY);
  
  // TODO: discuss with driver if he wants this functionality (probably will)
  // if(mainController.get_digital(DIGITAL_R1)){
  //   intake.activateScan();
  //   intake.elevator();
  // }
  // else {
  //   intake.stopScan();
  // }

  if(mainController.get_digital(DIGITAL_L1)){
    intake.store();
  }
  else if(mainController.get_digital(DIGITAL_L2)){
    intake.score(Intake::BOTTOM);
  }
  else if(!sortActive){
    intake.stop();
  }

  // Evaluate new_press unconditionally so internal state resets on release
  bool r1NewPress = mainController.get_digital_new_press(DIGITAL_R1);
  bool r2NewPress = mainController.get_digital_new_press(DIGITAL_R2);

  if (sortEnabled) {
    // R1 held — sort normally (correct→TOP, wrong→MIDDLE)
    if(mainController.get_digital(DIGITAL_R1)) {
      if(r1NewPress) {
        intake.setSortHeights(Intake::TOP);
        intake.startReleasing();
        sortActive = true;
      }
    }
    // R2 held — sort inverted (correct→MIDDLE, wrong→TOP)
    else if(mainController.get_digital(DIGITAL_R2)) {
      if(r2NewPress) {
        intake.setSortHeights(Intake::MIDDLE);
        intake.startReleasing();
        sortActive = true;
      }
    }
    // neither held — stop sorting only if it was previously active
    else if(sortActive) {
      intake.stopReleasing();
      sortActive = false;
    }
  } else {
    // Sort off — reuse scoring behavior
    if(mainController.get_digital(DIGITAL_R1)) {
      intake.score(Intake::TOP);
    } else if(mainController.get_digital(DIGITAL_R2)) {
      intake.score(Intake::MIDDLE);
    }
  }

  // Change Brooks Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    brooks.toggle();
  }

  else if(mainController.get_digital_new_press(DIGITAL_LEFT)) {
    sem.toggle();
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    intake.toggleCart();
  }

  else if(mainController.get_digital_new_press(DIGITAL_X)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    sortEnabled = !sortEnabled;
    if (!sortEnabled && sortActive) {
      intake.stopReleasing();
      sortActive = false;
    }
  }


  #else

  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_X), SENSITIVITY);
  double leftY = AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_Y), SENSITIVITY);
  double rightX = AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_X), SENSITIVITY);
  double rightY = AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_Y), SENSITIVITY);
  drivetrain.drive(leftX, leftY, rightX, rightY);

  if(mainController.get_digital_new_press(DIGITAL_R2)) {
    size_t currentTime = pros::millis();

    if(currentTime - lastR2PressTime < DOUBLE_TAP_TIME){
      toggle(mergeCorridorAndElevator);
    }

    lastR2PressTime = currentTime;
  }

  // Storing
  if(mainController.get_digital(DIGITAL_R2)) {
    if (mergeCorridorAndElevator){
      intake.store();
    } else {
      intake.corridor();
    }
  }
  // Reject
  else if(mainController.get_digital(DIGITAL_L2)) {
    intake.reject();
  }
  // Score Low
  else if(mainController.get_digital(DIGITAL_L1)) {
    intake.score(Intake::BOTTOM);
  }

  // Lever // TODO: make this behavior native to the intake class
  if(mainController.get_digital_new_press(DIGITAL_R1)) {
    size_t currentTime = pros::millis();

    if(currentTime - lastR1PressTime < DOUBLE_TAP_TIME){
      intake.leverController->setTarget(0);
    } else {
      intake.leverController->setTarget(150);
    }

    lastR1PressTime = currentTime;
  } else if (intake.leverController->getError() < 10) {
    intake.leverController->setTarget(0);
  }

  // Optional single tap
  // Lever
  // const bool pressedR1 = mainController.get_digital_new_press(DIGITAL_R1);
  // if(pressedR1 && intake.leverController->getTarget() == 0 && intake.leverController->getError() < 10){
  //   intake.leverController->setTarget(140);
  // } else if ((pressedR1 && intake.leverController->getTarget() == 140 && !intake.leverController->isSettled())
  //             || (intake.leverController->getTarget() == 140 && intake.leverController->getError() < 10)){
  //   intake.leverController->setTarget(0);
  // } 

  if(!(mainController.get_digital(DIGITAL_R2) || mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_L1))){
    intake.corridor(0);
    intake.elevator(0);
    intake.judge(0);
  }
  
  // Change Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    intake.toggleScorerHeight();
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_A)) {
    intake.toggleCart();
  }
  else if(mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    intake.toggleTrapdoor();
  }
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    brooks.toggle();
  }

  if(mainController.get_digital(DIGITAL_DOWN)) {
    arrow.activate();
  } else {
    arrow.deactivate();
  }

  #endif
}

/// Kevin's Operator Control configuration
inline void DriveKevin() { DriveDefault(); }

/// Fabian's Operator Control configuration
inline void DriveFabian() { DriveDefault(); }

// ============================================================================
//    __  __      _        ___             _   _
//   |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _
//   | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \
//   |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_|
//
// ============================================================================

/// @brief Main function for operator control.
/// @details Control configurations for the different drivers are manipulated here.
/// @param driver the name of the person driving the robot
/// @see aon::operator_control::Drivers
inline void Run(const Drivers driver) {
  switch (driver) {
    case KEVIN:
      DriveKevin();
      break;

    case FABIAN:
      DriveFabian();
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

/// @brief Tests for the operator_control namespace
/// @details Tests helper methods and input scaling. These tests are pretty manual for now, but hopefully next year we'll have automated tests with a solid framework.
namespace test {

}  // namespace test

}  // namespace aon::operator_control
