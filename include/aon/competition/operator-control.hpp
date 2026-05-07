#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"

/// @brief Encapsulates functions and state for operator control.
/// @details Practically uses Singleton design pattern, but classes would have
///          made it more complicated for beginners to understand. Also makes extensive
///          use of USING_BIG_ROBOT global constant and preprocessor directives to
///          make switching between robots not require separate branches, which could make
///          fixes and updates to one branch not apply to the other.
namespace aon::operator_control {


// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================

#if USING_BIG_ROBOT
bool sortActive = false;
bool sortEnabled = true;
#else
size_t lastR1PressTime = 0;
size_t lastR2PressTime = 0;
const int DOUBLE_TAP_TIME = 250;
bool mergeCorridorAndElevator = true;
#endif


/// Default Operator Control configuration
inline void DriveDefault() { }

/// Kevin's Operator Control configuration
inline void DriveKevin() { 
  #if !USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = scaler.transform(mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler.transform(mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler.transform(mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler.transform(mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::SPLIT_ARCADE);

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

  // Lever
  if(mainController.get_digital_new_press(DIGITAL_R1)) {
    size_t currentTime = pros::millis();

    if(currentTime - lastR1PressTime < DOUBLE_TAP_TIME){
      intake.resetLever();
    } else {
      intake.extendLever();
    }

    lastR1PressTime = currentTime;
  } else if (intake.leverFinished()) {
    intake.resetLever();
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
    arrow.deactivate();
  } else {
    arrow.activate();
  }

  #endif
}

/// Fabian's Operator Control configuration
inline void DriveFabian() {
  #if USING_BIG_ROBOT
  //# From now on, all drivetrains used will need to use this format for driving
  double leftX = scaler.transform(-mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler.transform(-mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler.transform(-mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler.transform(-mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::HOLONOMIC);

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
  #endif
}

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
/// @see aon::operator_control::Driver
inline void Run(const Driver driver) {
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

}  // namespace aon::operator_control
