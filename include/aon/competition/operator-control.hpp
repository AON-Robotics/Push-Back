#pragma once

#include <algorithm>
#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/shadow/mechanisms.hpp"
#include "lemlib/api.hpp"

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

inline int scaleLemLibDriveInput(const int rawInput, const double scale) {
  return static_cast<int>(
      std::clamp(rawInput * scale, -127.0, 127.0));
}

inline void DriveKevinLemLibCurvature() {
  const double throttleScale = drivetrain.isTurbo() ? 1.0 : 0.6;
  const double turnScale = drivetrain.isTurbo() ? 0.667 : 0.6;
  const int throttle = scaleLemLibDriveInput(
      mainController.get_analog(ANALOG_LEFT_Y), throttleScale);
  const int turn = scaleLemLibDriveInput(
      mainController.get_analog(ANALOG_RIGHT_X), turnScale);

  aon::lemlib_integration::chassis().curvature(throttle, turn);
  aon::shadow::captureDrive(std::clamp(throttle + turn, -127, 127),
                            std::clamp(throttle - turn, -127, 127));
}


/// Default Operator Control configuration
inline void DriveDefault() { }

/// Kevin's Operator Control configuration
inline void DriveKevin() { 
  #if !USING_BIG_ROBOT
#if USE_LEMLIB_CURVATURE_DRIVER
  DriveKevinLemLibCurvature();
#else
  double leftX = scaler.transform(mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler.transform(mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler.transform(mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler.transform(mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::SPLIT_ARCADE);
  const double forwardFactor = drivetrain.isTurbo()
                                   ? speedFactors.forwardTurbo
                                   : speedFactors.forwardNoTurbo;
  const double turnFactor = drivetrain.isTurbo()
                                ? speedFactors.turnTurbo
                                : speedFactors.turnNoTurbo;
  const auto driveIntent = aon::shadow::normalizedArcadeDrive(
      leftY * forwardFactor, rightX * turnFactor);
  aon::shadow::captureDrive(driveIntent.left, driveIntent.right);
#endif

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
  const auto intakeIntent = mainController.get_digital(DIGITAL_R2)
      ? (mergeCorridorAndElevator ? aon::shadow::IntakeIntent::Store
                                  : aon::shadow::IntakeIntent::Corridor)
      : mainController.get_digital(DIGITAL_L2)
          ? aon::shadow::IntakeIntent::Reject
          : mainController.get_digital(DIGITAL_L1)
              ? aon::shadow::IntakeIntent::ScoreBottom
              : aon::shadow::IntakeIntent::Idle;
  aon::shadow::captureMechanism(
      aon::shadow::MechanismKind::IntakeMode,
      static_cast<std::int16_t>(intakeIntent));
  aon::shadow::captureMechanism(aon::shadow::MechanismKind::Lever,
                                intake.leverTarget != 0.0 ? 1 : 0);
  
  // Change Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    intake.toggleScorerHeight();
    aon::shadow::captureMechanism(
        aon::shadow::MechanismKind::ScorerHeight,
        intake.isScorerRaised() ? 1 : 0);
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_A)) {
    intake.toggleCart();
    aon::shadow::captureMechanism(aon::shadow::MechanismKind::Cart,
                                  intake.isCartDropped() ? 1 : 0);
  }
  else if(mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    drivetrain.toggleTurbo();
  }
  else if(mainController.get_digital_new_press(DIGITAL_Y)) {
    intake.toggleTrapdoor();
    aon::shadow::captureMechanism(aon::shadow::MechanismKind::Trapdoor,
                                  intake.isTrapdoorOpen() ? 1 : 0);
  }
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    brooks.toggle();
    aon::shadow::captureMechanism(aon::shadow::MechanismKind::Brooks,
                                  brooks.isActivated() ? 1 : 0);
  }

  if(mainController.get_digital(DIGITAL_DOWN)) {
    arrow.deactivate();
  } else {
    arrow.activate();
  }
  aon::shadow::captureMechanism(aon::shadow::MechanismKind::Arrow,
                                arrow.isActivated() ? 1 : 0);

  #endif
}

/// Fabian's Operator Control configuration
inline void DriveFabian() {
  #if USING_BIG_ROBOT
  double leftX = scaler.transform(-mainController.get_analog(ANALOG_LEFT_X));
  double leftY = scaler.transform(-mainController.get_analog(ANALOG_LEFT_Y));
  double rightX = scaler.transform(-mainController.get_analog(ANALOG_RIGHT_X));
  double rightY = scaler.transform(-mainController.get_analog(ANALOG_RIGHT_Y));
  drivetrain.drive(leftX, leftY, rightX, rightY, Drivetrain::HOLONOMIC);
  const double forwardFactor = drivetrain.isTurbo()
                                   ? speedFactors.forwardTurbo
                                   : speedFactors.forwardNoTurbo;
  const double turnFactor = drivetrain.isTurbo()
                                ? speedFactors.turnTurbo
                                : speedFactors.turnNoTurbo;
  const auto driveIntent = aon::shadow::normalizedArcadeDrive(
      leftY * forwardFactor, rightX * turnFactor);
  aon::shadow::captureDrive(driveIntent.left, driveIntent.right);

  auto intakeIntent = aon::shadow::IntakeIntent::Idle;

  if(mainController.get_digital(DIGITAL_L1)){
    intake.store();
    intakeIntent = aon::shadow::IntakeIntent::Store;
  }
  else if(mainController.get_digital(DIGITAL_L2)){
    intake.score(Intake::BOTTOM);
    intakeIntent = aon::shadow::IntakeIntent::ScoreBottom;
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
      intakeIntent = aon::shadow::IntakeIntent::SortNormal;
      if(r1NewPress) {
        intake.setSortHeights(Intake::TOP);
        intake.startReleasing();
        sortActive = true;
      }
    }
    // R2 held — sort inverted (correct→MIDDLE, wrong→TOP)
    else if(mainController.get_digital(DIGITAL_R2)) {
      intakeIntent = aon::shadow::IntakeIntent::SortInverted;
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
      intakeIntent = aon::shadow::IntakeIntent::ScoreTop;
    } else if(mainController.get_digital(DIGITAL_R2)) {
      intake.score(Intake::MIDDLE);
      intakeIntent = aon::shadow::IntakeIntent::ScoreMiddle;
    }
  }
  aon::shadow::captureMechanism(
      aon::shadow::MechanismKind::IntakeMode,
      static_cast<std::int16_t>(intakeIntent));

  // Change Brooks Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    brooks.toggle();
    aon::shadow::captureMechanism(aon::shadow::MechanismKind::Brooks,
                                  brooks.isActivated() ? 1 : 0);
  }

  else if(mainController.get_digital_new_press(DIGITAL_LEFT)) {
    sem.toggle();
    aon::shadow::captureMechanism(aon::shadow::MechanismKind::Sem,
                                  sem.isActivated() ? 1 : 0);
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    intake.toggleCart();
    aon::shadow::captureMechanism(aon::shadow::MechanismKind::Cart,
                                  intake.isCartDropped() ? 1 : 0);
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
