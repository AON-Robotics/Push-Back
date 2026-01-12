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

/// @brief Scales a joystick input to drivetrain motor intensity according to a percentage
/// @param input The joystick input to be scaled
/// @param percentage The percentage of the drivetrain's `MAX_RPM` to scale to
/// @return The `input` scaled to the `MAX_RPM` of the drivetrain as per `percentage`
inline double ApplySpeed(const double& input, const double& percentage){
  return input * MAX_RPM * percentage;
}

// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================

#if USING_BIG_ROBOT
bool shrimpOut = false;
bool brooksUp = false;
bool wingsOut = false;
bool turbo = false;
#else
bool cartOut = false;
bool scorerUp = false;
bool armOut = false;
bool trapdoorOpen = false;
bool turbo = false;
#endif

/// Default Operator Control configuration
inline void DriveDefault() { 
  //////////// DRIVE ////////////
  const double scaledVertical = AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_Y), SENSITIVITY);
  const double scaledTurn = AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_X), SENSITIVITY);
  const double turn = ApplySpeed(scaledTurn, turbo ? 1 : 0.4);
  
  #if USING_BIG_ROBOT
  const double scaledHorizontal = AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_X), SENSITIVITY);
  
  const double vertical = ApplySpeed(scaledVertical, turbo ? 1.41421356237 : 0.6);
  const double horizontal = ApplySpeed(scaledHorizontal, turbo ? 1.41421356237 : 0.6);
  
  mid.moveVelocity(horizontal);
  #else
  const double vertical = ApplySpeed(scaledVertical, turbo ? 1 : 0.6);
  #endif
  
  drivetrain.driveWhileTurning(vertical, turn);

  //# From now on, all drivetrains used will need to use this format for driving
  if(false){
    double leftX = AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_X), SENSITIVITY) * MAX_RPM;
    double leftY = AnalogInputScaling(mainController.get_analog(ANALOG_LEFT_Y), SENSITIVITY) * MAX_RPM;
    double rightX = AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_X), SENSITIVITY) * MAX_RPM;
    double rightY = AnalogInputScaling(mainController.get_analog(ANALOG_RIGHT_Y), SENSITIVITY) * MAX_RPM;
    drivetrain.drive(leftX, leftY, rightX, rightY);
  }

  #if USING_BIG_ROBOT
  // Score Mid from Bottom
  if(mainController.get_digital(DIGITAL_A)){
    intake.score(Intake::MIDDLE, Intake::BOTTOM);
  }

  else if(mainController.get_digital(DIGITAL_R2)){
    intake.store();
  }

  else if(mainController.get_digital(DIGITAL_Y)){
    intake.hoard();
  }

  else if(mainController.get_digital(DIGITAL_L2)){
    intake.score(Intake::BOTTOM);
  } 
  // Score Mid from Top
  else if(mainController.get_digital(DIGITAL_RIGHT)){
    intake.score(Intake::MIDDLE, Intake::TOP);
  }

  else {
    intake.elevator(0);
    intake.scorer(0);
    intake.topHoarder(0);
    intake.bottomHoarder(0);
  }

  // Score High
  if(mainController.get_digital(DIGITAL_B)) {
    intake.shotbelt();
    intake.shooter(200);
  }

  if(!(mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_B) || mainController.get_digital(DIGITAL_RIGHT))) {
    intake.shooter(0);
  }

  if(!(mainController.get_digital(DIGITAL_R2) || mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_RIGHT) || mainController.get_digital(DIGITAL_X) || mainController.get_digital(DIGITAL_B))){
    intake.shotbelt(0);
  }

  // Change Brooks Height
  if(mainController.get_digital_new_press(DIGITAL_DOWN)) {
    brooksPiston.set_value(toggle(brooksUp) ? HIGH : LOW);
  }
  // Toggle Wings
  else if(mainController.get_digital_new_press(DIGITAL_UP)) {
    wingsPistons.set_value(toggle(wingsOut) ? HIGH : LOW);
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_LEFT)) {
    toggle(shrimpOut) ? intake.dropShrimp() : intake.raiseShrimp();
  }
  // Toggle turbo
  else if(mainController.get_digital_new_press(DIGITAL_X)) {
    toggle(turbo);
  }

  #else
  // Storing
  if(mainController.get_digital(DIGITAL_R2)) {
    intake.store();
  }
  // Score Top
  if(mainController.get_digital(DIGITAL_R1)) {
    intake.scorer();
  }
  // Reject
  else if(mainController.get_digital(DIGITAL_L2)) {
    intake.reject();
  }
  // Score Low
  else if(mainController.get_digital(DIGITAL_L1)) {
    intake.score(Intake::BOTTOM);
  }
  else {
    intake.scorer(0);
  }
  
  if(!(mainController.get_digital(DIGITAL_R2) || mainController.get_digital(DIGITAL_L2) || mainController.get_digital(DIGITAL_L1))){
    intake.elevator(0);
    intake.judge(0);
  }

  // Toggle Arm
  if(mainController.get_digital_new_press(DIGITAL_DOWN)) {
    arm.moveAbsolute(armOut ? 20 : 90, armOut ? 70 : 200);
    toggle(armOut);
  }
  
  // Change Height
  if(mainController.get_digital_new_press(DIGITAL_B)) {
    intake.setScorerHeight(toggle(scorerUp) ? HIGH : LOW);
  }
  // Match loaders mechanism
  else if(mainController.get_digital_new_press(DIGITAL_A)) {
    toggle(cartOut) ? intake.dropCart() : intake.raiseCart();
  }
  // Toggle turbo
  else if(mainController.get_digital_new_press(DIGITAL_RIGHT)) {
    toggle(turbo);
  }
  // Toggle trapdoor
  else if(mainController.get_digital_new_press(DIGITAL_X)) {
    toggle(trapdoorOpen) ? intake.openTrapdoor() : intake.closeTrapdoor();
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
