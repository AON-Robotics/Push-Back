#include "Hdrivetrain.hpp"


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

/**
 * \brief Compute the time limit for 2D movement
 * 
 * \return Time limit for a 2D movement
 */
double computeTimeout(double distance, double maxVel, double maxAccel) {
    double t_ideal = (distance / maxVel) + (maxVel / maxAccel);
    return t_ideal * 1.5; // safety factor
}


// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================

void HDrive::opcontrol() {
  //////////// DRIVE ////////////
  const double vertical = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, SENSITIVITY);
  const double horizontal = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, SENSITIVITY);
  const double turn = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY) * .8;
  
  driveLeft.moveVelocity(MAX_RPM * std::clamp(vertical + turn, -1.0, 1.0));
  driveRight.moveVelocity(MAX_RPM * std::clamp(vertical - turn, -1.0, 1.0));
  middleMotor.moveVelocity(MAX_RPM * std::clamp(vertical - turn, -1.0, 1.0) * 0.5); // make sure not kill the motor
}

// ============================================================================
//   __  __  _____   _____ __  __ ___ _  _ _____
//  |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================

/*
IDEA
What if we dont stop the robot after a movement? and we choose when the robot should stop after a movement

*/

void move2D(double x, double y, double t = aon::odometry::GetDegrees()) {
  MoveTrapezoid(x, y, t)
}

