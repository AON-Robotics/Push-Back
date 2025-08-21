#ifndef AON_CONTROLS_PID_PID_HPP__
#define AON_CONTROLS_PID_PID_HPP__

#include <cmath>

namespace aon {
/**
 * \brief This class calculates the PID controls in a closed loop system.
 *
 * \note Information and explination about the PID used in this
*     class can be seen in <a href="PID Controller.pdf">Introduction to
Motion PID.pdf</a>
 */
class PID {
 private:
  /// Stores the error value from the previous iteration.
  double last_error = 0;
  /// Setpoint - Process Variable
  double error = 0;
  /// Allows for the mitigation of deviations and corrects said deviations.
  double integral = 0;
  /// Diminishes big changes like oscillation.
  double derivative = 0;
  /// Gains
  double kP, kI, kD, kT;
  /// Stores the value in Output method which is the sum of the pid constants.
  double result;
  /// Starts accumulating the integral when the error reaches a certain value.
  double start_integral;
  /// Keeps the integral from passing an error range.
  double anti_windup;

  /**
   * \brief Calculates the integral using the trapezoidal rule.
   *
   * \details Using the error and the last error as the upper and lower sums
   * of the graph and the kT gain as the difference between them one can
     establish and obtain an oproximate defined integral (delta_integral). When
    start_integral is out of the error range and our integral value added with
    our defined value is greater than then anti_windup we will reset our
    delta_integral and add that *value to the integral until the program
    finished running.
   *
   * \see https://andymath.com/wp-content/uploads/2019/08/Trapezoidal-Rule.jpg
   */
  void calculate_integral() {
    double delta_integral = ((error + last_error) / 2.0) * kT;

    if (error > -start_integral && error < start_integral) {
      if (last_error != 0) {  // If we're NOT in our first iteration
        if (delta_integral + integral > anti_windup) delta_integral = 0;
        if (delta_integral + integral < -anti_windup) delta_integral = 0;
        integral = integral + delta_integral;
      }
    }
  }

 public:
  /**
   * \brief Constructor for PID class with 4 gains and 2 attributes.
   *
   * \param Kp_ Proportional gain (remains constant)
   * \param Ki_ Integral gain (remains constant)
   * \param Kd_ Derivative (remains constant)
   * \param Kt_ Constant time period
   * \param start_integral_ Error at which to start including integral
   * \param anti_windup_ Prevent integral from crossing a range
   */
  PID(double Kp_, double Ki_, double Kd_, double Kt_ = 0.01,
      double start_integral_ = INFINITY, double anti_windup_ = INFINITY) {
    kP = Kp_;
    kI = Ki_;
    kD = Kd_;
    kT = Kt_;
    start_integral = start_integral_;
    anti_windup = anti_windup_;
  }

  /// Retrieves the proprtioanl gain.
  double GetKP() { return kP; }
  /// Retrieves the integral gain.
  double GetKI() { return kI; }
  /// Retrieves the derivative gain.
  double GetKD() { return kD; }
  /// Retrieves the constant time period.
  double GetKT() { return kT; }
  /// Retrieves and accumulates the integral when the error reaches a value.
  double GetStartIntegral() { return start_integral; }
  /// Retrieves the calculated range where the integral must stay within.
  double GetAntiWindup() { return anti_windup; }
  /// Retrieves the error used in the last calculation
  double GetError() { return error; }

  /**
   * \brief Method to calculate and add pid constants.
   *
   * \param setpoint Desired target value we want to achieve.
   * \param process_variable Current value the robot is processing.
   * \return double This is the sum of all the controls in the pid.
   */
  double Output(double setpoint, double process_variable) {
    // error = What we want - what we have
    error = setpoint - process_variable;
    // integral = integral + error * kT;
    calculate_integral();
    derivative = (error - last_error) / kT;

    last_error = error;

    const double p = error * kP;
    const double i = kI * integral;
    const double d = derivative * kD;

    result = p + i + d;
    return result;
  }
  double GetResult() { return result; }
  /**
   * \brief Sets attributes to their original values excluding gains.
   *
   */
  void Reset() {
    last_error = 0;
    error = 0;
    integral = 0;
    derivative = 0;
    result = 0;
  }
};
}  // namespace aon

#endif  // AON_CONTROLS_PID_PID_HPP__
