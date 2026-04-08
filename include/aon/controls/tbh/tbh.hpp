#ifndef AON_CONTROLS_TBH_TBH_HPP__
#define AON_CONTROLS_TBH_TBH_HPP__

#include <algorithm>
#include <cmath>

namespace aon {
/**
 * \brief This class calculates the Take Back Half (TBH) controls for velocity
 * regulation in a closed loop system.
 *
 * \details TBH is a velocity control algorithm designed for systems like
 * flywheels where a consistent output speed is desired. Instead of tuning
 * three gains like PID, TBH only requires a single gain and a stored
 * "take-back" value. When the error crosses zero (meaning the system has
 * overshot), the output is collapsed to the midpoint between the current
 * output and the last stored TBH value. This midpoint is then saved as the
 * new TBH value, causing the controller to converge quickly to a steady state.
 *
 * \note The output represents a motor drive value and is clamped to
 * [-1.0, 1.0].
 */
class TBH {
 private:
  /// Error from the previous iteration used to detect zero crossings.
  double prev_error = 0;
  /// Setpoint - Process Variable
  double error = 0;
  /// Accumulated drive output, updated each iteration by gain * error.
  double output = 0;
  /// Stored "take-back" value set whenever error crosses zero.
  double tbh = 0;
  /// Single tuning gain that scales how aggressively output is adjusted.
  double gain;
  /// Cached result from the last call to Output().
  double result = 0;

  /**
   * \brief Returns the sign of a value as -1, 0, or 1.
   *
   * \param value The value to evaluate.
   * \return double -1.0 if negative, 0.0 if zero, 1.0 if positive.
   */
  double sign(double value) const {
    if (value > 0.0) return 1.0;
    if (value < 0.0) return -1.0;
    return 0.0;
  }

 public:
  /**
   * \brief Constructor for the TBH class.
   *
   * \param gain_ Tuning gain that scales the rate of output change per unit
   *              of error. Larger values respond faster but risk oscillation.
   */
  explicit TBH(double gain_) { gain = gain_; }

  /// Retrieves the tuning gain.
  double GetGain() const { return gain; }
  /// Sets the tuning gain for live tuning.
  void SetGain(double v) { gain = v; }
  /// Retrieves the error used in the last calculation.
  double GetError() const { return error; }
  /// Retrieves the current accumulated output before clamping.
  double GetOutput() const { return output; }
  /// Retrieves the stored take-back half value.
  double GetTBH() const { return tbh; }
  /// Retrieves the result from the last call to Output().
  double GetResult() const { return result; }

  /**
   * \brief Calculates and returns the drive output using the TBH algorithm.
   *
   * \details Each call accumulates \c gain * error into the output. When the
   * sign of the error changes relative to the previous iteration (a zero
   * crossing), the output is set to the average of the current output and
   * the stored TBH value, and that average becomes the new TBH value. The
   * final output is clamped to [-1.0, 1.0].
   *
   * \param setpoint        Desired target velocity we want to achieve.
   * \param process_variable Current velocity the system is producing.
   * \return double The drive output in the range [-1.0, 1.0].
   */
  double Output(double setpoint, double process_variable) {
    error = setpoint - process_variable;

    output += gain * error;
    output = std::clamp(output, -1.0, 1.0);

    // Zero crossing detected: take back half
    if (sign(error) != sign(prev_error) && prev_error != 0) {
      output = (output + tbh) / 2.0;
      tbh = output;
    }

    prev_error = error;

    result = output;
    return result;
  }

  /**
   * \brief Resets all state to initial values, excluding the gain.
   *
   */
  void Reset() {
    prev_error = 0;
    error = 0;
    output = 0;
    tbh = 0;
    result = 0;
  }
};
}  // namespace aon

#endif  // AON_CONTROLS_TBH_TBH_HPP__
