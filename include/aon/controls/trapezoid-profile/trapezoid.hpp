/**
 * \file trapezoid.hpp
 * \author Marcos R. Pesante Colón (marcos.pesante@upr.edu)
 * \brief Defines self-contained TrapezoidProfile class within `aon` namespace.
 * \version 2.0
 * \date 2022-03-03
 *
 */

#ifndef AON_CONTROLS_TRAPEZOID_PROFILE_TRAPEZOID_HPP_
#define AON_CONTROLS_TRAPEZOID_PROFILE_TRAPEZOID_HPP_

#include <cmath>
#include <cfloat>
#include <exception>
#include <string>
#include "../../tools/logging.hpp"

namespace aon {

/**
 * \brief Calculates optimal motion curves using the shape of a Trapezoid.
 *
 * \details This class helps to model the speed and position of the robot at
 *    different stages of its motion. At the beginning the robot accelerates at
 *    a constant acceleration a_0 until it reaches a maximum velocity V. For a
 *    while, it remains traveling at this speed until the algorithm determines
 *    it's time to start deccelerating at a constant acceleration a_f.
 *
 * <a href="https://www.desmos.com/calculator/nllzeyl2ol">Demonstration</a>
 *
 * \note The explanation and maths required to derive the formulas used in this
 *     class is <a href="Introduction to Motion Profile.pdf">Introduction to
Motion Profile.pdf</a>
 *
 * \code{.cpp}
int main(){
  aon::TrapezoidProfile profile = aon::TrapezoidProfile(2, 1, 1);

  // D = 2, V = 1, T = 3
  std::cout + "D = " + profile.GetD() + ", V = " + profile.GetV()
  + ", T = " + profile.GetT() + std::endl;

  for(double t = 0; t < profile.GetT(); t += profile.GetT() / 10.0)
    std::cout + t + ", " + profile.SpeedProfile(t) + std::endl;
  // Outputs:
  // 0, 0
  // 0.3, 0.3
  // 0.6, 0.6
  // 0.9, 0.9
  // 1.2, 1
  // 1.5, 1
  // 1.8, 1
  // 2.1, 0.9
  // 2.4, 0.6
  // 2.7, 0.3
  // 3, 4.44089e-16
}
 * \endcode
 *
 */
class TrapezoidProfile {
 private:
  // Essential parameters that help shape the entire trapezoid
  /// Total distance to travel within single motion
  double D_;
  /// Maximum speed to travel at within single motion
  double V_;
  /// Total travel time.
  double T_;

  // Essential parameters that shape the 1st and 3rd stages of the motion
  /// Starting speed for motion
  double v_0_;
  /// Initial acceleration for this motion
  double a_0_;
  /// Final decceleration for this motion; MUST be negative for positive a_0_.
  double a_f_;
  /// Final speed for motion; it is usually 0.
  double v_f_;

  // Time durations for each of the 3 stages [seconds]
  //
  /// Time duration of first stage of motion
  double t_0_;
  /// Time duration of second stage of motion
  double t_v_;
  /// Time duration of third and final stage of motion
  double t_f_;

  /// Gain that dictates how much to compensate for overshoots/undershoots
  double correction_ratio_;

  /// Stores precalculated coefficients for the speed profile polynomials.
  double speed_coeffs_[3][2] = {};
  /// Stores precalculated coefficients for the position profile polynomials.
  double position_coeffs_[3][3] = {};

  /**
   * \brief Validates parameters.
   *
   * \details Parameters must be validated to ensure continuity between the
   *     piece-wise curves. Additionally, this method updates the parameters
   *     that are stored within this class, which is essential for the new
   *     parameters to be reflected in the actual motion.
   *
   * \note The parameter V_ is changed in this function to make sure that the
   *     curves are continuous but the other paramaters stay the same as the
   *     programmer wants them.
   */
  void ProcessParams() {
    // These variable's signs MUST be equal, so make sure they match.
    // https://www.cplusplus.com/reference/cmath/copysign/
    V_ = std::copysign(V_, D_);
    v_0_ = std::copysign(v_0_, V_);
    v_f_ = std::copysign(v_f_, V_);
    a_0_ = std::copysign(a_0_, v_0_);
    a_f_ = std::copysign(a_f_, v_f_);

    if (!std::isnormal(a_0_)) {
      aon::logging::Error(
          "[❌] `a_0` is not normal. It is NAN, INFINITY, or 0. `a_0 = " +
          std::to_string(a_0_) + "`\n");
      throw std::domain_error("");
    }

    if (!std::isnormal(a_f_)) {
      aon::logging::Error(
          "[❌] `a_f` is not normal. It is NAN, INFINITY, or 0. `a_f = " +
          std::to_string(a_f_) + "`\n");
      throw std::domain_error("");
    }

    double numerator =
        a_f_ * v_0_ * v_0_ + a_0_ * v_f_ * v_f_ + 2.0 * D_ * a_0_ * a_f_;
    double denominator = a_0_ + a_f_;
    const double max_speed = std::sqrt(numerator / denominator);

    // Constrain V.
    if (V_ < 0.0) {
      // Choose the smallest of v_0, v_f, or V to assign to V.
      V_ = std::fmin(std::fmin(v_0_, v_f_), V_);
      V_ = std::fmax(-max_speed, V_);
    } else {
      // Choose the largest of v_0, v_f, or V to assign to V.
      V_ = std::fmax(std::fmax(v_0_, v_f_), V_);
      V_ = std::fmin(max_speed, V_);
    }

    // Calculate T.
    if (!std::isnormal(V_)) {
      aon::logging::Error(
          "[❌] `V` is not normal. It is NAN, INFINITY, or 0. This means that "
          "`T_` will be indefinite. `V = " +
          std::to_string(V_) + "`\n");
      throw std::domain_error("");
    }

    T_ = D_ / V_ + (V_ - v_0_) * (V_ - v_0_) / (2.0 * V_ * a_0_) +
         (V_ - v_f_) * (V_ - v_f_) / (2.0 * V_ * a_f_);

    if (T_ < -FLT_EPSILON) {
      aon::logging::Error("[❌] `T_` stores time, but it's negative!! T_ = " +
                          std::to_string(T_) + "\n");
      throw std::domain_error("");
    }

    // Prepare additional helper variables.
    t_0_ = (V_ - v_0_) / a_0_;
    t_f_ = (V_ - v_f_) / a_f_;
    t_v_ = T_ - t_0_ - t_f_;

    // t_0_, t_f_, and t_v_ MUST be positive.
    if (t_0_ < -FLT_EPSILON) {
      aon::logging::Error(
          "[❌] `t_0_` stores time, but it's negative!! \nt0 = " +
          std::to_string(t_0_) + ", V_ = " + std::to_string(V_) + ", v_0_ = " +
          std::to_string(v_0_) + ", a_0_ = " + std::to_string(a_0_) + "\n");
      throw std::domain_error("");
    }
    if (t_f_ < -FLT_EPSILON) {
      aon::logging::Error(
          "[❌] `t_f_` stores time, but it's negative!! \ntf = " +
          std::to_string(t_f_) + ", V_ = " + std::to_string(V_) + ", v_f_ = " +
          std::to_string(v_f_) + ", a_f_ = " + std::to_string(a_f_) + "\n");
      throw std::domain_error("");
    }
    if (t_v_ < -FLT_EPSILON) {
      aon::logging::Error(
          "[❌] `t_v_` stores time, but it's negative!! \ntv = " +
          std::to_string(t_v_) + ", T_ = " + std::to_string(T_) + ", t_0_ = " +
          std::to_string(t_0_) + ", t_f_ = " + std::to_string(t_f_) + "\n");
      throw std::domain_error("");
    }

    // Stage 1: Constant acceleration
    speed_coeffs_[0][0] = v_0_;
    speed_coeffs_[0][1] = a_0_;
    position_coeffs_[0][0] = 0.0;
    position_coeffs_[0][1] = v_0_;
    position_coeffs_[0][2] = a_0_ / 2.0;

    // Stage 2: Constant speed
    speed_coeffs_[1][0] = V_;
    speed_coeffs_[1][1] = 0.0;
    position_coeffs_[1][0] = -(V_ - v_0_) * (V_ - v_0_) / (2.0 * a_0_);
    position_coeffs_[1][1] = V_;
    position_coeffs_[1][2] = 0.0;

    // Stage 3: Constant decceleration
    speed_coeffs_[2][0] = T_ * a_f_ + v_f_;
    speed_coeffs_[2][1] = -a_f_;
    position_coeffs_[2][0] = position_coeffs_[1][0];
    position_coeffs_[2][0] -=
        std::pow(T_ * a_f_ + v_f_ - V_, 2.0) / (2.0 * a_f_);
    position_coeffs_[2][1] = T_ * a_f_ + v_f_;
    position_coeffs_[2][2] = -a_f_ / 2.0;
  }

  /**
   * \brief Perform basic checks on parameters.
   *
   * \details Checks:
   *  - Initial and final speed are correct for Speed Profile.
   *  - Initial and final distance are correct for Position Profile.
   *  - All variables representing time are positive.
   */
  void TestBasic() {
    if (SpeedProfile(0.0) != v_0_) {
      aon::logging::Error(
          "[❌] Value Error: `v(0) = " + std::to_string(SpeedProfile(0)) +
          "` is not equal to `v_0 = " + std::to_string(v_0_) + "`\n");
    }

    if (SpeedProfile(T_) != v_f_) {
      aon::logging::Error(
          "[❌] Value Error: `v(T) = " + std::to_string(SpeedProfile(T_)) +
          "` is not equal to `v_f = " + std::to_string(v_f_) + "`\n");
    }

    if (PositionProfile(0.0) != 0.0) {
      aon::logging::Error(
          "[❌] Value Error: `d(0) = " + std::to_string(PositionProfile(0)) +
          "` is not equal to 0\n");
    }

    if (std::fabs(PositionProfile(T_) - D_) > FLT_EPSILON) {
      aon::logging::Error(
          "[❌] Value Error: `d(T) = " + std::to_string(PositionProfile(T_)) +
          "` is not equal to `D = " + std::to_string(D_) + "`\n");
    }

    if (T_ < -FLT_EPSILON) {
      aon::logging::Error(
          "[❌] Value Error: T is negative. `T = " + std::to_string(T_) + "`\n");
    }

    if (t_0_ < -FLT_EPSILON) {
      aon::logging::Error("[❌] Value Error: t_0 is negative. `t_0 = " +
                          std::to_string(t_0_) + "`\n");
    }

    if (t_v_ < -FLT_EPSILON) {
      aon::logging::Error("[❌] Value Error: t_v is negative. `t_v = " +
                          std::to_string(t_v_) + "`\n");
    }

    if (t_f_ < -FLT_EPSILON) {
      aon::logging::Error("[❌] Value Error: t_f is negative. `t_f = " +
                          std::to_string(t_f_) + "`\n");
    }
  }

  /**
   * \brief Check that each transition between stages is continuous.
   *
   * \details Discontinuities can be fatal for the Trapezoidal motion profile.
   *   Therefore, we should check the continuity of both the speed and position
   *   profiles at t_0 and `T - t_f` (whenever there is a change of stage).
   */
  void TestContinuity() {
    // Much like limits in calculus, I'll use epsilon and delta.
    const double epsilon = DBL_MIN;
    const double delta = DBL_EPSILON;

    // Check speed profile.
    double Delta = SpeedProfile(t_0_ + epsilon) - SpeedProfile(t_0_ - epsilon);
    if (std::fabs(Delta) >= delta) {
      aon::logging::Error(
          "[❌] Value Error: Discontinuity found in Speed Profile at `t_0`\n");
    }

    Delta =
        SpeedProfile(T_ - t_f_ + epsilon) - SpeedProfile(T_ - t_f_ - epsilon);
    if (std::fabs(Delta) >= delta) {
      aon::logging::Error(
          "[❌] Value Error: Discontinuity found in Speed Profile at `t_0 + "
          "t_v`\n");
    }

    // Check position profile.
    Delta = PositionProfile(t_0_ + epsilon) - PositionProfile(t_0_ - epsilon);
    if (std::fabs(Delta) >= delta) {
      aon::logging::Error(
          "[❌] Value Error: Discontinuity found in Position Profile at "
          "`t_0`\n");
    }

    Delta = PositionProfile(T_ - t_f_ + epsilon) -
            PositionProfile(T_ - t_f_ - epsilon);
    if (std::fabs(Delta) >= delta) {
      aon::logging::Error(
          "[❌] Value Error: Discontinuity found in Position Profile at `t_0 + "
          "t_v`\n");
    }
  }

  /**
   * \brief Make sure integral of `SpeedProfile` matches with `PositionProfile`.
   *
   * \details Using Trapezoidal Rule to estimate the integral of the
   *     SpeedProfile. At each integration step we are comparing the
   *     approximation the SpeedProfile's integral with PositionProfile.
   *
   */
  void TestIntegral() {
    // Maximum acceptable absolute error.
    const long double epsilon = FLT_EPSILON;
    const long double dx = 1.0 / (1 << 15);  // Step size = 2 ^ -15

    long double sum = 0.0;
    long double integral, value;

    for (long double t = dx; t < T_; t += dx) {
      sum += SpeedProfile(t - dx) + SpeedProfile(t);
      integral = dx * sum / 2.0;

      value = PositionProfile(t);
      if (std::fabs(integral - value) >= epsilon) {
        aon::logging::Error(
            "[❌] Value Error: The Position Profile does not match the integral "
            "of the Speed Profile at `t = " +
            std::to_string(t) +
            "`. Position Profile = " + std::to_string(value) +
            ", Integral = " + std::to_string(integral) + "\n");
      }
    }
  }

  /**
   * \brief Integral of the Speed Profile
   *
   * \details Calculates the ideal position at time t. It is used later to
   *     help introduce feedback to the control algorithm.
   *
   * \param t Current time with respect to start of motion [seconds]
   *
   * \returns double Output position [inches]
   */
  double PositionProfile(double t) {
    double output = 0.0;

    if (0.0 <= t && t <= T_) {
      const int profile = (t <= t_0_) ? 0 : (t <= t_0_ + t_v_) ? 1 : 2;

      // Use horner's nested evaluation.
      for (int i = 2; i >= 0; i--)
        output = output * t + position_coeffs_[profile][i];

    } else if (t < 0.0) {
      output = PositionProfile(t + T_) - D_;

    } else if (T_ < t) {
      output = PositionProfile(t - T_) + D_;
    }
    return output;
  }

 public:
  /**
   * \brief Performs tests on parameters.
   *
   * \details Should mostly be run when debugging and/or verifying parameters,
   *     so time and space complexity are terrible here. It is important that
   *     each motion profile has its own constraints. This function helps to
   *     verify that they are met. Example of constraints are:
   *     - Profile continuity
   *     - Integral of `SpeedProfile` is `PositionProfile`
   *     - Others
   *
   * \see TrapezoidProfile::TestBasic()
   * \see TrapezoidProfile::TestContinuity()
   * \see TrapezoidProfile::TestIntegral()
   */
  void RunTests() {
    TestBasic();
    TestContinuity();
    TestIntegral();
  }

  /**
   * \brief Helper function to quickly set and start processing parameters.
   *
   * \param D Distante to travel [inches]
   * \param V Maximum speed throughout motion [inches / s]
   * \param v_0 Motion's initial speed [inches / s]
   * \param v_f Motion's final speed [inches / s]
   * \param a_0 Motion's initial acceleration [inches / s^2]
   * \param a_f Motion's final acceleration [inches / s^2]
   * \param correction_ratio Establishes how much to compensate for overshoots.
   */
  void SetParams(double D, double V, double v_0, double v_f, double a_0,
                 double a_f, double correction_ratio) {
    D_ = D;
    V_ = V;
    a_0_ = a_0;
    a_f_ = a_f;
    v_0_ = v_0;
    v_f_ = v_f;
    correction_ratio_ = correction_ratio;

    try {
      ProcessParams();
    } catch (std::domain_error& e) {
      // If we can't use our custom values due to errors, force the entire
      // motion to just be constant velocity (stage 2);
      v_0_ = v_f_ = V;
      a_0_ = a_f_ = 1.0;
      correction_ratio_ = 0.0;

      ProcessParams();
    }
  }

  /// When empty, all variables will be 0.
  TrapezoidProfile() {
    D_ = 0.0;
    V_ = 0.0;
    T_ = 0.0;
    a_0_ = 0.0;
    a_f_ = 0.0;
    v_0_ = 0.0;
    v_f_ = 0.0;
    t_0_ = 0.0;
    t_v_ = 0.0;
    t_f_ = 0.0;
    correction_ratio_ = 0.0;
  }

  /**
   * \brief With only 3 parameters, assume a_0_ = a_f_ and v_0_ = v_f_ = 0.
   *
   * \param D Total travel distance [inches]
   * \param V Maximum travel speed [inches / second]
   * \param A Acceleration for stage 1 and 3 [inches / second^2]
   *
   * \see TrapezoidProfile::SetParams
   *
   */
  TrapezoidProfile(double D, double V, double A) {
    SetParams(D, V, 0.0, 0.0, A, A, 0.0);
  }

  /**
   * \brief When 4 parameters are assigned, prioritize a_0 and a_f.
   *
   * \param D Total travel distance [inches]
   * \param V Maximum travel speed [inches / second]
   * \param a0 Acceleration for stage 1 [inches / second^2]
   * \param af Acceleration for stage 3 [inches / second^2]
   *
   * \see TrapezoidProfile::SetParams
   */
  TrapezoidProfile(double D, double V, double a0, double af) {
    SetParams(D, V, 0.0, 0.0, a0, af, 0.0);
  }

  /// Retrieve the travel distance. [inches]
  double GetD() { return D_; }
  /// Retrieve the maximum speed. [inches / second]
  double GetV() { return V_; }
  /// Retrieve the estimated motion time. [seconds]
  double GetT() { return T_; }

  /// Retrieve the desired initial acceleration. [inches / second^2]
  double GetA0() { return a_0_; }
  /// Retrieve the desired final acceleration. [inches / second^2]
  double GetAf() { return a_f_; }
  /// Retrieve the desired initial speed. [inches / second]
  double GetV0() { return v_0_; }
  /// Retrieve the desired final speed. [inches / second]
  double GetVf() { return v_f_; }

  /// Retrieve duration of the first period of travel. (stage 1)
  double GetT0() { return t_0_; }
  /// Retrieve duration of the second period of travel. (stage 2)
  double GetTv() { return t_v_; }
  /// Retrieve duration of the third period of travel. (stage 3)
  double GetTf() { return t_f_; }

  /**
   * \brief Creative Pythonic way to access parameters.
   *
   * \param parameter Variable we want to extract. Can be one of the
   * following: [D, V, T, a_0, a_f, v_0, v_f, t_0, t_v, t_f]
   *
   * \return double Value of the requested parameter. If not valid, returns
   * NAN.
   */
  double operator[](std::string parameter) {
    if (parameter == "D")
      return GetD();
    else if (parameter == "V")
      return GetV();
    else if (parameter == "T")
      return GetT();
    else if (parameter == "a_0")
      return GetA0();
    else if (parameter == "a_f")
      return GetAf();
    else if (parameter == "v_0")
      return GetV0();
    else if (parameter == "v_f")
      return GetVf();
    else if (parameter == "t_0")
      return GetT0();
    else if (parameter == "t_v")
      return GetTv();
    else if (parameter == "t_f")
      return GetTf();

    aon::logging::Error("[❌] Invalid Argument: Parameter `" + parameter +
                        "` is not part of the Motion Profile.\n");
    return NAN;
  }

  /**
   * \brief Calculate desired speed for this time instance.
   *
   * \details This is the fundamental function for this class. It uses
  formulas
   *     in order to calculate the corresponding speed value for the current
   *     time. The speed vs time plot ressembles a trapezoid, hence this
  class'
   *     name.
   *
   * \param t Current time with respect to start of motion [seconds]
   *
   * \returns double Optimal output speed [inches / second]
   *
   * \code{.cpp}

  int main(){
  aon::TrapezoidProfile profile = aon::TrapezoidProfile(4, 2, 2, 1);

  // D_ = 4, V_ = 2, T_ = 3.5
  std::cout + "D = " + profile.GetD() + ", V = " + profile.GetV()
    + ", T = " + profile.GetT() + std::endl;

  for(double t = 0; t < profile.GetT(); t += profile.GetT() / 10.0)
    std::cout + t + ", " + profile.SpeedProfile(t) + std::endl;
  // Outputs:
  // 0, 0
  // 0.35, 0.7
  // 0.7, 1.4
  // 1.05, 2
  // 1.4, 2
  // 1.75, 1.75
  // 2.1, 1.4
  // 2.45, 1.05
  // 2.8, 0.7
  // 3.15, 0.35
  }

   * \endcode
   */
  double SpeedProfile(double t) {
    double output = 0;

    if (0.0 <= t && t <= T_) {  // If t is within the motion range
      const int stage = (t <= t_0_) ? 0 : (t <= t_0_ + t_v_) ? 1 : 2;
      output = speed_coeffs_[stage][1] * t + speed_coeffs_[stage][0];

    } else if (t < 0.0) {  // In case it undershoots
      output = correction_ratio_ * SpeedProfile(-t);

    } else if (T_ < t) {  // In case it overshoots
      output = -correction_ratio_ * SpeedProfile(2.0 * T_ - t);
    }
    return output;
  }

  /**
   * \brief Shorthand to call TrapezoidProfile::SpeedProfile easily
   *
   * \param t Current time with respect to start of motion [seconds]
   *
   * \returns double Optimal output speed for current time t [inches / second]
   *
   * \see TrapezoidProfile::SpeedProfile
   *
   * \code{.cpp}
  int main(){
  aon::TrapezoidProfile profile = aon::TrapezoidProfile();

  // Equivalent to calling `profile.SpeedProfile(0);`
  double velocity = profile(0);

  return 0;
  }
   * \endcode
   */
  double operator()(double t) { return SpeedProfile(t); }

  /**
   * \brief Find `t` in `s(t) = d` using the modified false-position method.
   *
   * \details The false position method can be used to find the roots of a
   *     function. Given that `s(t) = d => s(t) - d = 0`, we can find the
  value
   *     of `t` that produces the value `d`, effectively calculating the
  inverse
   *     function numerically.
   *
   * \param d The distance relative to starting point.  [inches]
   *
   * \returns Corresponding time interval [seconds] for the current position.
   *
   * \code{.cpp}

  int main(){
  aon::TrapezoidProfile profile = aon::TrapezoidProfile(4, 2, 2);

  // D_ = 4, V_ = 2, T_ = 3
  std::cout + "D_ = " + profile.GetD() + ", V_ = " + profile.GetV()
    + ", T_ = " + profile.GetT() + std::endl;

  for(double d = 0; d < profile.GetD(); d += profile.GetD() / 10.0)
    std::cout + d + ", " + profile.PositionProfileInverse(d) +
  std::endl;
  // Outputs:
  // 0, 0
  // 0.4, 0.632456
  // 0.8, 2.55278
  // 1.2, 3
  // 1.6, 3
  // 2, 3
  // 2.4, 3
  // 2.8, 3
  // 3.2, 3
  // 3.6, 3
  // 4, 3
  }

   * \endcode
   *
   * \warning Make sure that `d` has the correct sign!! If the paramter D is
   *    negative, `d` MUST be negative.
   *
   */
  double PositionProfileInverse(double d) {
    const double max_iter = 15;
    // Approximate errors below this are more than good enough
    const double min_error = 1.0 / 10.0;
    double approximate_error = 100.0;
    int iter = 0;

    double x_lower = -T_, x_upper = 2.0 * T_;
    double f_lower = PositionProfile(x_lower) - d;
    double f_upper = PositionProfile(x_upper) - d;
    double x_root = T_, x_root_old = 0.0;
    double f_root, test;

    // Keys to optimize the normal false-position method, hence the
    // "modified false-position method".
    int iter_lower = 0, iter_upper = 0;

    for (iter = 0; iter < max_iter && min_error <= approximate_error; iter++) {
      x_root_old = x_root;

      // This is the essence of the normal false-position method.
      x_root = -f_upper * (x_lower - x_upper) / (f_lower - f_upper) + x_upper;
      f_root = PositionProfile(x_root) - d;
      test = f_lower * f_root;

      if (x_root != 0.0)
        approximate_error = std::fabs((x_root - x_root_old) / x_root) * 100.0;
      else
        approximate_error = x_root_old * 100.0;

      if (test < 0.0) {
        x_upper = x_root;
        f_upper = f_root;
        iter_upper = 0;
        if (++iter_lower >= 2) f_lower /= 2.0;

      } else if (test > 0.0) {
        x_lower = x_root;
        f_lower = f_root;
        iter_lower = 0;
        if (++iter_upper >= 2) f_upper /= 2.0;

      } else {
        approximate_error = 0.0;
      }
    }

    return x_root;
  }
};
}  // namespace aon

#endif  // AON_CONTROLS_TRAPEZOID_PROFILE_TRAPEZOID_HPP_
