#ifndef AON_CONTROLS_SMART_MOTOR_HPP__
#define AON_CONTROLS_SMART_MOTOR_HPP__

#include "../../okapi/api.hpp"

/**
 * SMART_MOTOR
 *
 * Inherits from okapi Motor class and implements moveVelocity
 * and moveVoltage methods to apply a slew rate and in turn
 * protect our motors from overheating or being damaged over
 * time from drastic changes in current.
 *
 */

namespace aon {

class SmartMotor : public okapi::Motor {
 private:
  double current_voltage;
  // Using double because int conversions create loss of data.
  double current_velocity;
  int d_voltage;
  int acceleration;
  uint64_t now = 0;
  uint64_t previous_time = 0;
  int step = 0;

 public:
  explicit SmartMotor(std::int8_t iport, int voltage_delta = 0,
                      int velocity_delta = 0)
      : okapi::Motor(iport) {
    current_voltage = 0;
    current_velocity = 0;

    SetDVoltage(voltage_delta);
    // Acceleration has units rpm^2/min^2
    // By default there will be no effect on the Slew Rate Controller
    SetAcceleration(velocity_delta);

    now = previous_time = pros::micros();
  }

  /// Getters
  double GetCurrentVoltage() { return current_voltage; }
  double GetCurrentVelocity() { return current_velocity; }
  int GetDVoltage() { return d_voltage; }
  int GetAcceleration() { return acceleration; }
  uint64_t GetNow() { return now; }

  /// Setters
  void SetCurrentVoltage(double current_vol) { current_voltage = current_vol; }
  void SetCurrentVelocity(double current_vel) {
    current_velocity = current_vel;
  }
  void SetDVoltage(int delta_voltage) { d_voltage = std::abs(delta_voltage); }
  void SetAcceleration(int delta_velocity) {
    acceleration = std::abs(delta_velocity);
  }

  /**
   * \brief Move motor velocity according to slew rate calculations.
   *
   * \param ivelocity Target velocity [rpm]
   *
   * \returns moveVelocity output
   */
  std::int32_t moveVelocity(std::int16_t ivelocity) {
    if (GetAcceleration() != 0) {
      now = pros::micros();
      const double speed_difference = ivelocity - GetCurrentVelocity();

      if (std::abs(speed_difference) > 0) {
        const double sign = (speed_difference > 0) ? 1 : -1;
        // Kinematics equation v_f = v_o + a * Δt
        SetCurrentVelocity(GetCurrentVelocity() +
                           GetAcceleration() * (GetNow() - previous_time) *
                               sign / 1E6);
        ivelocity = std::ceil(GetCurrentVelocity());
      }
      previous_time = pros::micros();
    }
    return okapi::Motor::moveVelocity(ivelocity);
  }

  /**
   * \brief Move motor voltage according to slew rate calculations.
   *
   * \param ivelocity Target velocity [rpm]
   *
   * \returns moveVelocity output
   */
  std::int32_t moveVoltage(std::int16_t ivoltage) {
    if (GetDVoltage() != 0) {
      now = pros::micros();
      const double power_difference = ivoltage - GetCurrentVoltage();

      if (std::abs(power_difference) > 0) {
        const double sign = (power_difference > 0) ? 1 : -1;
        // Kinematics equation v_f = v_o + a * Δt
        SetCurrentVelocity(GetCurrentVoltage() +
                           GetDVoltage() * (GetNow() - previous_time) * sign /
                               1E6);
        ivoltage = std::ceil(GetCurrentVoltage());
      }
      previous_time = pros::micros();
    }
    return okapi::Motor::moveVoltage(ivoltage);
  }
};

/**
 * SMART_MOTOR_GROUP
 *
 * Inherits from okapi MotorGroup class and implements moveVelocity
 * and moveVoltage methods to apply a slew rate and in turn
 * protect our motors from overheating or being damaged over
 * time from drastic changes in current.
 *
 */

class SmartMotorGroup : public okapi::MotorGroup {
 private:
  double current_voltage;
  double current_velocity;
  int d_voltage;
  int acceleration;
  uint64_t now = 0;
  uint64_t previous_time = 0;
  int step = 0;

 public:
  SmartMotorGroup(const std::initializer_list<okapi::Motor> &imotors,
                  int voltage_delta = 0, int velocity_delta = 0)
      : okapi::MotorGroup(imotors) {
    current_voltage = 0;
    current_velocity = 0;

    SetDVoltage(voltage_delta);
    // Acceleration has units rpm/min^2
    // By default there will be no effect on the Slew Rate Controller
    SetAcceleration(velocity_delta);

    now = previous_time = pros::micros();
  }

  /// Getters
  double GetCurrentVoltage() { return current_voltage; }
  double GetCurrentVelocity() { return current_velocity; }
  int GetDVoltage() { return d_voltage; }
  int GetAcceleration() { return acceleration; }
  uint64_t GetNow() { return now; }

  /// Setters
  void SetCurrentVoltage(double current_vol) { current_voltage = current_vol; }
  void SetCurrentVelocity(double current_vel) {
    current_velocity = current_vel;
  }
  void SetDVoltage(int delta_voltage) { d_voltage = std::abs(delta_voltage); }
  void SetAcceleration(int delta_velocity) {
    acceleration = std::abs(delta_velocity);
  }

  /// Move motor velocity according to slew rate calculations.
  /// unit: rev/min^2
  /// \param ivelocity
  std::int32_t moveVelocity(std::int16_t ivelocity) {
    if (GetAcceleration() != 0) {
      now = pros::micros();
      const double speed_difference = ivelocity - GetCurrentVelocity();

      if (std::abs(speed_difference) > 0) {
        const double sign = (speed_difference > 0) ? 1 : -1;
        // Kinematics equation v_f = v_o + a * Δt
        SetCurrentVelocity(GetCurrentVelocity() +
                           GetAcceleration() * (GetNow() - previous_time) *
                               sign / 1E6);
        ivelocity = std::ceil(GetCurrentVelocity());
      }
      previous_time = pros::micros();
    }
    return okapi::MotorGroup::moveVelocity(ivelocity);
  }

  /// Move motor voltage according to slew rate calculations.
  /// unit: mV/s
  /// \param ivoltage
  std::int32_t moveVoltage(std::int16_t ivoltage) {
    if (GetDVoltage() != 0) {
      now = pros::micros();
      const double power_difference = ivoltage - GetCurrentVoltage();

      if (std::abs(power_difference) > 0) {
        const double sign = (power_difference > 0) ? 1 : -1;
        // Kinematics equation v_f = v_o + a * Δt
        SetCurrentVoltage(GetCurrentVoltage() + GetDVoltage() *
                                                    (GetNow() - previous_time) *
                                                    sign / 1E6);
        ivoltage = std::ceil(GetCurrentVoltage());
      }
      previous_time = pros::micros();
    }
    return okapi::MotorGroup::moveVoltage(ivoltage);
  }
};

}  // namespace aon

#endif  // AON_CONTROLS_SMART_MOTOR_HPP__
