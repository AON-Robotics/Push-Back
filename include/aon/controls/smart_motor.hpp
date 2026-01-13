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

  /// @brief Move motor velocity according to slew rate calculations. Slew is in \b rev/min^2
  /// @param ivelocity – The new motor velocity in \b rpm from -+-100, +-200, or +-600 depending on the motor's gearset
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t moveVelocity(std::int16_t ivelocity) {
    if (GetAcceleration() != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (GetNow() - previous_time) / 1E6;
      SetCurrentVelocity(std::clamp((double)ivelocity, GetCurrentVelocity() - GetAcceleration() * deltaTime,  GetCurrentVelocity() + GetAcceleration() * deltaTime));
      ivelocity = std::ceil(GetCurrentVelocity());

      previous_time = pros::micros();
    }
    return okapi::Motor::moveVelocity(ivelocity);
  }

  /// @brief Move motor voltage according to slew rate calculations. Slew is in \b mV/s
  /// @param ivoltage – ivoltage – The new voltage value from -12000 to 12000.
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t moveVoltage(std::int16_t ivoltage) {
    if (GetDVoltage() != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (GetNow() - previous_time) / 1E6;
      SetCurrentVoltage(std::clamp((double)ivoltage, GetCurrentVoltage() - GetDVoltage() * deltaTime, GetCurrentVoltage() + GetDVoltage() * deltaTime));
      ivoltage = std::ceil(GetCurrentVoltage());

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
  double acceleration;
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
  double GetAcceleration() { return acceleration; }
  uint64_t GetNow() { return now; }

  /// Setters
  void SetCurrentVoltage(double current_vol) { current_voltage = current_vol; }
  void SetCurrentVelocity(double current_vel) {
    current_velocity = current_vel;
  }
  void SetDVoltage(int delta_voltage) { d_voltage = std::abs(delta_voltage); }
  void SetAcceleration(double delta_velocity) {
    acceleration = std::fabs(delta_velocity);
  }

  /// @brief Move motor velocity according to slew rate calculations. Slew is in \b rev/min^2
  /// @param ivelocity – The new motor velocity in \b rpm from -+-100, +-200, or +-600 depending on the motor's gearset
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t moveVelocity(std::int16_t ivelocity) {
    if (this->GetAcceleration() != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (this->GetNow() - previous_time) / 1E6;
      const double upperLimit = this->GetCurrentVelocity() + this->GetAcceleration() * deltaTime;
      const double lowerLimit = this->GetCurrentVelocity() - this->GetAcceleration() * deltaTime;
      this->SetCurrentVelocity(std::clamp((double)ivelocity, lowerLimit,  upperLimit));
      ivelocity = std::ceil(this->GetCurrentVelocity());

      previous_time = pros::micros();
    }
    return okapi::MotorGroup::moveVelocity(ivelocity);
  }

  /// @brief Move motor voltage according to slew rate calculations. Slew is in \b mV/s
  /// @param ivoltage – ivoltage – The new voltage value from -12000 to 12000.
  /// @returns 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
  std::int32_t moveVoltage(std::int16_t ivoltage) {
    if (GetDVoltage() != 0) {
      now = pros::micros();

      // Kinematics equation v_f = v_o + a * Δt
      const double deltaTime = (GetNow() - previous_time) / 1E6;
      SetCurrentVoltage(std::clamp((double)ivoltage, GetCurrentVoltage() - GetDVoltage() * deltaTime, GetCurrentVoltage() + GetDVoltage() * deltaTime));
      ivoltage = std::ceil(GetCurrentVoltage());

      previous_time = pros::micros();
    }
    return okapi::MotorGroup::moveVoltage(ivoltage);
  }
};

}  // namespace aon

#endif  // AON_CONTROLS_SMART_MOTOR_HPP__
