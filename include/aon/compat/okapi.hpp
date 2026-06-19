#pragma once

#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

// Temporary source-compatibility types for the narrow Okapi API surface used
// by AON. All behavior is implemented with native PROS 4 devices.
namespace okapi {

struct AbstractMotor {
  enum class brakeMode { coast, brake, hold };
  enum class gearset { red, green, blue };
  enum class encoderUnits { degrees, rotations, counts };
};

inline pros::MotorBrake toPros(AbstractMotor::brakeMode mode) {
  switch (mode) {
    case AbstractMotor::brakeMode::hold: return pros::MotorBrake::hold;
    case AbstractMotor::brakeMode::brake: return pros::MotorBrake::brake;
    default: return pros::MotorBrake::coast;
  }
}

inline pros::MotorGears toPros(AbstractMotor::gearset gears) {
  switch (gears) {
    case AbstractMotor::gearset::red: return pros::MotorGears::red;
    case AbstractMotor::gearset::green: return pros::MotorGears::green;
    default: return pros::MotorGears::blue;
  }
}

inline pros::MotorUnits toPros(AbstractMotor::encoderUnits units) {
  switch (units) {
    case AbstractMotor::encoderUnits::rotations: return pros::MotorUnits::rotations;
    case AbstractMotor::encoderUnits::counts: return pros::MotorUnits::counts;
    default: return pros::MotorUnits::degrees;
  }
}

class Motor {
 public:
  Motor(std::int8_t port) : port_(port), motor_(port) {}

  std::int8_t port() const { return port_; }
  std::int32_t moveVelocity(std::int16_t velocity) {
    return motor_.move_velocity(velocity);
  }
  std::int32_t moveVoltage(std::int16_t voltage) {
    return motor_.move_voltage(voltage);
  }
  std::int32_t setBrakeMode(AbstractMotor::brakeMode mode) {
    return motor_.set_brake_mode(toPros(mode));
  }
  std::int32_t setGearing(AbstractMotor::gearset gears) {
    return motor_.set_gearing(toPros(gears));
  }
  std::int32_t setEncoderUnits(AbstractMotor::encoderUnits units) {
    return motor_.set_encoder_units(toPros(units));
  }
  std::int32_t tarePosition() { return motor_.tare_position(); }
  double getActualVelocity() const { return motor_.get_actual_velocity(); }
  double getPosition() const { return motor_.get_position(); }

 private:
  std::int8_t port_;
  pros::Motor motor_;
};

class MotorGroup {
 public:
  explicit MotorGroup(const std::initializer_list<Motor>& motors)
      : motors_(portsFrom(motors)) {}

  std::int32_t moveVelocity(std::int16_t velocity) {
    return motors_.move_velocity(velocity);
  }
  std::int32_t moveVoltage(std::int16_t voltage) {
    return motors_.move_voltage(voltage);
  }
  std::int32_t moveAbsolute(double position, std::int32_t velocity) {
    return motors_.move_absolute(position, velocity);
  }
  std::int32_t setBrakeMode(AbstractMotor::brakeMode mode) {
    return motors_.set_brake_mode_all(toPros(mode));
  }
  std::int32_t setGearing(AbstractMotor::gearset gears) {
    return motors_.set_gearing_all(toPros(gears));
  }
  std::int32_t setEncoderUnits(AbstractMotor::encoderUnits units) {
    return motors_.set_encoder_units_all(toPros(units));
  }
  std::int32_t tarePosition() { return motors_.tare_position_all(); }
  double getActualVelocity() const { return motors_.get_actual_velocity(); }
  double getPosition() const { return motors_.get_position(); }

 private:
  static std::vector<std::int8_t> portsFrom(
      const std::initializer_list<Motor>& motors) {
    std::vector<std::int8_t> ports;
    ports.reserve(motors.size());
    for (const Motor& motor : motors) ports.push_back(motor.port());
    return ports;
  }

  pros::MotorGroup motors_;
};

template <typename Input = double, typename Output = double>
class AsyncPositionController {
 public:
  explicit AsyncPositionController(const std::initializer_list<Motor>& motors)
      : motors_(motors) {}

  void setTarget(double target) {
    target_ = target;
    motors_.moveAbsolute(target, 200);
  }
  double getTarget() const { return target_; }
  double getError() const { return std::abs(target_ - motors_.getPosition()); }
  bool isSettled() const { return getError() < 10.0; }
  void tarePosition() {
    motors_.tarePosition();
    target_ = 0.0;
  }

 private:
  MotorGroup motors_;
  double target_ = 0.0;
};

class AsyncPosControllerBuilder {
 public:
  AsyncPosControllerBuilder& withMotor(const std::initializer_list<Motor>& motors) {
    ports_.clear();
    for (const Motor& motor : motors) ports_.push_back(motor.port());
    return *this;
  }

  std::shared_ptr<AsyncPositionController<double, double>> build() const {
    // The project only builds this controller from the small robot scorer
    // motor list. Preserve its configured reversal when reconstructing it.
    if (ports_.empty()) return nullptr;
    return buildFromPorts();
  }

 private:
  std::shared_ptr<AsyncPositionController<double, double>> buildFromPorts() const {
    if (ports_.size() == 1) {
      return std::make_shared<AsyncPositionController<double, double>>(
          std::initializer_list<Motor>{Motor(ports_[0])});
    }
    // No current robot configuration uses multiple lever motors.
    return nullptr;
  }

  std::vector<std::int8_t> ports_;
};

class EKFFilter {
 public:
  explicit EKFFilter(double processNoise = 0.0001,
                     double measurementNoise = 0.01)
      : processNoise_(processNoise), measurementNoise_(measurementNoise) {}

  double filter(double measurement) {
    if (!initialized_) {
      estimate_ = measurement;
      initialized_ = true;
      return estimate_;
    }
    covariance_ += processNoise_;
    const double gain = covariance_ / (covariance_ + measurementNoise_);
    estimate_ += gain * (measurement - estimate_);
    covariance_ *= 1.0 - gain;
    return estimate_;
  }

 private:
  double processNoise_;
  double measurementNoise_;
  double estimate_ = 0.0;
  double covariance_ = 1.0;
  bool initialized_ = false;
};

}  // namespace okapi
