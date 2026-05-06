#pragma once

#include "./drivetrain.hpp"

namespace aon {

class DifferentialDrive : public Drivetrain {
 private:
  SmartMotorGroup leftMotors;
  SmartMotorGroup rightMotors;

 public:
  DifferentialDrive(const std::initializer_list<okapi::Motor> &leftPorts = {0},
            const std::initializer_list<okapi::Motor> &rightPorts = {0},
            Pose pose = Pose(),
            std::unique_ptr<Odometry> odometry = nullptr,
            SpeedFactors speedFactors = SpeedFactors(),
            std::unique_ptr<MotionProfile> yProfile = nullptr,
            std::unique_ptr<MotionProfile> thetaProfile = nullptr
          )
      : leftMotors(leftPorts, 0, MAX_ACCEL),
        rightMotors(rightPorts, 0, MAX_ACCEL),
        Drivetrain(pose, std::move(odometry), speedFactors, nullptr, std::move(yProfile), std::move(thetaProfile)) {}

  /// @brief Drives the robot using tank control, mapping left and right inputs directly to each side of the drivetrain
  /// @param left The \b RPM to send to the left-side motors (positive is forward)
  /// @param right The \b RPM to send to the right-side motors (positive is forward)
  void tank(const double &left, const double &right) override;

  /// @brief Sets the brake mode for all motors of the drivetrain
  /// @param brakeMode The new brake mode for the drivetrain
  void setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode) override;

  /// @brief Sets the gearset for all motors of the drivetrain
  /// @param gearset The new gearset for the drivetrain
  void setGearset(okapi::AbstractMotor::gearset gearset) override;

  /// @brief Sets the units for all encoders of the motors of the drivetrain
  /// @param units The new units for the drivetrain
  void setEncoderUnits(okapi::AbstractMotor::encoderUnits units) override;

  /// @brief Sets the slew rate for all motors of the drivetrain
  /// @param slew The new slew rate for the drivetrain
  void setSlewRate(double slew) override;

  /// @brief Calculates average RPM forward
  /// @return The RPM of the motors with respect to the front of the robot
  double getRPM() override;

  /// @brief Goes to the target point
  /// @param pose The target pose
  /// @note Uses coordinate system from GPS in \b meters
  void goToPose(const Pose &pose) override; // TODO: add optional `settle` boolean

};

}  // namespace aon
