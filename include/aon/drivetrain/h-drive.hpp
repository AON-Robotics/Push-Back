#pragma once

#include "./drivetrain.hpp"

namespace aon {

class HDrive : public Drivetrain {
 private:
  SmartMotorGroup leftMotors;
  SmartMotorGroup rightMotors;
  SmartMotorGroup midMotors;

 public:
  HDrive(const std::initializer_list<okapi::Motor> &leftPorts = {0},
         const std::initializer_list<okapi::Motor> &rightPorts = {0},
         const std::initializer_list<okapi::Motor> &midPorts = {0},
         Pose pose = Pose(),
         std::unique_ptr<Odometry> odometry = nullptr,
         SpeedFactors speedFactors = SpeedFactors(),
         std::unique_ptr<MotionProfile> xProfile = nullptr,
         std::unique_ptr<MotionProfile> yProfile = nullptr,
         std::unique_ptr<MotionProfile> thetaProfile = nullptr
      )
      : leftMotors(leftPorts),
        rightMotors(rightPorts),
        midMotors(midPorts),
        Drivetrain(pose, std::move(odometry), speedFactors, std::move(xProfile), std::move(yProfile), std::move(thetaProfile)) {}

  /// @brief Configures the general settings for the motors
  /// @param brakeMode The braking paradigm we will use, usually `holding` for
  /// auton and `brake` for drivers
  /// @param gearset The gearbox the physical motors contain, they MUST be all
  /// the same
  void configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset, double slew) {
    Drivetrain::configure(brakeMode, gearset, slew);
    midMotors.setBrakeMode(brakeMode);
    midMotors.setGearing(okapi::AbstractMotor::gearset::red);
    midMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    midMotors.tarePosition();
  }

  void stop() override;

  /// @brief Moves all motors the same `rpm` to move sideways
  /// @param rpm The speed in which to move all motors in \b rpm (positive is right and negative is left)
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  void sideways(const double &rpm = MAX_RPM, const int& delay = 0) override;

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
  /// @param target The intended destination using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @note Uses coordinate system from GPS in \b meters
  void goToPose(const Pose& target) override;

  /// @brief Follows a path using a pure pursuit controller
  /// @param path The path to follow
  /// @note The `path`s intermediate headings are ignored, only the final one is actually aligned
  void follow(const std::vector<Pose>& path) override;
};

}  // namespace aon
