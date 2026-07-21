#pragma once

#include "pros/motors.h"

namespace aon::lemlib_integration {

struct DriveCommand {
  std::int8_t left = 0;
  std::int8_t right = 0;
};

struct DriveSensorSample {
  double leftMotorDegrees = 0.0;
  double rightMotorDegrees = 0.0;
  double leftTrackingInches = 0.0;
  double rightTrackingInches = 0.0;
  double backTrackingInches = 0.0;
  double imuDegrees = 0.0;
  bool leftMotorValid = false;
  bool rightMotorValid = false;
  bool leftTrackingValid = false;
  bool rightTrackingValid = false;
  bool backTrackingValid = false;
  bool imuValid = false;
};

DriveSensorSample sampleDriveSensors();
void commandTank(int left, int right);
DriveCommand effectiveDriveCommand();
void stopDrive();
void setDriveBrakeMode(pros::motor_brake_mode_e brakeMode);

}  // namespace aon::lemlib_integration
