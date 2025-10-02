#pragma once

#include "../constants.hpp"
#include "../../api.h"
#include "../../okapi/api.hpp"

// Intake system managing intake motors, rail, and gate mechanisms.

namespace aon {
class Intake {
  private:

  okapi::MotorGroup intake;
  okapi::MotorGroup rail;
  okapi::MotorGroup gate;
  pros::Distance distanceSensor;
  
  volatile bool objectDetected = false;
  volatile bool scanning = false;

  public:

  Intake(const std::initializer_list<okapi::Motor> &allPorts, const std::initializer_list<okapi::Motor> &railPort, const std::initializer_list<okapi::Motor> &gatePort, int distanceSensorPort);

  void startScan();
  
  void stopScan();

  /// @brief This small subroutine moves the intake such that a ring is scored on the mobile goal being carried
  /// @param delay The time in \b milliseconds to leave the intake running
  void pickUp(const int &delay = 1000);
  
  /// @brief This small subroutine moves the rail such that a ring is scored on the mobile goal being carried
  /// @param delay The time in \b milliseconds to leave the intake running
  void score(const int &delay = 1500);

  /// @brief Discards disk at beginning of match
  /// @note This function is really meant for routines that will focus on enemy rings
  void discard();
  
  /// @brief Drops the gate from starting position so the robot can grab stuff
  void openGate(const int &delay = 250);
  
  /// @brief Runs a background loop to auto-pick rings when scanning is active.
  /// For this function, the drivetrain logic can be accessed via a bool in autonomous so that
  /// So instead of using drivetrain logic here just use a bool to not have drivetrain logic here. 

  void scan();
  
  void move(const int &rpm);
  
  void moveRail(const int &rpm);
  
  void moveGate(const int &rpm);
  
  /// @brief Gets the distance from the distance sensor
  /// @return The distance from the sensor to whatever it is detecting
  double distance();
  
  bool isObjectDetected();
};
}  // namespace aon
