#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"
#include <atomic>

// Intake system managing intake motors, rail, and gate mechanisms.

namespace aon {
class Intake {
  okapi::MotorGroup intake;
  okapi::Motor rail;
  okapi::Motor gate;
  pros::Distance distanceSensor;
  
  volatile bool objectDetected = false;
  volatile bool scanning = false;
  public:
   Intake(const std::initializer_list<okapi::Motor> &allPorts, int railPort, int gatePort, int distanceSensorPort);

  // High Stakes Control and Setup Methods:
  void startScan();
  void stopScan();
  void pickUp(int delayMs = 1000);
  void score(int delayMs = 1500);
  void discard();
  void openGate(int ms = 250);
  void scan();
  void move(int velocity);
  void moveRail(int velocity);
  void moveGate(int velocity);
  double getDistance();
  bool isObjectDetected();
};
}  // namespace aon
