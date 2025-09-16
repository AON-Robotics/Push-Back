#pragma once
#include "../api.h"
#include <atomic>
#include "../okapi/api.hpp"

// Intake system managing intake motors, rail, and gate mechanisms.

namespace aon{
class Intake {  
public:

  Intake();
  okapi::MotorGroup& intake;
  okapi::Motor&      rail;
  okapi::Motor&      gate;
  volatile bool intakeScanning = false;
  //Singleton Access
  static Intake& instance();
  //High Stakes Control and Setup Methods:
  void startScan();
  void stopScan();
  void pickUp(int delayMs = 1000);
  void score(int delayMs = 1500);
  void discard();
  void openGate(int ms = 250);
  void scan();
  void setBrakeMode();
  void setGearing();
  void setEncoderUnits();
  void tarePosition();
  void move(int velocity);
};
}

