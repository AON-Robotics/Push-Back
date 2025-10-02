#include "../include/aon/intake/intake.hpp"

namespace aon{

Intake::Intake(const std::initializer_list<okapi::Motor> &allPorts, const std::initializer_list<okapi::Motor> &railPort, const std::initializer_list<okapi::Motor> &gatePort, int distanceSensorPort)
  : intake(allPorts)
  , rail(railPort)
  , gate(gatePort)
  , distanceSensor(distanceSensorPort) {
}

void Intake::startScan() { 
  objectDetected = false; // TODO: test with phsyical system if this works as intended
  scanning = true;
}

void Intake::stopScan()  { scanning = false; }

void Intake::pickUp(const int &delay) {
  intake.moveVelocity(INTAKE_VELOCITY / 0.8); // run a bit faster than our default
  pros::delay(delay);
  intake.moveVelocity(0);                     // stop after the delay
}

void Intake::score(const int &delay) {
  rail.moveVelocity(INTAKE_VELOCITY);
  pros::delay(delay);
  rail.moveVelocity(0);
}

void Intake::discard() {
  intake.moveVelocity(-INTAKE_VELOCITY);
  pros::delay(1000);
  intake.moveVelocity(0);
}

void Intake::openGate(const int &delay) {
  gate.moveVelocity(-100);
  pros::delay(delay);
  gate.moveVelocity(0);
}

void Intake::move(const int &rpm){
  intake.moveVelocity(rpm);
}

void Intake::moveRail(const int &rpm){
  rail.moveVelocity(rpm);
}

void Intake::moveGate(const int &rpm){
  gate.moveVelocity(rpm);
}

void Intake::scan(){
  while(true){
    if (scanning && distanceSensor.get() <= DISTANCE) {
      objectDetected = true;
      pickUp();
      intake.moveVelocity(0);
    }
    pros::delay(20);
  }
}

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return objectDetected; }
}