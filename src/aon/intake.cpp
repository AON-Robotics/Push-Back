#include "../include/aon/intake/intake.hpp"

namespace aon {

Intake::Intake(const std::initializer_list<okapi::Motor> &allPorts,
               const std::initializer_list<okapi::Motor> &railPorts,
               const std::initializer_list<okapi::Motor> &gatePorts,
               int distanceSensorPort)
    : _intake(allPorts),
      _rail(railPorts),
      _gate(gatePorts),
      distanceSensor(distanceSensorPort) {}

void Intake::move(const int &rpm) { _intake.moveVelocity(rpm); }

void Intake::rail(const int &rpm) { _rail.moveVelocity(rpm); }

void Intake::gate(const int &rpm) { _gate.moveVelocity(rpm); }

void Intake::stop() { _intake.moveVelocity(0); }

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= DISTANCE; }

void Intake::scan() {
  while (true) {
    if (scanning && this->isObjectDetected()) {
      this->pickUp();
    }
    pros::delay(20);
  }
}

void Intake::activateScan() { scanning = true; }

void Intake::stopScan() { scanning = false; }

void Intake::openGate(const int &delay) {
  this->gate(-100);
  pros::delay(delay);
  this->gate(0);
}

void Intake::pickUp(const int &delay) {
  this->move(INTAKE_VELOCITY / 0.8);  // run a bit faster than our default
  pros::delay(delay);
  this->stop();  // stop after the delay
}

void Intake::score(const int &delay) {
  this->rail(INTAKE_VELOCITY);
  pros::delay(delay);
  this->rail(0);
}

void Intake::discard() {
  this->move(-INTAKE_VELOCITY);
  pros::delay(1000);
  this->stop();
}

void Intake::kickBackRail(){
  this->rail(-100);
  pros::delay(150);
  this->rail(0);

}

}  // namespace aon
