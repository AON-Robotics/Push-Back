#include "main.hpp"
#include "aon/pi_comm.hpp"

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "USB Serial Ready");
  aon::initUsbSerial();
  aon::sendToPi("VS_BOOT");
  aon::pollUsbSerial();  // flush startup message
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  aon::sendToPi("VS_READY");
  while (true) {
    aon::pollUsbSerial();
    pros::delay(5);
  }
}
