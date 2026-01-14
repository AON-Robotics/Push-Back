#include "main.hpp"
#include "aon/globals.hpp"
#include "aon/handler.hpp"
#include "aon/EKF/EKFDebug.hpp"
#include "pros/llemu.hpp"

namespace {
void ekfTaskFn(void*) {
  while (true) {
    sensorFeeder.step(ekf);
    pros::delay(10);
  }
}
}  // namespace

void initialize() {
  aon::gui::Initialize();
  aon::logging::Initialize();
  sensorFeeder.applyEkfDefaults(ekf);
  sensorFeeder.reset();
  // sensorFeeder.initializeGpsSnapshot(ekf);
  pros::Task(ekfTaskFn, nullptr, TASK_PRIORITY_DEFAULT,
                     TASK_STACK_DEPTH_DEFAULT, "EKF");
}
void autonomous(){
  drivetrainTank.move(5);
  while (true) {
    aon::gui::DisplayDebugMenu4();
    pros::delay(50);
  }
}

void opcontrol() {

}
