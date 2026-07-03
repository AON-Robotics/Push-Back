#include "aon/core/robot.hpp"

#include "main.hpp"

#include "aon/lemlib/chassis.hpp"

namespace aon::core {

void Robot::initialize() {
#if LEMLIB_SENSOR_TEST
  aon::logging::Initialize();
  aon::lemlib_integration::startSensorTest();
#else
  pros::Task guiLoopTask([] { aon::gui->initialize(); });
  aon::logging::Initialize();
  aon::Configure(false);
  pros::Task odomTask([] { drivetrain.initialize(); });
  pros::delay(3000);
  pros::Task safetyTask(aon::autonSafety);
  pros::Task intakeScanning([] { intake.scan(); });
  pros::Task intakeSorting([] { intake.sort(); });
#endif
}

void Robot::disabled() {}

void Robot::competitionInitialize() {}

void Robot::autonomous() {
#if LEMLIB_TRACKING_CALIBRATION_TEST
  aon::lemlib_integration::runTrackingCalibrationTest();
#elif LEMLIB_TURN_TEST
  aon::lemlib_integration::runTurnTest(LEMLIB_TURN_TEST_ANGLE);
#elif !LEMLIB_SENSOR_TEST
  aon::Configure(false);
  aon::autonomousReader->ExecuteFunction("autonomous");
  pros::delay(10);
#endif
}

void Robot::opcontrol() {
#if LEMLIB_SENSOR_TEST || LEMLIB_TURN_TEST || LEMLIB_TRACKING_CALIBRATION_TEST
  while (true) pros::delay(50);
#else
  aon::Configure();
  while (true) {
#if TESTING_AUTONOMOUS
    aon::Configure(false);
    aon::autonomousReader->ExecuteFunction("autonomous");
    pros::delay(5000);
#else
    aon::operator_control::Run(driver);
#endif
    pros::delay(10);
  }
#endif
}

}  // namespace aon::core
