#include "../include/main.hpp"
#include "aon/lemlib/chassis.hpp"

void initialize() {
#if LEMLIB_SENSOR_TEST
  aon::logging::Initialize();
  aon::lemlib_integration::startSensorTest();
#else
  pros::Task guiLoopTask([]{aon::gui->initialize();});
  aon::logging::Initialize();
  aon::Configure(false);
  pros::Task odomTask([]{drivetrain.initialize();});
  pros::delay(3000);
  pros::Task safetyTask(aon::autonSafety);
  // pros::Task turretFollowTask([]{orbit.follow();});
  // pros::Task turretScanTask([]{orbit.scan();}); // TODO: combine this with the follow task
  pros::Task intakeScanning([]{intake.scan();});
  pros::Task intakeSorting([]{intake.sort();});
#endif
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
#if LEMLIB_TRACKING_CALIBRATION_TEST
  aon::lemlib_integration::runTrackingCalibrationTest();
#elif LEMLIB_TURN_TEST
  aon::lemlib_integration::runTurnTest(LEMLIB_TURN_TEST_ANGLE);
#elif !LEMLIB_SENSOR_TEST
  aon::Configure(false); // Set drivetrain to hold for auton
  // TODO: add presetFunction
  aon::autonomousReader->ExecuteFunction("autonomous");
  pros::delay(10);
#endif
}

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components
void opcontrol() {
#if LEMLIB_SENSOR_TEST || LEMLIB_TURN_TEST || LEMLIB_TRACKING_CALIBRATION_TEST
  while (true) pros::delay(50);
#else
  aon::Configure();
  while (true) {
    #if TESTING_AUTONOMOUS
    aon::Configure(false); // Set drivetrain to hold for auton testing

    // TODO: add presetFunction
    aon::autonomousReader->ExecuteFunction("autonomous");

    pros::delay(5000);
    #else
    aon::operator_control::Run(driver);
    #endif
    pros::delay(10);
  }
#endif
}
