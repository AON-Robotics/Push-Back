#include "aon/core/robot.hpp"

#include "main.hpp"

#include "aon/competition/autonomous-routines.hpp"
#include "aon/lemlib/chassis.hpp"

namespace aon::core {

void Robot::initialize() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 8, 8, "AON boot: normal mode");

#if LEMLIB_SENSOR_TEST
  aon::logging::Initialize();
  aon::lemlib_integration::startSensorTest();
#else
  aon::logging::Initialize();
  aon::Configure(false);
  // Preselect the current JerryIO path autonomous; GUI selections overwrite it.
  aon::gui->selectedRedAut = 1;
  aon::autonomousReader->AddFunction("autonomous", aon::routines::RedRoutine1);
  pros::Task guiLoopTask([] { aon::gui->initialize(); });
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
