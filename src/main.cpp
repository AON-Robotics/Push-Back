#include "main.hpp"

void initialize() {
  pros::Task guiTask(aon::gui::Initialize);
  aon::logging::Initialize();
  pros::lcd::initialize();
  aon::ConfigureMotors(false);
  aon::ConfigureColors();
  aon::odometry::Initialize();
  pros::Task odomTask(aon::odometry::Odometry);
  pros::Task safetyTask(aon::autonSafety);
  pros::Task turretFollowTask(aon::turretFollow);
  pros::Task intakeTask(aon::intakeScan);
  pros::Task turretScanTask(aon::turretScan); // TODO: combine this with the follow task
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  aon::AutonomousReader->ExecuteFunction("autonomous");
  pros::delay(10);
}

// During development
// Program slot 1 with Pizza Icon is for opcontrol
// Program slot 2 with Planet Icon is for autonomous routine
// Program slot 3 with Alien Icon is for tests or miscellaneous components
void opcontrol() {
  aon::ConfigureMotors();
  while (true) {
    #if TESTING_AUTONOMOUS
    aon::ConfigureMotors(false); // Set drivetrain to hold for auton testing

    aon::AutonomousReader->ExecuteFunction("autonomous");

    pros::delay(3000);
    #else
    aon::operator_control::Run(aon::operator_control::DEFAULT);
    #endif
    pros::delay(10);
  }
}
