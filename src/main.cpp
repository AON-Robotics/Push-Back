#include "../include/main.hpp"

void initialize() {
  pros::Task guiTask(aon::gui::Initialize);
  aon::logging::Initialize();
  pros::lcd::initialize();
  aon::Configure(false);
  aon::odometry::Initialize();
  pros::Task odomTask(aon::odometry::Odometry);
  pros::Task safetyTask(aon::autonSafety);
  pros::Task intakeTask([]{intake.scan();});
  pros::Task turretFollowTask([]{orbit.follow();});
  pros::Task turretScanTask([]{orbit.scan();});
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
  aon::Configure();
  while (true) {
    #if TESTING_AUTONOMOUS
    aon::Configure(false); // Set drivetrain to hold for auton testing

    aon::AutonomousReader->ExecuteFunction("autonomous");

    pros::delay(3000);
    #else
    aon::operator_control::Run(aon::operator_control::DEFAULT);
    #endif
    pros::delay(10);
  }
}
