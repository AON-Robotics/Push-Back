#include "../include/main.hpp"

void initialize() {
  aon::gui::Initialize();
  aon::logging::Initialize();
  aon::Configure(false);
  aon::odometry::Initialize();
  pros::Task odomTask(aon::odometry::Odometry);
  pros::Task safetyTask(aon::autonSafety);
  // pros::Task turretFollowTask([]{orbit.follow();});
  // pros::Task turretScanTask([]{orbit.scan();}); // TODO: combine this with the follow task
  pros::Task intakeScanning([]{intake.scan();});
  pros::Task intakeSorting([]{intake.sort();});
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
    // aon::odometry::Debug();
    // std::cout << "Robot move 12 inches to the front.";
    // drivetrain.move(12);
    // std::cout << "Robot moved 12 inches to the front.";
    // drivetrain.turn(90);
    // std::cout << "Robot moved 12 inches to the front.";
    // drivetrain.moveHorizontalProfiled(6);
    // std::cout << "Robot moved 6 inches to the right.";
    drivetrain.move2D(12, 12, 90);
    std::cout << "Robot moved 6 inches to the right, 6 inches to the front, and turn 90 degrees.";

    pros::delay(5000);
    // drivetrain.MoveHolonomicMotionH(6, 6, 90);

    // aon::AutonomousReader->ExecuteFunction("autonomous");

    pros::delay(5000);
    #else
    aon::operator_control::Run(aon::operator_control::DEFAULT);
    #endif
    pros::delay(10);
  }
}
