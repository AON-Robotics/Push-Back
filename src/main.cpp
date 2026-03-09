#include "../include/main.hpp"


void initialize() {
  pros::Task guiLoopTask([]{aon::InitializeGui(); aon::gui.RunLoop();});
  aon::logging::Initialize();
  aon::Configure(false);
  pros::Task odomTask([]{drivetrain.odom.initialize();});
  pros::delay(3000);
  pros::Task safetyTask(aon::autonSafety);
  // pros::Task turretFollowTask([]{orbit.follow();});
  // pros::Task turretScanTask([]{orbit.scan();}); // TODO: combine this with the follow task
  pros::Task intakeScanning([]{intake.scan();});
  pros::Task intakeSorting([]{intake.sort();});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  aon::Configure(false); // Set drivetrain to hold for auton
  #if USING_BIG_ROBOT
  aon::safeBigBotRoutine();
  #else
  aon::testSmallBotRoutine();
  #endif
  // aon::AutonomousReader->ExecuteFunction("autonomous");
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

    #if USING_BIG_ROBOT
    aon::safeBigBotRoutine();
    #else
    // aon::testSmallBotRoutine();
    aon::testXDriveRoutine();
    #endif

    pros::delay(5000);
    #else
    aon::operator_control::Run(aon::operator_control::DEFAULT);
    #endif
    pros::delay(10);
  }
}
