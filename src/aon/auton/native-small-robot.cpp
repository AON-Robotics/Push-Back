#include "aon/auton/routines.hpp"
#include "aon/auton/mechanism-actions.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"
#include "aon/globals.hpp"

namespace aon::routines {

#if !USING_BIG_ROBOT

void smallBotRoutine(){
  aon::auton::logStep("Kevin Loader", "start");
  aon::auton::logStep("Kevin Loader", "align loader");
  drivetrain.move(31); // Align with match loader
  drivetrain.turn(86);
  aon::auton::mechanisms::prepareLoaderCart();
  pros::delay(200);
  drivetrain.move(4); // Go to match loader
  aon::auton::mechanisms::beginLoaderCollection();
  aon::auton::logStep("Kevin Loader", "collect blocks");
  drivetrain.motors(-MAX_RPM / 2); // Jerk back
  pros::delay(100); // for an instance,
  drivetrain.motors(MAX_RPM / 2); // then push into loader
  pros::delay(400); // for a bit of time,
  drivetrain.stop(); // and stop.
  pros::delay(5000); // Take up some blocks (6);
  aon::auton::mechanisms::finishLoaderCollection();
  aon::auton::logStep("Kevin Loader", "approach long goal");
  drivetrain.move(-13); // Move to Long goal
  aon::auton::mechanisms::resetLoaderCart();
  drivetrain.turn(85);
  drivetrain.turn(85);
  aon::auton::mechanisms::prepareTopScorer();
  drivetrain.move(6.5);
  aon::auton::logStep("Kevin Loader", "score long goal");
  aon::auton::mechanisms::scoreTopBlocks(3000);
  aon::auton::logStep("Kevin Loader", "park setup");
  drivetrain.move(-23); // Go back a little
  drivetrain.turn(-93); // Orient towards parking
  drivetrain.move(20); // Go to parking
  drivetrain.motors(MAX_RPM);
  pros::delay(1200);
  drivetrain.stop();
  aon::auton::logStep("Kevin Loader", "finish");
  //* Works till here
}

void blackBeard(){
  drivetrain.move(30, false);
  drivetrain.driveAngleOfArc(-9, 50);
  intake.score(Intake::BOTTOM, 750);
  drivetrain.move(-9, false);
  drivetrain.driveAngleOfArc(-5, -155, false);
  intake.dropCart();
  drivetrain.move(28, false);
  drivetrain.driveAngleOfArc(6, 82);
  drivetrain.move(8);
  intake.activateScan();
  intake.move();
  drivetrain.jiggle(7, 110, 200);
  pros::delay(200);
  intake.stop();
  drivetrain.move(-13);
  intake.stopScan();
  intake.raiseCart();
  intake.raiseScorer();
  drivetrain.turnToHeading(7);
  intake.openTrapdoor();
  drivetrain.move(12);
  intake.store();
  pros::delay(500);
  intake.lever(750);
  pros::delay(500);
  intake.lever(750);
  pros::delay(500);
  intake.lever(750);
  pros::delay(500);
  intake.lever(750);
  pros::delay(500);
  intake.stop();
  drivetrain.driveAngleOfArc(8, -120, false);
  drivetrain.move(34, false);
  drivetrain.driveAngleOfArc(8.2, 37, false);
  drivetrain.move(-3, false);
  drivetrain.motors(MAX_RPM, 2000);
}

void jackSparrow(){
  intake.stopScan();
  drivetrain.move(30, false);
  drivetrain.driveAngleOfArc(-9, 50);
  intake.score(Intake::BOTTOM, 750);
  drivetrain.move(-4, false);
  drivetrain.driveAngleOfArc(-5, -180, false);
  drivetrain.driveAngleOfArc(5, 60, false);
  drivetrain.move(4, false);
  drivetrain.driveAngleOfArc(-5, 40, false);
  drivetrain.move(1, false);
  drivetrain.motors(600, 2000);
}

// TODO(AUTON-SMALL-WORLDS): Validate the full route before selection.
void smallBotRoutineWorlds(){
  drivetrain.move(31); // Align with match loader
  drivetrain.turn(90);
  intake.dropCart(); // Prepare loader mechanism
  intake.activateScan();
  drivetrain.move(4); // Go to match loader
  pros::delay(200);
  drivetrain.motors(-MAX_RPM / 2); // Jerk back
  pros::delay(100); // for an instance,
  drivetrain.motors(MAX_RPM / 2); // then push into loader
  pros::delay(400); // for a bit of time,
  drivetrain.stop(); // and stop.
  pros::delay(5000); // Take up some blocks (6);
  intake.stopScan();
  drivetrain.move(-6); // Move to Long goal
  intake.raiseCart(); // Reset loader mechanism
  drivetrain.turn(-180);
  intake.raiseScorer();
  drivetrain.move(13);
  intake.score(Intake::TOP, 3000); // Score all blocks
  drivetrain.move(-23); // Go back a little
  drivetrain.turn(-90); // Orient towards parking
  drivetrain.move(20); // Go to parking
  drivetrain.motors(MAX_RPM);
  pros::delay(1200);
  drivetrain.stop();
}

void smallBotCurves(){
  intake.dropCart(); // Prepare loader mechanism
  drivetrain.driveAngleOfArc(17, 160); // Align with match loader
  intake.activateScan();
  drivetrain.move(4); // Go to match loader
  pros::delay(200);
  drivetrain.motors(-MAX_RPM / 2, 100); // Jerk back for an instance,
  drivetrain.motors(MAX_RPM / 2, 400); // then push into loader for a bit of time,
  pros::delay(3000); // Take up some blocks (6);
  intake.stopScan();
  drivetrain.driveAngleOfArc(-10, -90); // Move to Long goal
  intake.raiseCart(); // Reset loader mechanism
  drivetrain.driveAngleOfArc(10, 90);
  intake.raiseScorer();
  intake.openTrapdoor();
  drivetrain.move(5);
  intake.store(500);
  intake.lever(); // Score some blocks
  drivetrain.motors(MAX_RPM / 2, 200);
  pros::delay(200);
  drivetrain.move(-20); // Go back a little
  intake.closeTrapdoor();
  intake.lowerScorer();
  drivetrain.turn(-100); // Orient towards parking
  drivetrain.move(20); // Go to parking,
  drivetrain.motors(MAX_RPM, 1000); // and push into it.
}

void smallBotPark(){
  aon::auton::logStep("Kevin Park", "start");
  drivetrain.move(-5, false);
  aon::auton::logStep("Kevin Park", "parking push");
  drivetrain.motors(MAX_RPM, 500); // Push into parking to put a row of wheels over for a bit of time, then stop.
  aon::auton::logStep("Kevin Park", "deploy park mechanism");
  aon::auton::mechanisms::deployParkMechanism();
  aon::auton::logStep("Kevin Park", "finish");
}

#endif

}  // namespace aon::routines
