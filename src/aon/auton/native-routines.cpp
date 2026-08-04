#include "aon/auton/routines.hpp"
#include "aon/auton/mechanism-actions.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"
#include "aon/globals.hpp"

namespace aon {
// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

namespace routines {

#if USING_BIG_ROBOT

/// @brief Starting position is the left side of the parking facing towards the drive team, placed parallel to the side of the parking with the second shaft of the drivetrain aligned with the end of the goal
void safeBigBotRoutine(){
  intake.activateScan();
  drivetrain.strafe(28.5); // Align with match loader.
  intake.dropCart(); // Prepare loader mechanism.
  drivetrain.move(6); // Move to match loader.
  drivetrain.motors(MAX_RPM / 2); // Push into loader
  pros::delay(200); // for a bit of time,
  drivetrain.stop(); // then stop.
  pros::delay(8000); // Take up all the blocks (9).
  drivetrain.move(-22); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2); // Push into goal
  pros::delay(200); // for a bit of time,
  drivetrain.stop(); // then stop.
  intake.raiseCart(); // Reset loader mechanism.
  intake.score(Intake::TOP, 8000); // Score all 9 blocks.
  drivetrain.move(15); // Go back a little.
  drivetrain.turn(-90); // Orient towards parking.
  drivetrain.move(12); // Move towards parking.
  drivetrain.strafe(12); // Align with parking.
  drivetrain.move(11); // Move to parking.
  drivetrain.motors(MAX_RPM); // Push into parking to put a row of wheels over
  pros::delay(1000); // for a bit of time,
  drivetrain.stop(); // then stop.
  brooks.activate(); // Park.
  intake.stopScan();
  //* Works till here
}

void bigBotCurves(){
  intake.activateScan();
  drivetrain.strafe(28.5); // Align with match loader.
  intake.dropCart(); // Prepare loader mechanism.
  drivetrain.move(6); // Move to match loader.
  drivetrain.motors(MAX_RPM / 2, 200); // Push into loader for a bit of time, then stop.
  pros::delay(7000); // Take up all the blocks (9).
  drivetrain.move(-22); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2, 200); // Push into goal for a bit of time, then stop.
  intake.raiseCart(); // Reset loader mechanism.
  intake.score(Intake::TOP, 8000); // Score all 9 blocks.

  drivetrain.driveAngleOfArc(-15, 90);

  // drivetrain.move(15); // Go back a little.
  // drivetrain.turn(-90); // Orient towards parking.
  // drivetrain.move(12); // Move towards parking.
  
  drivetrain.strafe(12); // Align with parking.
  drivetrain.move(11); // Move to parking.
  drivetrain.motors(MAX_RPM, 1000); // Push into parking to put a row of wheels over for a bit of time, then stop.
  brooks.activate(); // Park.
  intake.stopScan();
}

void bigBotContinuity(){
  // Sorting remains disabled because rejection changes route timing.
  drivetrain.strafe(28.5); // Align with match loader.
  intake.dropCart(); // Prepare loader mechanism.
  drivetrain.move(5, false); // Move to match loader.
  drivetrain.motors(MAX_RPM / 2, 200); // Push into loader for a bit of time, then stop.
  intake.store(6000); // Take up all the blocks (12).
  drivetrain.move(-20, false); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2, 200); // Push into goal for a bit of time, then stop.
  intake.raiseCart(); // Reset loader mechanism.
  intake.score(Intake::TOP, 900); // Score 3 blocks.
  intake.score(Intake::BOTTOM, 300); // Kick back intake to unjam blocks
  intake.score(Intake::MIDDLE, 2000); // Reject 3 blocks.
  intake.score(Intake::TOP, 2000); // Score 3 blocks.
  drivetrain.driveAngleOfArc(-16, 40); // Align with middle goal
  drivetrain.move(-47.5); // Go to middle goal
  intake.score(Intake::BOTTOM, 300); // Kick back intake to unjam blocks
  intake.score(Intake::MIDDLE, 2000); // Score 3 blocks.
  drivetrain.move(44.5); // Go back to long goal
  drivetrain.turn(-45); // Align with long goal
  drivetrain.strafe(9); // Push against it to block descoring
}

void bigBotStayThere(){
  aon::auton::logStep("Kevin Loader", "start");
  // Sorting remains disabled because rejection changes route timing.
  aon::auton::logStep("Kevin Loader", "align loader");
  drivetrain.strafe(28.5); // Align with match loader.
  aon::auton::mechanisms::prepareLoaderCart();
  drivetrain.move(5, false); // Move to match loader.
  aon::auton::mechanisms::beginLoaderCollection();
  drivetrain.motors(MAX_RPM / 2, 200); // Push into loader for a bit of time, then stop.

  aon::auton::logStep("Kevin Loader", "collect blocks");
  drivetrain.jiggle(16, 120, 200); // Use if cart kinda works

  aon::auton::logStep("Kevin Loader", "approach long goal");
  drivetrain.move(-20, false); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2, 200); // Push into goal for a bit of time, then stop.
  aon::auton::mechanisms::resetLoaderCart();

  aon::auton::logStep("Kevin Loader", "score long goal");
  if (intake.startReleasing(Intake::Height::TOP)) {
    pros::delay(8000);
  } else {
    intake.stop();
    aon::auton::mechanisms::finishLoaderCollection();
    aon::auton::logStep("Kevin Loader", "sorter unavailable; abort");
    return;
  }
  aon::auton::mechanisms::finishLoaderCollection();
  intake.stopReleasing();
  // intake.score(Intake::TOP, 900); // Score 3 blocks.
  // intake.score(Intake::BOTTOM, 300); // Kick back intake to unjam blocks
  // intake.score(Intake::MIDDLE, 2000); // Reject 3 blocks.
  // intake.score(Intake::TOP, 5000); // Score 6 blocks.

  aon::auton::logStep("Kevin Loader", "block descoring");
  drivetrain.move(6); // Move back a bit
  drivetrain.turn(-90); // Align wall with long goal
  drivetrain.strafe(9); // Push against it to block descoring
  aon::auton::logStep("Kevin Loader", "finish");
}

// TODO(AUTON-BIG-LONG-GOAL): Validate distances with the loaded robot.
void bigBotLongGoalThenPark(){
  // Sorting remains disabled because rejection changes route timing.
  drivetrain.strafe(28.5); // Align with match loader.
  intake.dropCart(); // Prepare loader mechanism.
  drivetrain.move(5, false); // Move to match loader.
  drivetrain.motors(MAX_RPM / 2, 200); // Push into loader for a bit of time, then stop.
  intake.store(6000); // Take up all the blocks (12).
  drivetrain.move(-20, false); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2, 200); // Push into goal for a bit of time, then stop.
  intake.raiseCart(); // Reset loader mechanism.
  intake.score(Intake::TOP, 900); // Score 3 blocks.
  intake.score(Intake::BOTTOM, 300); // Kick back intake to unjam blocks
  intake.score(Intake::MIDDLE, 2000); // Reject 3 blocks.
  intake.score(Intake::TOP, 5000); // Score 6 blocks.
  drivetrain.driveAngleOfArc(16, 80); // Start aligning with goal
  drivetrain.strafe(20); // Align with goal
  drivetrain.move(6, false); // Get close to goal,
  drivetrain.motors(MAX_RPM, 1000); // then push in
  brooks.activate(); // and park.
}

void bigBotPark(){
  aon::auton::logStep("Kevin Park", "start");
  aon::auton::logStep("Kevin Park", "parking push");
  drivetrain.motors(MAX_RPM, 1000); // Push into parking to put a row of wheels over for a bit of time, then stop.
  aon::auton::logStep("Kevin Park", "deploy park mechanism");
  aon::auton::mechanisms::deployParkMechanism();
  aon::auton::logStep("Kevin Park", "finish");
}

void BigBotSkillsRoutine(){
  drivetrain.strafe(28.5);
  intake.dropCart(); 
  drivetrain.move(6); 
  drivetrain.motors(MAX_RPM / 2); 
  pros::delay(200); 
  drivetrain.stop(); 
  pros::delay(8000); 
  drivetrain.move(-22);
  intake.raiseCart();
  drivetrain.motors(-MAX_RPM / 2);
  drivetrain.move(22);
  drivetrain.turn(180);
  drivetrain.strafe(-10.5);
  drivetrain.move(50);
  drivetrain.strafe(10.5);
  intake.dropCart(); 
  drivetrain.move(6); 
  drivetrain.motors(MAX_RPM / 2); 
  pros::delay(200); 
  drivetrain.stop(); 
  pros::delay(8000); 
  drivetrain.strafe(10);
  intake.dropCart(); 
  drivetrain.turn(90);
  drivetrain.motors(MAX_RPM);
  pros::delay(1000); 
  drivetrain.stop(); 
  brooks.activate();
}

#else

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

void smallbotjorgeg(){
  drivetrain.move(31); // Align with match loader
  drivetrain.turn(87);
  intake.dropCart(); // Prepare loader mechanism
  pros::delay(200);  
  drivetrain.move(5); // Go to match loader
  intake.activateScan(); 
  drivetrain.motors(-MAX_RPM / 2); // Push into loader
  pros::delay(100); // for a bit of time,
  drivetrain.motors(MAX_RPM / 2); // Push into loader
  pros::delay(300); // for a bit of time,
  drivetrain.stop(); // then stop.
  pros::delay(5000); // Take up some blocks (6);
  intake.stopScan();
  drivetrain.move(-13); // Move to Long goal
  intake.raiseCart(); // Reset loader mechanism
  // drivetrain.turn(173);
  drivetrain.turn(85);
  drivetrain.move(1);
  drivetrain.turn(85);
  intake.raiseScorer();
  drivetrain.move(6.5);
  intake.score(Intake::TOP, 1000); // Score all blocks
  drivetrain.move(-6.5); // reset long goal distance
  drivetrain.turn(-45); // face bottom middle goal
  drivetrain.move(-31); // going backward
  // intake.setScorerHeight(LOW);// prepare for middle-middle goal
  // intake.move(); //score in bottom goal
  drivetrain.move(3);
  drivetrain.turn(90); // allign for middle-middle
  drivetrain.move(20);
  drivetrain.turn(-45);
  drivetrain.move();  
}

#endif

} // namespace aon::routines

};  // namespace aon

#include "aon/auton/routines.hpp"
