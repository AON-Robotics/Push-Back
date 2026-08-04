#include "aon/auton/routines.hpp"
#include "aon/auton/mechanism-actions.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"
#include "aon/globals.hpp"

namespace aon::routines {

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

#endif

}  // namespace aon::routines
