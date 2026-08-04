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
