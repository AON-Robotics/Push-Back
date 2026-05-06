#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../odometry/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../tools/logging.hpp"
#include "../tools/moving-average.hpp"
#include "../tools/general.hpp"
#include "../math/misc/misc.hpp"

// TODO: for modularity we will have odometry, drivetrain, navigator, orbit, intake, and claw (the last two will most likely change with each game and modules may be added or removed as needed)
// TODO: add support for a drive mode that is videogame-like (i think rocket league has it). Basically with reference to where the driver is standing on the field, the direction towards which you move the joystick is where the robot will turn to and drive to at the same time. This should greatly facilitate general directional movement if implemented correctly. Leave a toggle available for traditional driving in accordance to the chosen drivetrain for better fine grained control in tight spaces.
// TODO: odometry should also have an traditional odometer functionality to track how much distance the robot has traveled and also use integration for all measurements as a fallback if sensors fail


/**
 * For GPS coord system: https://pros.cs.purdue.edu/v5/tutorials/topical/gps.html
 */

namespace aon {


// ============================================================================|
//   ____        _       ____             _   _                
//  / ___| _   _| |__   |  _ \ ___  _   _| |_(_)_ __   ___  ___
//  \___ \| | | | '_ \  | |_) / _ \| | | | __| | '_ \ / _ \/ __|
//   ___) | |_| | |_) | |  _ < (_) | |_| | |_| | | | |  __/\__ \
//  |____/ \__,_|_.__/  |_| \_\___/ \__,_|\__|_|_| |_|\___||___/
// ============================================================================|



/**
 * \brief Aligns ORBIT and DRIVETRAIN to the item with the set `COLOR`
 * 
 * \param color The color of the object to which we wish to align ourselves
 * 
 * \note Setting `color` to `STAKE` makes the robot turn 180° after alignment
 */
void alignRobotTo(const Colors &color = orbit.getColor()){
  orbit.setColor(orbit.getColor());
  orbit.follow();
  pros::delay(500);
  const double tolerance = 5;
  double difference;
  while(!orbit.isAligned(tolerance)){
    difference = orbit.difference();
    double SPEED = turnPID.Output(0, -difference) * 400;
    drivetrain.rotate(SPEED);
    pros::delay(20);
  }
  drivetrain.stop();  
  orbit.deactivateFollow();
  if(color == STAKE){
    drivetrain.turn(180);
  }
}


/// @brief Calculates the distance to a ring of the specified `color` using a EKF
/// @param color The color of the ring we wish to track
/// @return The filtered distance to that ring
/// @note Takes half a second (0.5s) to complete
double getDistanceToRing(const Colors &color = orbit.getColor()){
  orbit.setColor(orbit.getColor());
  okapi::EKFFilter ekf;

  double distance;

  // Filter the distance for half a second using 100 measurements (1 every 5 milliseconds)
  for(int i = 0; i < 100; i++){
    distance = ekf.filter(orbit.groundDistanceToDisk((orbit.getLargestObject()).width));
    pros::delay(5);
  }

  return distance;
}

/// @brief Drives forward until a ring hits the distance sensor
/// @param distance The distance from the robot to a ring
void driveTillPickUp(const double &distance = getDistanceToRing()){
  const double additional_distance = 0; //? This is to give the robot some distance to actually grip the donut, determine this experimentally
  intake.activateScan();
  drivetrain.move(distance + additional_distance);
  intake.stopScan();
}

/// @brief Aligns robot to the ring of the specified `color` and grabs it and scores it on the held stake
/// @param color The color of the ring to be picked up
void alignAndIntake(const Colors &color = orbit.getColor()){
  orbit.setColor(color);
  drivetrain.move(12);
  alignRobotTo(orbit.getColor());
  drivetrain.move(12);
  driveTillPickUp();
  intake.score();
}

/// @brief Uses the ORBIT to adjust the path going toward a ring to intake it accurately
/// @param color The color of the ring we wish to intake
void driveIntoRing(const Colors &color = orbit.getColor()){
  orbit.setColor(orbit.getColor());
  orbit.follow();
  intake.activateScan();
  pros::delay(500);
  okapi::EKFFilter ekf;
  const double tolerance = 5; //? Probably adjust this
  double difference;

  const double dt = 0.02;

  drivetrain.setMaxVelocity(MAX_RPM / 2);

  while(!orbit.isAligned(tolerance)){
    auto object = (orbit.getLargestObject());

    // Safety to scan when object is lost
    if(object.signature != color) { 
      orbit.activateScan();
      drivetrain.stop();
      continue;
    } 
    else {
      orbit.activateFollow();
    }

    difference = orbit.difference();
    //? maybe motion profile this variable
    double TURN = turnPID.Output(0, -difference) * 500;
    
    const double distance = ekf.filter(orbit.groundDistanceToDisk((orbit.getLargestObject()).width));

    double FORWARD = drivetrain.updateProfile(distance, dt);

    drivetrain.arcade(FORWARD, TURN);
    pros::delay(20);
  }
  #undef TIME
  orbit.deactivateFollow();
  orbit.deactivateScan();
  intake.stopScan();
  drivetrain.setMaxVelocity(MAX_RPM);
  driveTillPickUp();
}


// ============================================================================
//   _____ ___ ___ _____ ___
//  |_   _| __/ __|_   _/ __|
//    | | | _|\__ \ | | \__ \
//    |_| |___|___/ |_| |___/
//
// ============================================================================

namespace tests {

/// @brief Basic Routine to make the robot go in circles around the map to test GPS setup.
void gpsOctagon() {
  drivetrain.goToPoint(.6, -1.2);
  drivetrain.goToPoint(1.2, -.6);
  drivetrain.goToPoint(1.2, .6);
  drivetrain.goToPoint(.6, 1.2);
  drivetrain.goToPoint(-.6, 1.2);
  drivetrain.goToPoint(-1.2, .6);
  drivetrain.goToPoint(-1.2, -.6);
  drivetrain.goToPoint(-.6, -1.2);
  drivetrain.goToPoint(.6, -1.2);
  drivetrain.goToPoint(1.2, -.6);
}

/// @brief  Speed calculation test using the distance sensor
/// @param RPM The velocity for the motors
void distanceSensorSpeed(double RPM = MAX_RPM){
  MovingAverage mav(50);
  while(true) {
    drivetrain.motors(RPM);
    // double measured = math::metersToInches(distanceSensor.get_object_velocity());
    // double calculated = getSpeed(RPM);
    // double error = abs(math::getErrorPercentage(calculated, measured));
    // double avg = mav.update(error);
    // pros::lcd::print(1, "RPM: %.2f", RPM);
    // pros::lcd::print(2, "Calculated Velocity: %.2f", calculated);
    // pros::lcd::print(3, "Measured Velocity: %.2f", measured);
    // pros::lcd::print(4, "Error %: %.2f%", avg);
    pros::delay(10);
  }
}

/// @brief Small test to see if odom works with auton
void odom(){
  // Motion Profile
  drivetrain.move(12 * 3);
  pros::delay(1000);
  drivetrain.move(-12 * 3);
  pros::delay(1000);
  drivetrain.turn(90);
  pros::delay(1000);
  drivetrain.turn(-90);
  pros::delay(1000);

  // PID Forward
  drivetrain.drivePID(drivePID, 12 * 3, 100.0);
  pros::delay(1000);
  drivetrain.drivePID(drivePID, -12 * 3, 100.0);
  pros::delay(1000);

  // PID Rotations
  drivetrain.turnPID(turnPID, 90, 50.0);
  pros::delay(1000);
  drivetrain.turnPID(turnPID, -90, 50.0);
  pros::delay(1000);
  drivetrain.turnPID(turnPID, 45, 50.0);
  pros::delay(1000);
  drivetrain.turnPID(turnPID, 45, 50.0);
  pros::delay(1000);
  drivetrain.turnPID(turnPID, -45, 50.0);
  pros::delay(1000);
  drivetrain.turnPID(turnPID, -45, 50.0);
  pros::delay(1000);
}

/// @brief Test to ensure the concurrency is working fine, requires `intake.scan()` to be running in another thread
void concurrency(){
  intake.activateScan();
  int startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime
  while(time < 5){
    drivetrain.motors(100);
    pros::delay(20);
  }
  #undef time
  drivetrain.stop();
  intake.stopScan();
}

/// @brief Tests the alignment of the robot to the object of `COLOR` using tasks
void alignment(){
  while(true){
    alignRobotTo(RED);
    pros::delay(20);
  }
}

/// @brief Outputs and logs the width of a ring, and the distance to it based on that width
void visionSensorDistance(){
  MovingAverage readingMav(50);
  MovingAverage avgMav(50);
  MovingAverage ekfMav(50);
  MovingAverage avg_ekfMav(50);
  okapi::EKFFilter ekf;
  orbit.activateFollow();
  while(true){
    pros::vision_object block = orbit.getLargestObject();
    if(block.signature == RED){
      const double distance = orbit.groundDistanceToDisk(block.width);
      if(!::std::isnormal(distance)) { continue; }
      const double avg = readingMav.update(distance);
      const double filtered = ekf.filter(distance); // this seems to be the best alternative out of the 2
      const double avgDif = avgMav.update(math::getPercentDifference(avg, distance));
      const double ekfDif = ekfMav.update(math::getPercentDifference(filtered, distance));
      const double avg_ekfDif = avg_ekfMav.update(math::getPercentDifference(avg, filtered));
      pros::lcd::print(0, "Ring width = %d", block.width);
      pros::lcd::print(1, "Raw Distance = %.2f in", distance);
      pros::lcd::print(2, "MAV-50 Distance = %.2f in", avg);
      pros::lcd::print(3, "EKF Distance = %.2f in", filtered);
      pros::lcd::print(4, "MAV-50 Dif = %.2f %", avgDif);
      pros::lcd::print(5, "EKF Dif = %.2f %", ekfDif);
      pros::lcd::print(6, "MAV-EKF Dif = %.2f %", avg_ekfDif);
    }
    pros::delay(20);
  }
  orbit.deactivateFollow();
}

/// @brief Uses the gyro to test the precision of an ekf
void gyroWithEKF(){
  okapi::EKFFilter ekf1;
  okapi::EKFFilter ekf2(2.6E-4, 0.04);
  okapi::EKFFilter ekf3(3E-4, 0.04);
  okapi::EKFFilter ekf4(4E-4, 0.04); 
  okapi::EKFFilter ekf5(5E-4, 0.04);
  while(true){
    const double pos = odometry.gyroscope.get_heading();
    pros::lcd::print(0, "Raw Heading = %.2f", pos);
    pros::lcd::print(1, "Default Filter = %.2f", ekf1.filter(pos));
    pros::lcd::print(2, "Tweaked Filter 2 = %.2f", ekf2.filter(pos)); // this one is slower which might mean i want to tweak the values for the ekf
    pros::lcd::print(3, "Tweaked Filter 3 = %.2f", ekf3.filter(pos));
    pros::lcd::print(4, "Tweaked Filter 4 = %.2f", ekf4.filter(pos));
    pros::lcd::print(5, "Tweaked Filter 5 = %.2f", ekf5.filter(pos));
    pros::delay(20);
  }
}

/// @brief Function wrapper for test function that is to be executed through the GUI
/// @return 1 for successful execution
/// @note Usually the tests in here use `potentiometer.get_value()` to tune a parameter in a function as well as testing the function itself
int adjustable(){
  drivetrain.driveInArcTo(math::inchesToMeters(TILE_WIDTH / 2), math::inchesToMeters(TILE_WIDTH / 2));
  return 1;
}

/// @brief Function wrapper for test functions that are to be executed through the GUI
/// @return 1 for successful execution
/// @note Choose between 3 tests depending on the result of `potentiometer.get_value()`
int multiple(){
  int choice = potentiometer.get_value();
  // UP
  if(choice > 2550){
    drivetrain.driveInArcTo(-math::inchesToMeters(TILE_WIDTH / 2), math::inchesToMeters(TILE_WIDTH / 2));
  }
  // MIDDLE
  else if (choice > 1100){
    drivetrain.driveInArcTo(-math::inchesToMeters(TILE_WIDTH / 2), -math::inchesToMeters(TILE_WIDTH / 2));
  }
  // DOWN
  else {
    drivetrain.driveInArcTo(math::inchesToMeters(TILE_WIDTH / 2), -math::inchesToMeters(TILE_WIDTH / 2));
  }
  return 1;
}

void turns(){
  for (int i = 0; i < 4; i++){drivetrain.turn(); pros::delay(750);}
  for (int i = 0; i < 4; i++){drivetrain.turn(-90); pros::delay(750);}
}

void square(){
  for (int i = 0; i < 4; i++){
    drivetrain.move();
    pros::delay(750);
    drivetrain.turn();
    pros::delay(750);
  }
}

void continuity(){
  drivetrain.driveAngleOfArc(8, 180, false);
  drivetrain.driveAngleOfArc(-8, 180, false);
  drivetrain.move(6);
  drivetrain.move(-6, false);
  drivetrain.driveAngleOfArc(-8, -180, false);
  drivetrain.driveAngleOfArc(8, -180);
}

void colorSorting(){
  intake.activateScan();
}

void purePursuitPoint(){
  drivetrain.goToPose(Pose(TILE_WIDTH, -TILE_WIDTH / 2, 0));
  drivetrain.goToPose(Pose(TILE_WIDTH * 2, TILE_WIDTH / 2, 90));
  drivetrain.goToPose(Pose(0, TILE_WIDTH / 2, 180));
  drivetrain.goToPose(Pose(0, -TILE_WIDTH / 2, 270));
  drivetrain.goToPose(Pose(TILE_WIDTH, TILE_WIDTH / 2, 0));
  drivetrain.goToPose(Pose(TILE_WIDTH * 2, -TILE_WIDTH / 2, 90));
  drivetrain.goToPose(Pose(0, 0, 0));
}

void purePursuitSimpleFollow(){
  std::vector<Pose> path = {
    Pose(TILE_WIDTH, 0, 0),
    Pose(0, 12, 0),
    Pose(-12, 12, 0),
    Pose(0, 0, 90),
    Pose(-12, 18, 0),
    Pose(-TILE_WIDTH, TILE_WIDTH, 90),
    Pose(0, 0, 0),
   
  };
  drivetrain.follow(path);
}

void purePursuitPath(){
  std::vector<Pose> path = {
    Pose(0, 0, 0),
    Pose(1, 0.5, 0),
    Pose(2, 1.2, 0),
    Pose(3, 2.0, 0),
    Pose(4, 3.0, 0),
    Pose(5, 4.2, 0),
    Pose(6, 5.5, 0),
    Pose(7, 6.8, 0),
    Pose(8, 8.0, 0),
    Pose(9, 9.0, 0),
    Pose(10, 9.5, 0),

    // curve back (middle of S)
    Pose(11, 9.0, 0),
    Pose(12, 8.0, 0),
    Pose(13, 6.8, 0),
    Pose(14, 5.5, 0),
    Pose(15, 4.2, 0),
    Pose(16, 3.0, 0),
    Pose(17, 2.0, 0),
    Pose(18, 1.2, 0),
    Pose(19, 0.5, 0),
    Pose(20, 0.0, 0),

    // final curve (irregularity)
    Pose(21, -0.8, 0),
    Pose(22, -1.5, 0),
    Pose(23, -2.0, 0),
    Pose(24, -2.3, 0),
    Pose(25, -2.5, 0),
    Pose(26, -2.6, 0),
    Pose(27, -2.4, 0),
    Pose(28, -2.0, 0),
    Pose(29, -1.2, 0),
    Pose(30, 0.0, 0),
    Pose(31, 1.5, 0),
    Pose(32, 3.0, 0),
    Pose(33, 4.5, 0),
    Pose(34, 6.0, 0),
    Pose(35, 7.5, 0),
    Pose(36, 9.0, 0),
    Pose(37, 10.5, 0),
    Pose(38, 12.0, 270) // test with 180 as well
  };
  drivetrain.follow(path);
}

#if USING_BIG_ROBOT



#else

void xDriveRoutine(){
  drivetrain.goToPose(Pose(-TILE_WIDTH, 0, 0));
  drivetrain.goToPose(Pose(0, 12, 0));
  drivetrain.goToPose(Pose(-12, 12, 0));
  drivetrain.goToPose(Pose(0, 0, 90));
  drivetrain.goToPose(Pose(-12, 18, 0));
  drivetrain.goToPose(Pose(-TILE_WIDTH, TILE_WIDTH, 90));
  drivetrain.goToPose(Pose(0, 0, 0));
}

#endif

} // namespace aon::tests

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
  
  // drivetrain.goToPose(Pose(0,0,-90)); // TODO: doing this depends on whether the new odom works
  drivetrain.strafe(12); // Align with parking.
  drivetrain.move(11); // Move to parking.
  drivetrain.motors(MAX_RPM, 1000); // Push into parking to put a row of wheels over for a bit of time, then stop.
  brooks.activate(); // Park.
  intake.stopScan();
}

void bigBotContinuity(){
  // TODO: check color sorting integration
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
  // TODO: check color sorting integration
  drivetrain.strafe(28.5); // Align with match loader.
  intake.dropCart(); // Prepare loader mechanism.
  drivetrain.move(5, false); // Move to match loader.
  intake.activateScan();
  drivetrain.motors(MAX_RPM / 2, 200); // Push into loader for a bit of time, then stop.

  drivetrain.jiggle(16, 120, 200); // Use if cart kinda works
  // pros::delay(8000); // Use if cart works
  
  drivetrain.move(-20, false); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2, 200); // Push into goal for a bit of time, then stop.
  intake.raiseCart(); // Reset loader mechanism.

  intake.setSortHeights(Intake::Height::TOP);
  intake.startReleasing();
  pros::delay(8000);
  intake.stopScan();
  intake.stopReleasing();
  // intake.score(Intake::TOP, 900); // Score 3 blocks.
  // intake.score(Intake::BOTTOM, 300); // Kick back intake to unjam blocks
  // intake.score(Intake::MIDDLE, 2000); // Reject 3 blocks.
  // intake.score(Intake::TOP, 5000); // Score 6 blocks.

  drivetrain.move(6); // Move back a bit
  drivetrain.turn(-90); // Align wall with long goal
  drivetrain.strafe(9); // Push against it to block descoring
}

// TODO: tune distances
void bigBotLongGoalThenPark(){
  // TODO: check color sorting integration
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
  drivetrain.motors(MAX_RPM, 1000); // Push into parking to put a row of wheels over for a bit of time, then stop.
  brooks.activate(); // Park.
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
  //empy the match loader 
  //empty match loader across 
  //Park
}

// Wrappers for the GUI

int RedRoutine1(){
  bigBotStayThere();
  return 1;
}

int RedRoutine2(){
  bigBotContinuity();
  return 1;
}

int RedRoutine3(){
  bigBotLongGoalThenPark();
  return 1;
}

int BlueRoutine1(){
  bigBotStayThere();
  return 1;
}

int BlueRoutine2(){ 
  bigBotPark();
  return 1;
}

int BlueRoutine3(){
  aon::tests::turns();
  return 1;
}

int SkillsRoutine1(){
  bigBotContinuity();
  return 1;
}

int SkillsRoutine2(){ 
  aon::tests::square();
  return 1;
}

int SkillsRoutine3(){
  aon::tests::turns();
  return 1;
}

#else

void smallBotRoutine(){
  drivetrain.move(31); // Align with match loader
  drivetrain.turn(86);
  intake.dropCart(); // Prepare loader mechanism
  pros::delay(200);  
  drivetrain.move(4); // Go to match loader
  intake.activateScan(); 
  drivetrain.motors(-MAX_RPM / 2); // Jerk back
  pros::delay(100); // for an instance,
  drivetrain.motors(MAX_RPM / 2); // then push into loader
  pros::delay(400); // for a bit of time,
  drivetrain.stop(); // and stop.
  pros::delay(5000); // Take up some blocks (6);
  intake.stopScan();
  drivetrain.move(-13); // Move to Long goal
  intake.raiseCart(); // Reset loader mechanism
  drivetrain.turn(85);
  drivetrain.turn(85);
  intake.raiseScorer();
  drivetrain.move(6.5);
  intake.score(Intake::TOP, 3000); // Score all blocks
  drivetrain.move(-23); // Go back a little
  drivetrain.turn(-93); // Orient towards parking
  drivetrain.move(20); // Go to parking
  drivetrain.motors(MAX_RPM);
  pros::delay(1200);
  drivetrain.stop(); 
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

// TODO: test
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
  drivetrain.move(-5, false);
  drivetrain.motors(MAX_RPM, 500); // Push into parking to put a row of wheels over for a bit of time, then stop.
  brooks.activate(); // Park.
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

// Wrappers for the GUI

int RedRoutine1() {
  blackBeard();
  return 1;
}

int RedRoutine2(){
  jackSparrow();
  return 1;
}

int RedRoutine3(){
  smallBotPark();
  return 1;
}

int BlueRoutine1(){
  blackBeard();
  return 1;
}

int BlueRoutine2(){ 
  jackSparrow();
  return 1;
}

int BlueRoutine3(){
  aon::tests::turns();
  return 1;
}

int SkillsRoutine1(){
  blackBeard();
  return 1;
}

int SkillsRoutine2(){ 
  jackSparrow();
  return 1;
}

int SkillsRoutine3(){
  aon::tests::turns();
  return 1;
}

#endif

} // namespace aon::routines

};  // namespace aon

