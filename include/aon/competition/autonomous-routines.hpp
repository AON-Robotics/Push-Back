#pragma once

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../globals.hpp"
#include "../sensing/odometry.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../tools/logging.hpp"
#include "../tools/moving-average.hpp"
#include "../tools/general.hpp"
#include "../math/misc/misc.hpp"

// TODO: for modularity we will have odometry, drivetrain, navigator, orbit, intake, and claw (the last two will most likely change with each game and modules may be added or removed as needed)
//# Navigator will use odometry and drivetrain under the hood for auton, but drivers will use just drivetrain for driving
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
  #undef TURRET_ANGLE
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


// ============================================================================
//   _____ ___ ___ _____ ___
//  |_   _| __/ __|_   _/ __|
//    | | | _|\__ \ | | \__ \
//    |_| |___|___/ |_| |___/
//
// ============================================================================

/// @brief Basic Routine to make the robot go in circles around the map to test GPS setup.
void testGPS() {
  drivetrain.goTo(.6, -1.2);
  drivetrain.goTo(1.2, -.6);
  drivetrain.goTo(1.2, .6);
  drivetrain.goTo(.6, 1.2);
  drivetrain.goTo(-.6, 1.2);
  drivetrain.goTo(-1.2, .6);
  drivetrain.goTo(-1.2, -.6);
  drivetrain.goTo(-.6, -1.2);
  drivetrain.goTo(.6, -1.2);
  drivetrain.goTo(1.2, -.6);
}

/// @brief  Speed calculation test using the distance sensor
/// @param RPM The velocity for the motors
void testSpeed(double RPM = MAX_RPM){
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
void testOdom(){
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
void testConcurrency(){
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
void testAlignment(){
  while(true){
    alignRobotTo(RED);
    pros::delay(20);
  }
}

/// @brief Checks the output of an optical shaft encoder
void testADIEncoder(){
  pros::lcd::print(1, "Encoder value: %d", opticalEncoder.get_value());
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
  #define TURRET_ANGLE turretEncoder.get_angle() / 100

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

    drivetrain.driveWhileTurning(FORWARD, TURN);
    pros::delay(20);
  }
  #undef TURRET_ANGLE
  #undef TIME
  orbit.deactivateFollow();
  orbit.deactivateScan();
  intake.stopScan();
  drivetrain.setMaxVelocity(MAX_RPM);
  driveTillPickUp();
}

/// @brief Outputs and logs the width of a ring, and the distance to it based on that width
void testDistanceFromVision(){
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
      if(!std::isnormal(distance)) { continue; }
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
void testEKFWithGyro(){
  okapi::EKFFilter ekf1;
  okapi::EKFFilter ekf2(2.6E-4, 0.04);
  okapi::EKFFilter ekf3(3E-4, 0.04);
  okapi::EKFFilter ekf4(4E-4, 0.04);
  okapi::EKFFilter ekf5(5E-4, 0.04);
  while(true){
    const double pos = odometry::gyroscope.get_heading();
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
int testAdjustable(){
  drivetrain.driveInArcTo(math::inchesToMeters(TILE_WIDTH / 2), math::inchesToMeters(TILE_WIDTH / 2));
  return 1;
}

/// @brief Function wrapper for test functions that are to be executed through the GUI
/// @return 1 for successful execution
/// @note Choose between 3 tests depending on the result of `potentiometer.get_value()`
int testMultiple(){
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

void testTurns(){
  for (int i = 0; i < 4; i++){drivetrain.turn(); pros::delay(750);}
  for (int i = 0; i < 4; i++){drivetrain.turn(-90); pros::delay(750);}
}

void testSquare(){
  for (int i = 0; i < 4; i++){
    drivetrain.move();
    pros::delay(750);
    drivetrain.turn();
    pros::delay(750);
  }
}

#if USING_BIG_ROBOT

/// @brief Starting position is the left side of the parking facing towards the drive team, placed paralele to the side of the parking with the second shaft of the drivetrain aligned with the end of the goal
void safeBigBotRoutine(){
  intake.activateScan();
  drivetrain.strafe(28.5); // Align with match loader.
  intake.dropShrimp(); // Prepare loader mechanism.
  drivetrain.move(6); // Move to match loader.
  drivetrain.motors(MAX_RPM / 2); // Push into loader
  pros::delay(200); // for a bit of time,
  drivetrain.stop(); // then stop.
  pros::delay(8000); // Take up all the blocks (9).
  drivetrain.move(-22); // Move to long goal.
  drivetrain.motors(-MAX_RPM / 2); // Push into goal
  pros::delay(200); // for a bit of time,
  drivetrain.stop(); // then stop.
  intake.raiseShrimp(); // Reset loader mechanism.
  intake.score(Intake::TOP, Intake::BOTTOM, 8000); // Score all 9 blocks.
  drivetrain.move(15); // Go back a little.
  drivetrain.turn(-90); // Orient towards parking.
  drivetrain.move(12); // Move towards parking.
  drivetrain.strafe(13); // Align with parking.
  drivetrain.move(12); // Move to parking.
  drivetrain.motors(MAX_RPM); // Push into parking to put a row of wheels over
  pros::delay(1000); // for a bit of time,
  drivetrain.stop(); // then stop.
  brooksPiston.set_value(HIGH); // Park.
  intake.stopScan();
  //* Works till here
}

void BigBotSkillsRoutine(){
  drivetrain.strafe(28.5);
  intake.dropShrimp(); 
  drivetrain.move(6); 
  drivetrain.motors(MAX_RPM / 2); 
  pros::delay(200); 
  drivetrain.stop(); 
  pros::delay(8000); 
  drivetrain.move(-22); 
  drivetrain.motors(-MAX_RPM / 2);
  drivetrain.move(22);
  drivetrain.turn(180);
  drivetrain.strafe(-10.5);
  drivetrain.move(50);
  drivetrain.strafe(10.5);
  intake.dropShrimp(); 
  drivetrain.move(6); 
  drivetrain.motors(MAX_RPM / 2); 
  pros::delay(200); 
  drivetrain.stop(); 
  pros::delay(8000); 
  drivetrain.strafe(10); 
  drivetrain.turn(90);
  drivetrain.motors(MAX_RPM);
  pros::delay(1000); 
  drivetrain.stop(); 
  brooksPiston.set_value(HIGH); 
  //empy the match loader 
  //empty match loader across 
  //Park
}


#else

void testSmallBotRoutine(){
  drivetrain.move(31); // Align with match loader
  drivetrain.turn(86);
  intake.dropCart(); // Prepare loader mechanism
  pros::delay(200);  
  drivetrain.move(4); // Go to match loader
  intake.activateScan(); 
  drivetrain.motors(-MAX_RPM / 2); // Push into loader
  pros::delay(100); // for a bit of time,
  drivetrain.motors(MAX_RPM / 2); // Push into loader
  pros::delay(400); // for a bit of time,
  drivetrain.stop(); // then stop.
  pros::delay(5000); // Take up some blocks (6);
  intake.stopScan();
  drivetrain.move(-13); // Move to Long goal
  intake.raiseCart(); // Reset loader mechanism
  drivetrain.turn(85);
  drivetrain.turn(85);
  intake.setScorerHeight(HIGH);
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

void testXDriveRoutine(){
  // drivetrain.goToPose(Pose(-TILE_WIDTH, 0, 0));
  // drivetrain.goToPose(Pose(0, 12, 0));
  // drivetrain.goToPose(Pose(-12, 12, 0));
  // drivetrain.goToPose(Pose(0, 0, 90));
  // drivetrain.goToPose(Pose(-12, 18, 0));
  // drivetrain.goToPose(Pose(-TILE_WIDTH, TILE_WIDTH, 90));
  // drivetrain.goToPose(Pose(0, 0, 0));
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
  intake.setScorerHeight(HIGH);
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

// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

#if USING_BIG_ROBOT

/**
 * \brief This routine is if WE ARE RED and want to grab RED RINGS
 *
 * \note Designed for being in the third quadrant
 * \note Starting Position (-0.34, -0.82) \b m facing towards 296.86 \b deg
 *
 * \author Kevin Gomez
*/
int RedRingsRoutine(){
  // Secure and score the first ring in the middle stake
  drivetrain.move(6);
  intake.score();
  // intake.openGate();

  // Get the next ring in our side
  drivetrain.turnTo(-.6, -1.2);
  drivetrain.move(6);
  drivetrain.move(-6);
  alignRobotTo(orbit.getColor());
  driveTillPickUp();

  // Get the last ring in that line
  drivetrain.move(-6);
  drivetrain.turnTo(-1.2, -1.2);
  drivetrain.move(6);
  driveTillPickUp();

  // Bring down the 4 stack
 
  return 1;
}

/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \note Designed for being in the first quadrant
 * \note Starting Position (0.34, 0.82) \b m facing towards 116.86 \b deg
 *
 * \author Kevin Gomez
 */
int BlueRingsRoutine(){
  // Secure and score the first ring in the middle stake
  drivetrain.move(6);
  intake.score();
  // intake.openGate();
  
  // Get the next ring in our side
  drivetrain.turnTo(.6, 1.2);
  drivetrain.move(6);
  drivetrain.move(-6);
  alignRobotTo(orbit.getColor());
  driveTillPickUp();
  
  // Get the last ring in that line
  drivetrain.move(-6);
  drivetrain.turnTo(1.2, 1.2);
  drivetrain.move(6);
  driveTillPickUp();
  
  // Bring down the 4 stack
  
  return 1;
}


/**
  WILL CLEAR POSITIVE SIDE JUST TO BE SURE

  LOOKING AT THE POSITIVE SIDE OF OUR SIDE

  TRY TO PUT IT 4 INCHES AWAY AS BEST AS POSSIBLE
  */

int BlueRingsRoutine_JorgeGuz(){
  // Go into the esquina
  drivetrain.move(4);
  intake.pickUp(1500);
  drivetrain.move(-4);
  drivetrain.move(4);
  intake.pickUp(1500);
  drivetrain.move(-4);
  drivetrain.move(4);
  intake.pickUp(1500);
  drivetrain.move(-4);
  
  return 0;
}

/**
  Will take the closest to us, in our side and take th rings that are below

  START IN THE POSITIVE SIDE OF THE FIELD, LOOKING AT THE STAKE WHEN WE ARE RED.

  RED-NEGATIVE SIDE
*/
int safeRingRoutine() {
  drivetrain.turnTo(-1.2,-1.2);
  drivetrain.goTo(-1.2,-1.2);
  driveIntoRing(RED);
  drivetrain.turnTo(-1.5,0);
  drivetrain.goTo(-1.5,0);
  driveIntoRing(RED);
  drivetrain.turnTo(1.2,-1.2);
  drivetrain.goTo(1.2,-1.2);
  driveIntoRing(RED);
  return 0;
}

int safeRingRoutine2() {
  drivetrain.turnTo(-1.2,-1.2);
  drivetrain.goTo(-1.2,-1.2);
  driveIntoRing(RED);
  drivetrain.turnTo(-1.5,0);
  drivetrain.goTo(-1.5,0);
  driveIntoRing(RED);
  drivetrain.turnTo(-1.2,1.2);
  drivetrain.goTo(-1.2,1.2);
  driveIntoRing(RED);
  return 0;
}





/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \author Jorge Luis
*/    

int BlueRingsRoutineJorgeLuna() {
  /*
    go for negative side mobile goal, score rings, and prepare for go enemy double side
  */
  // go to the side mobile goal
  drivetrain.move(6);
  // intake.score(2000);
  // intake.openGate();

  // go to ring on the bottom
  drivetrain.goTo(1.2, -0.55);
  driveIntoRing(orbit.getColor());

  // then the one below that one
  drivetrain.goTo(1.2, -1.1);
  driveIntoRing(orbit.getColor());

  // drive into the corner and try to grab the rings
  drivetrain.goTo(1.7, -1.7);
  driveIntoRing(orbit.getColor());
  drivetrain.turnTo(.7, -1.7);
  driveIntoRing(orbit.getColor());

  drivetrain.move(-6);
  drivetrain.turnTo(1.8, 1.8);
  return 0;
}

/**
 * \author Jorge L
 */
int SkillsBlackBotJorge(){
  // grab skate in the middle bottom
  // grab 0, -1.2
  // grab 0.6, -1.2
  // turn to -1.4, 0
  // grab -1.4, 0
  // turn to -0.6, -0.6
  // go closer
  // grab -0.6, -0.6
  // turn to -1.2, -1.2
  // grab it
  // grab -1.8, -1.8
  // turn 180 
  // let stake at the esquina
  
  // go to 0.6, -0.6
  // grab stake
  // grab rings in the middle
  // let stake
  // go to -.6, -0.6
  // grab stake most right
  // take ring 1.5, 0
  // take ring 1.2, -0.6
  // take ring 0.6, -1.2
  // take ring 1.2, -1.2
  // take ring -1.8, 1.8
  // if we suppose all the red rings are as points
    // take blue ring 1.8, -1.8
  // put stake in 1.8, -1.8
  return 0;
}

/**
 * \author Kevin
 * 
 * \note Starts with claw in (-1.2, -6) facing bottom-most goal
 */
int SkillsBlackBotKevin(){
  // Grab bottom-most goal
  
  // Grab ring in (-1.2, -1.2)
  drivetrain.turnTo(-1.2, -1.2);
  driveIntoRing();
  drivetrain.move(-12);

  // Grab ring in (-1.8, -1.8)
  drivetrain.turnTo(-1.8, -1.8);
  drivetrain.move(6);
  driveIntoRing();
  drivetrain.move(-12);

  // Grab ring in (-.6, -.6)
  drivetrain.goTo(-.9, -.9);
  driveIntoRing();
  drivetrain.move(-24);

  // Grab ring in (0, -1.2)
  drivetrain.turnTo(0, -1.2);
  
  return 0;
}


#else

/**
 * \brief This routine is if WE ARE RED and want to grab RED RINGS
 *
 * \note Designed for being in the third quadrant
 * \note Starting Position (-0.34, -0.82) \b m facing towards 296.86 \b deg
 *
 * \author Kevin Gomez
*/
int RedRingsRoutine(){
  // Secure and score the first ring in the middle stake
  drivetrain.move(6);
  intake.score();
  // intake.openGate();

  // Get the next ring in our side
  drivetrain.turnTo(-.6, -1.2);
  drivetrain.move(6);
  drivetrain.move(-6);
  alignRobotTo(orbit.getColor());
  driveTillPickUp();

  // Get the last ring in that line
  drivetrain.move(-6);
  drivetrain.turnTo(-1.2, -1.2);
  drivetrain.move(6);
  driveTillPickUp();

  // Bring down the 4 stack
 
  return 1;
}

/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \note Designed for being in the first quadrant
 * \note Starting Position (0.34, 0.82) \b m facing towards 116.86 \b deg
 *
 * \author Kevin Gomez
 */
int BlueRingsRoutine(){
  // Secure and score the first ring in the middle stake
  drivetrain.move(6);
  intake.score();
  // intake.openGate();
  
  // Get the next ring in our side
  drivetrain.turnTo(.6, 1.2);
  drivetrain.move(6);
  drivetrain.move(-6);
  alignRobotTo(orbit.getColor());
  driveTillPickUp();
  
  // Get the last ring in that line
  drivetrain.move(-6);
  drivetrain.turnTo(1.2, 1.2);
  drivetrain.move(6);
  driveTillPickUp();
  
  // Bring down the 4 stack
  
  return 1;
}


/**
  WILL CLEAR POSITIVE SIDE JUST TO BE SURE

  LOOKING AT THE POSITIVE SIDE OF OUR SIDE

  TRY TO PUT IT 4 INCHES AWAY AS BEST AS POSSIBLE
  */

int BlueRingsRoutine_JorgeGuz(){
  // Go into the esquina
  drivetrain.move(4);
  intake.pickUp(1500);
  drivetrain.move(-4);
  drivetrain.move(4);
  intake.pickUp(1500);
  drivetrain.move(-4);
  drivetrain.move(4);
  intake.pickUp(1500);
  drivetrain.move(-4);
  
  return 0;
}

/**
  Will take the closest to us, in our side and take th rings that are below

  START IN THE POSITIVE SIDE OF THE FIELD, LOOKING AT THE STAKE WHEN WE ARE RED.

  RED-NEGATIVE SIDE
*/
int safeRingRoutine() {
  drivetrain.turnTo(-1.2,-1.2);
  drivetrain.goTo(-1.2,-1.2);
  driveIntoRing(RED);
  drivetrain.turnTo(-1.5,0);
  drivetrain.goTo(-1.5,0);
  driveIntoRing(RED);
  drivetrain.turnTo(1.2,-1.2);
  drivetrain.goTo(1.2,-1.2);
  driveIntoRing(RED);
  return 0;
}

int safeRingRoutine2() {
  drivetrain.turnTo(-1.2,-1.2);
  drivetrain.goTo(-1.2,-1.2);
  driveIntoRing(RED);
  drivetrain.turnTo(-1.5,0);
  drivetrain.goTo(-1.5,0);
  driveIntoRing(RED);
  drivetrain.turnTo(-1.2,1.2);
  drivetrain.goTo(-1.2,1.2);
  driveIntoRing(RED);
  return 0;
}





/**
 * \brief This routine is if WE ARE BLUE and want to grab BLUE RINGS
 *
 * \author Jorge Luis
*/    

int BlueRingsRoutineJorgeLuna() {
  /*
    go for negative side mobile goal, score rings, and prepare for go enemy double side
  */
  // go to the side mobile goal
  drivetrain.move(6);
  // intake.score(2000);
  // intake.openGate();

  // go to ring on the bottom
  drivetrain.goTo(1.2, -0.55);
  driveIntoRing(orbit.getColor());

  // then the one below that one
  drivetrain.goTo(1.2, -1.1);
  driveIntoRing(orbit.getColor());

  // drive into the corner and try to grab the rings
  drivetrain.goTo(1.7, -1.7);
  driveIntoRing(orbit.getColor());
  drivetrain.turnTo(1.7, -1.7);
  driveIntoRing(orbit.getColor());

  drivetrain.move(-6);
  drivetrain.turnTo(1.8, 1.8);
  return 0;
}

/**
 * \author Jorge L
 */
int SkillsBlackBotJorge(){
  // grab skate in the middle bottom
  // grab 0, -1.2
  // grab 0.6, -1.2
  // turn to -1.4, 0
  // grab -1.4, 0
  // turn to -0.6, -0.6
  // go closer
  // grab -0.6, -0.6
  // turn to -1.2, -1.2
  // grab it
  // grab -1.8, -1.8
  // turn 180 
  // let stake at the esquina
  
  // go to 0.6, -0.6
  // grab stake
  // grab rings in the middle
  // let stake
  // go to -.6, -0.6
  // grab stake most right
  // take ring 1.5, 0
  // take ring 1.2, -0.6
  // take ring 0.6, -1.2
  // take ring 1.2, -1.2
  // take ring -1.8, 1.8
  // if we suppose all the red rings are as points
    // take blue ring 1.8, -1.8
  // put stake in 1.8, -1.8
  return 0;
}

/**
 * \author Kevin
 * 
 * \note Starts with claw in (-1.2, -6) facing bottom-most goal
 */
int SkillsBlackBotKevin(){
  // Grab bottom-most goal
  
  // Grab ring in (-1.2, -1.2)
  drivetrain.turnTo(-1.2, -1.2);
  driveIntoRing();
  drivetrain.move(-12);

  // Grab ring in (-1.8, -1.8)
  drivetrain.turnTo(-1.8, -1.8);
  drivetrain.move(6);
  driveIntoRing();
  drivetrain.move(-12);

  // Grab ring in (-.6, -.6)
  drivetrain.goTo(-.9, -.9);
  driveIntoRing();
  drivetrain.move(-24);

  // Grab ring in (0, -1.2)
  drivetrain.turnTo(0, -1.2);
  
  return 0;
}

#endif

};  // namespace aon

