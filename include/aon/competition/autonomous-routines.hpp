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
// TODO: odometry should also have an traditional odometer functionality to track how much distance the robot has traveled and also use integration for all measurements as a fallbakc if sensors fail


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
 * \brief This small subroutine grabs a goal (stake)
 *
 * \param delay The amount of time in \b milliseconds you will be moving back (500-600 is quick and works)
 *
 * \warning You must already be very close to the goal and facing away (with the clamp towards it)
 *
 * \details This routine uses timing but ideally there would be a way of knowing when we have the goal within our grasp
 */
void grabGoal(const int &delay = 600){
  drivetrain.motors(-100);
  pros::delay(delay * 5 / 6);
  claw.set_value(true);
  pros::delay(delay * 1/6);
  drivetrain.motors(100);
  pros::delay(delay);
  drivetrain.stop();
}

/**
 * \brief This subroutine moves toward a mobile goal IN REVERSE
 *
 * \param dist This is the absolute value of the distance the mobile goal is from the robot in \b inches
 *
 * \details The function already converts the distance to negative so the robot drives into the goal backwards
 *
 */
void raceToGoal(const double &dist = 47){
  drivetrain.move(-abs(dist));
  grabGoal(300);
}

/// @brief Drops the goal by releasing the claw
void dropGoal(){
  claw.set_value(false);
}

/// @brief Extends or retracts indexer to later knock down rings
/// @param extend If true, indexer will extend, if false, it will retract
void moveIndexer(const bool &extend = true){
  indexer.set_value((extend ? 1 : 0) );
}

/// @brief This small subroutine removes the top ring of a stack of two and scores the ring at top. use ONLY when the indexer is at the right side of stack.
void RemoveTop(){
  moveIndexer();
  drivetrain.turn(-45);
  moveIndexer(false);
}

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

/// @brief Aligns front of robot and turns around to grab the stake
/// @param dist The absolute value of the distance that the robot is from the stake when it begins alignment in \b inches
void findAndGrabGoal(const double &dist = 8){
  alignRobotTo(STAKE);
  drivetrain.move(-abs(dist));
  grabGoal();
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

/// @brief Get a stake and scores a preload
void grabAndScore(){
  findAndGrabGoal(10);
  intake.score();
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

/// @brief Competition specific test to see if the indexer interacted as intended
void testIndexer(){
  moveIndexer();
  drivetrain.move(40);
  drivetrain.move(-6);
  drivetrain.turn(180);
  drivetrain.move(-2);
  grabGoal();
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
    
    const double distance = ekf.filter((orbit.getLargestObject()).width);

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
    pros::vision_object ring = orbit.getLargestObject();
    if(ring.signature == RED){
      const double distance = orbit.groundDistanceToDisk(ring.width);
      if(!std::isnormal(distance)) { continue; }
      const double avg = readingMav.update(distance);
      const double filtered = ekf.filter(distance); // this seems to be the best alternative out of the 2
      const double avgDif = avgMav.update(math::getPercentDifference(avg, distance));
      const double ekfDif = ekfMav.update(math::getPercentDifference(filtered, distance));
      const double avg_ekfDif = avg_ekfMav.update(math::getPercentDifference(avg, filtered));
      pros::lcd::print(0, "Ring width = %d", ring.width);
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
  okapi::EKFFilter ekf(2.6E-4, 0.04);
  while(true){
    const double pos = odometry::gyroscope.get_heading();
    pros::lcd::print(0, "Heading = %.2f", pos);
    pros::lcd::print(1, "Filtered = %.2f", ekf.filter(pos)); // this one is slower which might mean i want to tweak the values for the ekf
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

// ============================================================================|
//   ___  ___  _   _ _____ ___ _  _ ___ ___                                    |
//  | _ \/ _ \| | | |_   _|_ _| \| | __/ __|                                   |
//  |   / (_) | |_| | | |  | || .` | _|\__ \                                   |
//  |_|_\\___/ \___/  |_| |___|_|\_|___|___/                                   |
//                                                                             |
// ============================================================================|

/// @brief This is a safety routine to at least grab one goal and score on it
void quickMiddleScore(){
  drivetrain.move(-3);
  grabGoal();
  intake.score();
  drivetrain.move(10);
}

#if USING_BLACK_ROBOT

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
  raceToGoal();
  drivetrain.move(6);
  intake.score();
  intake.openGate();

  // Get the next ring in our side
  drivetrain.turnTo(-.6, -1.2);
  drivetrain.move(6);
  RemoveTop();
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
  raceToGoal();
  drivetrain.move(6);
  intake.score();
  intake.openGate();
  
  // Get the next ring in our side
  drivetrain.turnTo(.6, 1.2);
  drivetrain.move(6);
  RemoveTop();
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
  findAndGrabGoal(8); // 8 inches from stake
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
  findAndGrabGoal(6); //6 inches from stake
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
  raceToGoal();
  drivetrain.move(6);
  intake.score(2000);
  intake.openGate();

  // go to ring on the bottom
  drivetrain.goTo(1.2, -0.55);
  RemoveTop();
  driveIntoRing(orbit.getColor());

  // then the one below that one
  drivetrain.goTo(1.2, -1.1);
  driveIntoRing(orbit.getColor());

  // drive into the corner and try to grab the rings
  drivetrain.goTo(1.7, -1.7);
  RemoveTop();
  driveIntoRing(orbit.getColor());
  drivetrain.turnTo(.7, -1.7);
  RemoveTop();
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
  raceToGoal(TILE_DIAG_LENGTH);
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
  raceToGoal(TILE_DIAG_LENGTH);
  
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

int greenBotRedSide(){
  drivetrain.move(-6);
  grabGoal();
  intake.score();
  enableGate();

  drivetrain.turnTo(-1.2,1.2);
  moveTilesStraight(1.3);
  driveTillPickUp();

  drivetrain.turnTo(-0.6,1.2);
  moveTilesStraight(1);
  driveTillPickUp();

  drivetrain.turnTo(-1.5,0);
  moveTilesStraight(1.3);
  driveTillPickUp();
}

void simple_Auto_Red(){
    drivetrain.move(-.5);
    grabGoal();
    intake.score();
    enableGate();
  
    drivetrain.turnTo(-1.2,1.2);
    moveTilesStraight(1.3);
    driveIntoRing(COLOR);// change
  
    drivetrain.turnTo(-0.6,1.2);
    moveTilesStraight(1);
    driveIntoRing(COLOR);//change
  
    drivetrain.turnTo(-1.5,0);
    moveTilesStraight(1.3);
    driveIntoRing(COLOR);//change
  
}

void Auto_with_indexer(){
  moveTilesStraight(1.5);
  moveIndexer();
  moveTilesStraight(-.5);
  drivetrain.turn(180);
  grabGoal();
  intake.score();
  enableGate();

  drivetrain.turnTo(-0.6,1.2);
  moveTilesStraight(1);
  driveIntoRing(COLOR); //channge 

  drivetrain.turnTo(-1.2,1.2);
  moveTilesStraight(1.3);
  driveIntoRing(COLOR);// change

  drivetrain.turnTo(-1.5,0);
  moveTilesStraight(1.3);
  driveIntoRing(COLOR);//change


}


/**
 * \author Solimar
 */
int SkillsGreenBotSoli(){
  //First Grab Nearest stake 
  drivetrain.move(-1.3);
  grabGoal();
  
  //Then attempt to grab the red rings towards the corner 
  drivetrain.turnTo(-1.8, 1.8);
  driveIntoRing(COLOR);
  driveIntoRing(COLOR);
  
  //turn towards remaining going up
  //turn towards red on the line
  //drop steak
  drivetrain.goTo(-1.5, -0.3);
  drivetrain.turnTo(-1.2, 0.0);
  FollowWithTurret(RED);
  dropGoal();
  
  //pick up second steak 
  drivetrain.turnTo(0.6, 0.6);
  drivetrain.turn(180);
  moveTilesDiag(-1);
  grabGoal();
  //pick up reds around whilst also tumbando los stacks red-blue
  drivetrain.turnTo(1.8, 1.8);
  driveIntoRing(COLOR);
  driveIntoRing(COLOR);
  
  //turn towards remaining going up
  //turn towards red on the line
  //drop steak
  drivetrain.goTo(1.5, -0.3);
  drivetrain.turnTo(-1.2, 0.0);
  FollowWithTurret(RED);
  dropGoal();
}

/**
 * \author Jorge G
 */
int SkillsGreenBotJorge(){
  raceToGoal(50); //preg a kev
  drivetrain.turnTo(0, -1.2);
  drivetrain.goTo(0, -1.2);
  FollowWithTurret(RED);
  //First ring ^
  drivetrain.turnTo(-0.6, 0.6);
  drivetrain.goTo(-0.6, 0.6);
  FollowWithTurret(RED);
  // Second ring ^
  drivetrain.turnTo(-1.2, 1.2);
  drivetrain.goTo(-1.2, 1.2);
  FollowWithTurret(RED);
  // Third Ring ^
  drivetrain.turnTo(-1.8, 1.8);
  drivetrain.goTo(-1.8, 1.8);
  FollowWithTurret(RED); 
  // Fourth Ring ^
  drivetrain.turnTo(0, 1.2);
  drivetrain.goTo(0, 1.2);
  FollowWithTurret(RED); 
  // Fifth Ring ^
  dropGoal();
  // release stake
  drivetrain.turnTo(0.6, 0.6);
  drivetrain.move(12); // measure
  findAndGrabGoal(10); //measure
  // grab stake at (0.6, 0.6)
  drivetrain.turnTo(0, 1.5);
  drivetrain.goTo(0, 1.5);
  FollowWithTurret(RED); 
  // 2nd stake first ring
  drivetrain.turnTo(0.6, 1.2);
  drivetrain.goTo(0.6, 1.2);
  FollowWithTurret(RED);   
  // Second Ring ^
  drivetrain.turnTo(1.2, 1.2);
  drivetrain.goTo(1.2, 1.2);
  FollowWithTurret(RED);  
  // Third Ring ^
  drivetrain.turnTo(1.2, 0.6);
  drivetrain.goTo(1.2, 0.6);
  FollowWithTurret(RED);  
  // Fourth Ring ^
  drivetrain.turnTo(1.8, 1.8);
  drivetrain.goTo(1.8, 1.8);
  FollowWithTurret(RED); 
  // Fifth Ring ^
  drivetrain.move(-3);
  FollowWithTurret(BLUE);
  dropGoal();
}
#endif

};  // namespace aon

