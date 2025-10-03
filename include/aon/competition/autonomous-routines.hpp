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

void turretRotationAbsolute(double targetAngle);
double widthToDistance(const double &width);
double groundDistanceToDisk(const double &pixels);

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

/// @brief Async task to align ORBIT only to the item with the globally set `COLOR` signature
void turretFollow(){
  const int TOLERANCE = 10;
  const int VISION_FIELD_CENTER = 315 / 2;
  int OBJ_CENTER;
  double position;
  
  while(true){
    if(turretFollowing){
      auto object = vision_sensor.get_by_sig(0, COLOR);
      OBJ_CENTER = object.x_middle_coord;
      double SPEED = turretPID.Output(0, VISION_FIELD_CENTER - OBJ_CENTER);
      position = turretEncoder.get_angle() / 100;

      if(object.signature == COLOR){
        if(abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE){
          turret.moveVelocity(0);
        }
        // Limiting to protect hardware
        else if (ORBIT_LIMITED && (ORBIT_LEFT_LIMIT >= position && position >= ORBIT_RIGHT_LIMIT)) {
          turretRotationAbsolute(nearest(position, std::make_pair(ORBIT_LEFT_LIMIT + 10, ORBIT_RIGHT_LIMIT - 10)));
        }
        else { // Turn Towards Object
          turret.moveVelocity(SPEED);
        }
      }
      // Dont move if nothing is there
      else {
        activateORBITScan();
      }
    }
    else if(turretBraking) {
      turret.moveVelocity(0);
    }
    pros::delay(10);
  }
  turret.moveVelocity(0);
}

/**
 * \brief Aligns ORBIT and DRIVETRAIN to the item with the set `COLOR`
 * 
 * \param color The color of the object to which we wish to align ourselves
 * 
 * \note Setting `color` to `STAKE` makes the robot turn 180° after alignment
 */
void alignRobotTo(const Colors &color = COLOR){
  COLOR = color;
  activateORBITFollow();
  pros::delay(500);
  const int TOLERANCE = 5;
  double difference;
  #define TURRET_ANGLE turretEncoder.get_angle() / 100
  while(abs(TURRET_ANGLE) > TOLERANCE){
    difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;
    double SPEED = turnPID.Output(0, -difference) * 400;
    drivetrain.rotate(SPEED);
    pros::delay(20);
  }
  #undef TURRET_ANGLE
  deactivateORBITFollow();
  drivetrain.stop();
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

/// @brief Rotates the ORBIT to a given angle, with respect to 0 degrees facing forward. (Absolute Rotation)
/// @param targetAngle Angle in degrees we wish to rotate ORBIT. within [-180, 180] or [0, 360]
/// @details `turretEncoder.get_angle()` is divided by 100 for scaling purposes.
inline void turretRotationAbsolute(double targetAngle) { 
  const double TOLERANCE = 5;
  if(targetAngle > 180) targetAngle -= 360;
  double currentAngle;
  do {
    currentAngle = turretEncoder.get_angle() / 100.0;
    if(currentAngle > 180) currentAngle -= 360;
    double output = turretPID.Output(targetAngle, currentAngle); 
    turret.moveVelocity(output); 
    pros::delay(10);
  } while(abs(currentAngle - targetAngle) > TOLERANCE);
  turret.moveVelocity(0);
}


/**
 * \brief Rotates the ORBIT a given angle, starting from wherever it currently is. (Relative Rotation)
 * 
 * \param givenAngle Angle in degrees we wish to rotate ORBIT.
 *
 * \details `turretEncoder.get_angle()` is divided by 100 for scaling purposes.
 */
inline void turretRotationRelative(const double &givenAngle) { 
  const double TOLERANCE = 5;
  double currentAngle;
  double initialAngle = turretEncoder.get_position() / 100.0; 
  double targetAngle = initialAngle + givenAngle; 
  do {
    currentAngle = turretEncoder.get_position() / 100.0;
    double output = turretPID.Output(targetAngle, currentAngle); 
    turret.moveVelocity(output); 
    pros::delay(10);
  } while(abs(currentAngle - targetAngle) > TOLERANCE);
  turret.moveVelocity(0);
}


/// @brief Calculates the distance to a ring of the specified `color` using a EKF
/// @param color The color of the ring we wish to track
/// @return The filtered distance to that ring
/// @note Takes half a second (0.5s) to complete
double distanceToRing(const Colors &color = COLOR){
  COLOR = color;
  okapi::EKFFilter ekf;

  double distance;

  // Filter the distance for half a second using 100 measurements (1 every 5 milliseconds)
  for(int i = 0; i < 100; i++){
    distance = ekf.filter(groundDistanceToDisk(vision_sensor.get_by_sig(0, color).width));
    pros::delay(5);
  }

  return distance;
}

/// @brief Drives forward until a ring hits the distance sensor
/// @param distance The distance from the robot to a ring
void driveTillPickUp(const double &distance = distanceToRing()){
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
void alignAndIntake(const Colors &color = COLOR){
  COLOR = color;
  drivetrain.move(12);
  alignRobotTo(COLOR);
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
  drivetrain.motors(0);
  intake.stopScan();
}

/// @brief Test function to see if the angle from the ORBIT makes sense
void testTurret(){
  while(true){
    double position = turretEncoder.get_angle() / 100.0;
    // const double turretTurn = aon::operator_control::AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY);
    const double turretTurn = 0.5;
    if (ORBIT_LIMITED && (ORBIT_LEFT_LIMIT >= position && position >= ORBIT_RIGHT_LIMIT)) {
      turret.moveVelocity(0);
      turretRotationAbsolute(0);
    }
    else {
      turret.moveVelocity(MAX_RPM * turretTurn * .1);
    }
    pros::lcd::print(1, "ORBIT Angle: %.2f", position);
    pros::delay(20);
  }
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
void driveIntoRing(const Colors &color = COLOR){
  COLOR = color;
  activateORBITFollow();
  intake.activateScan();
  pros::delay(500);
  okapi::EKFFilter ekf;
  const int TOLERANCE = 5; //? Probably adjust this
  double difference;
  #define TURRET_ANGLE turretEncoder.get_angle() / 100

  const double dt = 0.02;

  drivetrain.setMaxVelocity(MAX_RPM / 2);

  while(abs(TURRET_ANGLE) > TOLERANCE){
    auto object = vision_sensor.get_by_sig(0, color);

    // Safety to scan when object is lost
    if(object.signature != color) { 
      activateORBITScan();
      drivetrain.stop();
      continue;
    } 
    else {
      activateORBITFollow();
    }

    difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;
    //? maybe motion profile this variable
    double TURN = turnPID.Output(0, -difference) * 500;
    
    const double distance = ekf.filter(groundDistanceToDisk(object.width));

    double FORWARD = drivetrain.updateProfile(distance, dt);

    drivetrain.driveWhileTurning(FORWARD, TURN);
    pros::delay(20);
  }
  #undef TURRET_ANGLE
  #undef TIME
  deactivateORBITFollow();
  deactivateORBITScan();
  intake.stopScan();
  drivetrain.setMaxVelocity(MAX_RPM);
  driveTillPickUp();
}

/// @brief Calculates the distance the robot would have to travel to get to an object
/// @param pixels The pixels reported by the vision sensor viewing an object (preferably width of that object)
/// @return The distance in \b inches that the robot is from the object, probably to pass into the `move()` function
double groundDistanceToDisk(const double &pixels){
  const double distance = widthToDistance(pixels);
  if(distance < ORBIT_HEIGHT) { return distance; } // avoid √(-1) issues if the ring is detected to be bigger than it should be for some reason
  // pythagorean theorem: a^2 + b^2 = c^2
  // a = √(c^2 - b^2)
  return std::sqrt((distance * distance) - (ORBIT_HEIGHT * ORBIT_HEIGHT));
}

/// @brief Converts the amount of `pixels` seen from the vision sensor, to the corresponding \b inches
/// @param pixels The pixels reported by the vision sensor viewing an object (preferably width of that object)
/// @return The corresponding amount of \b inches
double pixelsToInches(const double &pixels){
  const double CONSTANT = 0.000208333; // found experimentally be measuring the distance from the vision sensor to the object and using algebra to tune the value until consistent/realistic results were returned
  return pixels * CONSTANT;
}

/// @brief Calculates the distance in \b inches of an object based on its width in \b pixels from the vision sensor
/// @param width The width of the object detected by the sensor in \b pixels
/// @return The distance from the vision sensor to the object in \b inches
/// @note This function assumes the entire object is in view, this may be changed later
/// @details The funcion uses `pixelsToInches()` as a crucial part of the calculations
/// @details The math is explained inside and the formulas are from optical geometry
double widthToDistance(const double &width){
  // m = -i/d
  // m = w_i / w_o
  // d = |i| * (w_o / w_i)
  // w_i = pixels * CONSTANT
  // since i, w_o and CONSTANT are constants
  // then the formula technically is:
  // d = K / pixels
  // where K is a constant K = |i| * (w_o / CONSTANT)
  const double REAL_WIDTH = 7;
  const double DISTANCE_OF_IMAGE = 0.0625; // estimated/experimental
  const double imageWidthInInches = pixelsToInches(width); // also somewhat estimated/experimental
  const double distance = DISTANCE_OF_IMAGE * (REAL_WIDTH / imageWidthInInches);
  return distance;
}

/// @brief Outputs and logs the width of a ring, and the distance to it based on that width
void testDistanceFromVision(){
  MovingAverage readingMav(50);
  MovingAverage avgMav(50);
  MovingAverage ekfMav(50);
  MovingAverage avg_ekfMav(50);
  okapi::EKFFilter ekf;
  activateORBITFollow();
  while(true){
    pros::vision_object ring = vision_sensor.get_by_sig(0, RED);
    if(ring.signature == RED){
      const double distance = groundDistanceToDisk(ring.width);
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
  deactivateORBITFollow();
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

/// @brief ORBIT async task scanning test function
void turretScan(){
  // To scan, make the ORBIT go from one side of its maximum FOV to the other,
  // if the ORBIT is not limited, make it go from 175° to 185° (going the long way)
  // if at any point the ORBIT detects an object, start following it and stop scanning
  bool goingLeft = true;
  while(true) {
    if(turretScanning && !turretFollowing){

      deactivateORBITFollow(); // redundant but ensures no fight for the vision sensor
      pros::vision_object object = vision_sensor.get_by_sig(0, COLOR);

      if(object.signature == COLOR){
        // stop scanning and start following if we find something
        activateORBITFollow();
      }
      else {
        double position = turretEncoder.get_angle() / 100;
        // scan if we find nothing
        // Limiting to protect hardware (even if the rotation is 360°, we dont want to twist the cable)
        if (ORBIT_LEFT_LIMIT >= position && position >= ORBIT_RIGHT_LIMIT) {
          goingLeft = !goingLeft;
          // Make the ORBIT go to the nearest limit and keep rotating from there
          turretRotationAbsolute(nearest(position, std::make_pair(ORBIT_LEFT_LIMIT + 20, ORBIT_RIGHT_LIMIT - 20)));
        }
        turret.moveVelocity(40 * (goingLeft ? -1 : 1));
      }
    }
    else if(turretFollowing) {
      deactivateORBITScan(); // dont scan if the ORBIT following was activated elsewhere for some reason
    } // an else would be redundant for our purposes
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
  alignRobotTo(COLOR);
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
  alignRobotTo(COLOR);
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
  driveIntoRing(COLOR);

  // then the one below that one
  drivetrain.goTo(1.2, -1.1);
  driveIntoRing(COLOR);

  // drive into the corner and try to grab the rings
  drivetrain.goTo(1.7, -1.7);
  RemoveTop();
  driveIntoRing(COLOR);
  drivetrain.turnTo(1.7, -1.7);
  RemoveTop();
  driveIntoRing(COLOR);

  drivetrain.move(-6);
  drivetrain.turnTo(1.8, 1.8);
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


