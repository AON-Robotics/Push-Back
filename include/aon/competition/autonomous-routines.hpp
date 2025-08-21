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

// TODO: for modularity we will have odometry, drivetrain, navigator, orbit, intake, and claw (the last two will most likely change with each game and modules may be added or removed as needed)
//# Navigator will use odometry and drivetrain under the hood for auton, but drivers will use just drivetrain for driving
// TODO: add support for a drive mode that is videogame-like (i think rocket league has it). Basically with reference to where the driver is standing on the field, the direction towards which you move the joystick is where the robot will turn to and drive to at the same time. This should greatly facilitate general directional movement if implemented correctly. Leave a toggle available for traditional driving in accordance to the chosen drivetrain for better fine grained control in tight spaces.
// TODO: odometry should also have an traditional odometer functionality to track how much distance the robot has traveled and also use integration for all measurements as a fallbakc if sensors fail


/**
 * For GPS coord system: https://pros.cs.purdue.edu/v5/tutorials/topical/gps.html
 */

namespace aon {

int move(const double &dist);
int turn(const double &angle);
void grabGoal(const int &delay);
void raceToGoal(const double &dist);
void pickUpRing(const int &delay);
void scoreRing(const int &delay);
inline double metersToInches(const double &meters);
void discardDisk();
void dropGoal();
void moveIndexer(const bool &extend);
void enableGate();
void turretRotationAbsolute(double targetAngle);
double widthToDistance(const double &width);
double groundDistanceToDisk(const double &pixels);


// ============================================================================
//    ___   _   _    ___ _   _ _      _ _____ ___ ___  _  _ ___  
//   / __| /_\ | |  / __| | | | |    /_\_   _|_ _/ _ \| \| / __|
//  | (__ / _ \| |_| (__| |_| | |__ / _ \| |  | | (_) | .` \__ \
//   \___/_/ \_\____\___|\___/|____/_/ \_\_| |___\___/|_|\_|___/
//
// ============================================================================

/**
 * \brief Determines the speed of the robot given drivetrain motors' RPM
 *
 * \param RPM The RPM for which to calculate the velocity (default current RPM)
 *
 * \returns The speed in \b in/s at which the robot would move at the given RPM
 *
 * \note Test the accuracy precision of the `getActualVelocity()` method,
 * \note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
 * \note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`
 */
inline double getSpeed(const double &RPM = (int)driveFull.getActualVelocity()){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double RPS = RPM / 60;
  double speed = MOTOR_TO_DRIVE_RATIO * circumference * RPS;
  return speed;
}

/**
 * \brief Calculates time for the robot to reach a given distance
 *
 * \param distance Distance from the robot to the target (remains constant) in \b inches
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 */
inline double getTimetoTarget(const double &distance, const double &RPM = MAX_RPM){
  double time = 4 * distance / getSpeed(RPM);
  return time;
}


/**
 * \brief Calculates time for the robot to turn an angle
 *
 * \param radians Angle remaining from the robot's current angle to the target (remains constant) in \b radians
 *
 * \details The arc length formula is used as s = theta * radius (theta in radians)
 *
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 *
 */
inline double getTimetoTurnRad(const double &radians, const double &RPM = MAX_RPM / 4){
  double arcLength = radians * AVG_DRIVETRAIN_RADIUS; // Of the turn (inches)
  double time = 2 * arcLength / getSpeed(RPM); // Calculated time (seconds)
  return time;
}

/**
 * \brief Calculates time for the robot to turn an angle
 *
 * \param degrees Angle remaining from the robot's current angle to the target (remains constant) in \b degrees
 *
 * \returns The approximate time necessary to reach the target (overestimation) in \b seconds
 *
 */
inline double getTimetoTurnDeg(const double &degrees) { return getTimetoTurnRad(degrees * M_PI / 180); }

/**
 * \brief Conversion from \b meters to \b inches
 *
 * \param meters The \b meters to be converted
 *
 * \returns The distance in \b inches
 */
inline double metersToInches(const double &meters) { return meters * 39.3701; }

/**
 * \brief Conversion from \b inches to \b meters
 *
 * \param inches The \b inches to be converted
 *
 * \returns The distance in \b meters
 */
inline double inchesToMeters(const double &inches) { return inches / 39.3701; }

/**
 * \brief Gets the distance between two points in the field
 *
 * \param target The target location
 * \param current The current location
 *
 * \returns The distance between the two points
 */
double findDistance(Vector target, Vector current){
  double distInMeters = (target - current).GetMagnitude();
  return metersToInches(distInMeters);
}

/**
 * \brief Determines the angle needed to be turned in order to face a specific point in the field
 *
 * \param target The point we wish to face
 * \param current Where the robot is now
 *
 * \returns The angle the robot needs to turn in order to face the target location
 *
 * \note The result must be passed into functions such as `turn()` and `MoveTurnPID()` as negative because of the GPS convention
 */
double calculateTurn(Vector target, Vector current) {
  // Get and change the heading to the common cartesian plane
  double heading = 90 - gps.get_heading();

  // Limiting the heading to the 0-360 range
  if (heading < 0) heading += 360;
  else if (heading > 360) heading -= 360;
 
  // This number is in respect to the common cartesian plane if odometry position is used
  double toTarget = (target - current).GetDegrees();
 
  // Limiting the the target to the 0-360 range
  if (toTarget < 0) toTarget += 360;
  else if (toTarget >= 360) toTarget -= 360;

  double angle = toTarget - heading; // Calculate the angle to turn
 
  // Limiting the heading to the -180-180 range
  if (angle > 180) angle -= 360;
  else if (angle < -180) angle += 360;

  return angle;
}

/**
 * \brief Calculates the error percentage for an actual given the expected value
 * 
 * \param expected Usually a calculated value
 * \param actual Usually the measured value
 * 
 * \returns The error percentage for the measured value
 */
double getErrorPercentage(const double &expected, const double &actual) {
  return ((expected - actual) / expected) * 100;
}

/// @brief Calculate percent difference of two values
/// @param a Value of one number
/// @param b Valuer of the other number
/// @return The percent difference between them
double getPercentDifference(const double &a, const double &b) {
  return (std::abs(a - b) / ((a + b) / 2)) * 100;
}


// ============================================================================
//   __  __  _____   _____ __  __ ___ _  _ _____
//  |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================

/**
 * \brief Moves the robot a given distance (default forward)
 *
 * \param pid The PID used for the driving
 * \param dist The distance to be moved in \b inches
 * \param MAX_REVS The maximum RPM to send to the movement
 */
void MoveDrivePID(PID pid = drivePID, double dist = TILE_WIDTH, const double &MAX_REVS = 100.0) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive
  pid.Reset();

  aon::Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = getTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time //every time the variable is called it is recalculated automatically

  // while (time < timeLimit) {
  while((aon::odometry::GetPosition() - initialPos).GetMagnitude() < dist){

    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();

    double output = pid.Output(dist, currentDisplacement);

    pros::lcd::print(0, "Time Limit %.2f", timeLimit);
    pros::lcd::print(1, "Time: %.2f", time);
    pros::lcd::print(2, "Odometry Displacement %.2f", currentDisplacement);

    driveFull.moveVelocity(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));

    pros::delay(10);
  }

  // Stop the motors
  driveFull.moveVelocity(0);

  #undef time
}

/**
 * \brief Turns the robot by a given angle (default clockwise)
 *
 * \param pid The PID to be used for the turn
 * \param angle The angle to make the robot turn in \b degrees
 */
void MoveTurnPID(PID pid = turnPID, double angle = 90, const double &MAX_REVS = 50.0){
  const int sign = angle / abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = odometry::GetDegrees(); // Angle relative to the start
  
  double timeLimit = getTimetoTurnDeg(angle);

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < 3 * timeLimit){

    double traveledAngle = abs(odometry::GetDegrees() - startAngle);

    double output = pid.Output(angle, traveledAngle);

    pros::lcd::print(0, "Time Limit %.2f", timeLimit);
    pros::lcd::print(1, "Time: %.2f", time);
    pros::lcd::print(2, "Gyroscope Displacement %.2f", traveledAngle);

    // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
    driveLeft.moveVelocity(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
    driveRight.moveVelocity(-sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));

    pros::delay(10);
  }

  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);

  #undef time
}

/**
 * \brief Turns the robot towards a specific direction
 *
 * \param x The x component of the point we wish to face
 * \param y The y component of the point we wish to face
 * 
 * \note Uses coordinate system from GPS in \b meters
*/
void turnToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = position();

  // Do the movement
  turn(-calculateTurn(target, current));
}

/**
 * \brief Goes to the target point
 *
 * \param x The x component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
 * \param y The y component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
 *  
 * \note Uses coordinate system from GPS in \b meters
*/
void goToTarget(double x, double y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = position();

  // Do the movement
  turn(-calculateTurn(target, current));
  move(findDistance(target, current));
}

/**
 * \brief S-graph motion profile for linear movement
 * 
 * \param dist The distance to be moved in \b inches, positive values will move forward and negative values backwards
 */
void motionProfile(double dist = TILE_WIDTH){
  if(dist == 0) { return; }
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  double dt = 0.02; // (s)
  double currVelocity = 0;
  double traveledDist = 0;
  Vector startPos = aon::odometry::GetPosition();

  double now = pros::micros() / 1E6;
  double lastTime = now;

  forwardProfile.setVelocity(driveFull.getActualVelocity());

  while(traveledDist < dist){
    traveledDist = (aon::odometry::GetPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt =  now - lastTime;
    lastTime = now;

    currVelocity = forwardProfile.update(remainingDist, dt);

    driveFull.moveVelocity(sign * currVelocity);

    if(traveledDist >= dist) { break; } // Overshoot prevention

    pros::delay(20);
  }

  driveFull.moveVelocity(0);
}

/**
 * \brief S-graph motion profile for rotations
 * 
 * \param angle The angle in \b degrees we wish to rotate the robot, positive is clockwise and negative is counter-clockwise
 */
void turnProfile(double angle = 90){
  if(angle == 0) { return; }
  const int sign = angle / abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  
  const double circumference = DRIVE_LENGTH * M_PI; // Of the robot's rotation, used in the condition to calculate the length of arc remaining
  const double MAX_VELOCITY = MAX_RPM; // (RPM)
  const double MAX_JERK = MAX_ACCEL; // (RPM/s^2)
  double dt = 0.02; // (s)
  double currVelocity = 0;
  double currAccel = 0;
  double traveledAngle = 0;
  double startAngle = aon::odometry::GetDegrees();

  double now;
  double lastTime = pros::micros() / 1E6;
  
  while(traveledAngle < angle){
    traveledAngle = abs(aon::odometry::GetDegrees() - startAngle);
    double remainingAngle = angle - traveledAngle;
    now = pros::micros() / 1E6;
    dt =  now - lastTime;
    lastTime = now;

    // Debugging output to brain
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
    pros::lcd::print(2, "RPM: %.2f, Accel: %.2f", currVelocity, currAccel);
    pros::lcd::print(3, "Remaining: %.2f", remainingAngle);
    pros::lcd::print(4, "Calculated Velocity: %.2f", getSpeed(currVelocity));
    pros::lcd::print(5, "Max Velocity: %.2f", getSpeed(MAX_VELOCITY));

    // Acceleration
    // For the condition, consider half the deceleration for accuracy (there is an error of half an inch almost constant when not used, I have to investigate a bit further on that part but if works fine like this)
    if(circumference * (remainingAngle / 360.0) <= getSpeed(currVelocity) * getSpeed(currVelocity) / (2.0 * getSpeed(MAX_DECEL * 0.5))){
      currAccel = - MAX_DECEL;
    } else {
      currAccel = std::min(currAccel + (MAX_JERK * dt), MAX_ACCEL);
    }

    currVelocity += currAccel * dt;
    currVelocity = std::min(currVelocity,  MAX_VELOCITY);

    driveLeft.moveVelocity(sign * currVelocity);
    driveRight.moveVelocity(-sign * currVelocity);

    if(traveledAngle >= angle) { break; } // Overshoot prevention

    pros::delay(20);
  }

  driveFull.moveVelocity(0);
}


// ============================================================================
//   ___ ___ __  __ ___ _    ___   __  __  _____   _____ __  __ ___ _  _ _____
//  / __|_ _|  \/  | _ \ |  | __| |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  \__ \| || |\/| |  _/ |__| _|  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |___/___|_|  |_|_| |____|___| |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================

/**
 * \brief Moves the robot straight accross a given amount of tiles
 *
 * \param amt The amount of tiles to be driven
 *
 * \attention To move in reverse make the amount negative
 *
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveTilesStraight(double amt = 1) {
  move(TILE_WIDTH * amt);
}

/**
 * \brief Moves the robot straight accross a given amount of half-tiles
 *
 * \param amt The amount of half-tiles to be driven
 *
 * \attention To move in reverse make the amount negative
 *
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveHalfTiles(int amt = 1) {
  move((TILE_WIDTH / 2) * amt);
}

/**
 * \brief Moves the robot diagonally accross a given amount of tiles
 *
 * \param amt The amount of tiles to be driven
 *
 * \attention To move in reverse make the amount negative
 *
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveTilesDiag(int amt = 1) {
  move(TILE_DIAG_LENGTH * amt);
}

/**
 * \brief Moves the robot diagonally accross a given amount of half-tiles
 *
 * \param amt The amount of half-tiles to be driven
 *
 * \attention To move in reverse make the amount negative
 *
 * \warning Robot should be aligned properly to achieve desired result
 */
void moveHalfDiagTiles(int amt = 1) {
  move((TILE_DIAG_LENGTH / 2) * amt);
}

/**
 * \brief Turns the robot clockwise by 90
 *
 * \param amt The amount of 90 degree turns to make
 *
 * \attention To turn counterclockwise make the amount negative
 */
void turn90(int amt = 1){
  turn(90 * amt);
}

/**
 * \brief Moves the robot a given distance
 *
 * \param dist The distance to move in \b inches
 * 
 * \details A positive `dist` makes the robot go forward while a negative `dist` makes the robot go backwards
 */
int move(const double &dist = TILE_WIDTH)
{
  motionProfile(dist);
  return 1;
}

/**
 * \brief Turn the robot a given angle (default is clockwise)
 *
 * \param angle The angle to turn in \b degrees
 *
 * \details Clockwise is positive and counter-clockwise is negative
 */
int turn(const double &angle = 90)
{
  turnProfile(angle);
  return 1;
}


// ============================================================================|
//   ____        _       ____             _   _                
//  / ___| _   _| |__   |  _ \ ___  _   _| |_(_)_ __   ___  ___
//  \___ \| | | | '_ \  | |_) / _ \| | | | __| | '_ \ / _ \/ __|
//   ___) | |_| | |_) | |  _ < (_) | |_| | |_| | | | |  __/\__ \
//  |____/ \__,_|_.__/  |_| \_\___/ \__,_|\__|_|_| |_|\___||___/
// ============================================================================|

/**
 * \brief This small subroutine moves the intake such that a ring is scored on the mobile goal being carried
 *
 * \param delay The time in \b milliseconds to leave the intake running
 */
void pickUpRing(const int &delay = 1000){
  intake.moveVelocity(INTAKE_VELOCITY / .8);
  pros::delay(delay);
  intake.moveVelocity(0);
}

/**
 * \brief This small subroutine moves the rail such that a ring is scored on the mobile goal being carried
 *
 * \param delay The time in \b milliseconds to leave the intake running
 */
void scoreRing(const int &delay = 1500){
  rail.moveVelocity(INTAKE_VELOCITY);
  pros::delay(delay);
  rail.moveVelocity(0);
}

/// @brief Asynchronous task for activating the intake when a ring is encountered
void intakeScan(){
  while(true){
    if (intakeScanning && distanceSensor.get() <= DISTANCE) {
      pickUpRing();
      driveFull.moveVelocity(0);
      intake.moveVelocity(0);
    }
    pros::delay(20);
  }
}

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
  driveFull.moveVelocity(-100);
  pros::delay(delay * 5 / 6);
  claw.set_value(true);
  pros::delay(delay * 1/6);
  driveFull.moveVelocity(100);
  pros::delay(delay);
  driveFull.moveVelocity(0);
}

/**
 * \brief Discards disk at beginning of match
 *
 * \note This function is really meant for routines that will focus on enemy rings
 */
void discardDisk(){
  intake.moveVelocity(-INTAKE_VELOCITY);
  pros::delay(1000);
  intake.moveVelocity(0);
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
  move(-abs(dist));
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
  turn(-45);
  moveIndexer(false);
}

/// @brief Drops the gate from starting position so the robot can grab stuff
void enableGate(){
  gate.moveVelocity(-100);
  pros::delay(250);
  gate.moveVelocity(0);
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
    driveLeft.moveVelocity(SPEED);
    driveRight.moveVelocity(-SPEED);
    pros::delay(20);
  }
  #undef TURRET_ANGLE
  deactivateORBITFollow();
  driveLeft.moveVelocity(0);
  driveRight.moveVelocity(0);
  if(color == STAKE){
    turn(180);
  }
}

/// @brief Aligns front of robot and turns around to grab the stake
/// @param dist The absolute value of the distance that the robot is from the stake when it begins alignment in \b inches
void findAndGrabGoal(const double &dist = 8){
  alignRobotTo(STAKE);
  move(-abs(dist));
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
double getDistanceToRing(const Colors &color = COLOR){
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
void driveTillPickUp(const double &distance = getDistanceToRing()){
  const double additional_distance = 0; //? This is to give the robot some distance to actually grip the donut, determine this experimentally
  activateIntakeScan();
  move(distance + additional_distance);
  deactivateIntakeScan();
}

/// @brief Get a stake and scores a preload
void grabAndScore(){
  findAndGrabGoal(10);
  scoreRing();
}

/// @brief Aligns robot to the ring of the specified `color` and grabs it and scores it on the held stake
/// @param color The color of the ring to be picked up
void alignAndIntake(const Colors &color = COLOR){
  COLOR = color;
  move(12);
  alignRobotTo(COLOR);
  move(12);
  driveTillPickUp();
  scoreRing();
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
  aon::goToTarget(.6, -1.2);
  aon::goToTarget(1.2, -.6);
  aon::goToTarget(1.2, .6);
  aon::goToTarget(.6, 1.2);
  aon::goToTarget(-.6, 1.2);
  aon::goToTarget(-1.2, .6);
  aon::goToTarget(-1.2, -.6);
  aon::goToTarget(-.6, -1.2);
  aon::goToTarget(.6, -1.2);
  aon::goToTarget(1.2, -.6);
}

/// @brief  Speed calculation test using the distance sensor
/// @param RPM The velocity for the motors
void testSpeed(double RPM = (double)driveFull.getGearing()){
  MovingAverage mav(50);
  while(true) {
    driveFull.moveVelocity(RPM);
    double measured = metersToInches(distanceSensor.get_object_velocity());
    double calculated = getSpeed(RPM);
    double error = abs(getErrorPercentage(calculated, measured));
    double avg = mav.update(error);
    pros::lcd::print(1, "RPM: %.2f", RPM);
    pros::lcd::print(2, "Calculated Velocity: %.2f", calculated);
    pros::lcd::print(3, "Measured Velocity: %.2f", measured);
    pros::lcd::print(4, "Error %: %.2f%", avg);
    pros::delay(10);
  }
}

/// @brief Small test to see if odom works with auton
void testOdom(){
  // Motion Profile
  move(12 * 3);
  pros::delay(1000);
  move(-12 * 3);
  pros::delay(1000);
  turn(90);
  pros::delay(1000);
  turn(-90);
  pros::delay(1000);

  // PID Forward
  MoveDrivePID(drivePID, 12 * 3);
  pros::delay(1000);
  MoveDrivePID(drivePID, -12 * 3);
  pros::delay(1000);

  // PID Rotations
  MoveTurnPID(turnPID, 90);
  pros::delay(1000);
  MoveTurnPID(turnPID, -90);
  pros::delay(1000);
  MoveTurnPID(turnPID, 45);
  pros::delay(1000);
  MoveTurnPID(turnPID, 45);
  pros::delay(1000);
  MoveTurnPID(turnPID, -45);
  pros::delay(1000);
  MoveTurnPID(turnPID, -45);
  pros::delay(1000);
}

/// @brief Competition specific test to see if the indexer interacted as intended
void testIndexer(){
  moveIndexer();
  motionProfile(40);
  move(-6);
  turn(180);
  move(-2);
  grabGoal();
}

/// @brief Test to ensure the concurrency is working fine, requires `intakeScanning` to be running in another thread
void testConcurrency(){
  activateIntakeScan();
  int startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime
  while(time < 5){
    driveFull.moveVelocity(100);
    pros::delay(20);
  }
  #undef time
  driveFull.moveVelocity(0);
  deactivateIntakeScan();
}

/// @brief Test function to see if the angle from the ORBIT makes sense
void testTurret(){
  while(true){
    double position = turretEncoder.get_angle() / 100.0;
    const double turretTurn = aon::operator_control::AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY);
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
  activateIntakeScan();
  pros::delay(500);
  okapi::EKFFilter ekf;
  const int TOLERANCE = 5; //? Probably adjust this
  double difference;
  #define TURRET_ANGLE turretEncoder.get_angle() / 100

  const double dt = 0.02;

  forwardProfile.setMaxVelocity(MAX_RPM / 2);

  while(abs(TURRET_ANGLE) > TOLERANCE){
    auto object = vision_sensor.get_by_sig(0, color);

    // Safety to scan when object is lost
    if(object.signature != color) { 
      activateORBITScan();
      driveFull.moveVelocity(0);
      continue;
    } 
    else {
      activateORBITFollow();
    }

    difference = TURRET_ANGLE < 180 ? TURRET_ANGLE : TURRET_ANGLE - 360;
    //? maybe motion profile this variable
    double TURN = turnPID.Output(0, -difference) * 500;
    
    const double distance = ekf.filter(groundDistanceToDisk(object.width));

    double FORWARD = forwardProfile.update(distance, dt);

    driveLeft.moveVelocity(FORWARD + TURN);
    driveRight.moveVelocity(FORWARD - TURN);
    pros::delay(20);
  }
  #undef TURRET_ANGLE
  #undef TIME
  deactivateORBITFollow();
  deactivateORBITScan();
  deactivateIntakeScan();
  forwardProfile.setMaxVelocity(MAX_RPM);
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
      const double avgDif = avgMav.update(getPercentDifference(avg, distance));
      const double ekfDif = ekfMav.update(getPercentDifference(filtered, distance));
      const double avg_ekfDif = avg_ekfMav.update(getPercentDifference(avg, filtered));
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
    const double pos = gyroscope.get_heading();
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

/// @brief Makes the robot drive in an arc motion based on a given `radius`
/// @param radius The radius of the arc of the motion in \b inches measured from the center of rotation of the robot to the reference point in the right when positive and in the left when negative
/// @param midSpeed The speed with which to drive in \b RPM (positive speed will go forward and negative speed will go backwards)
/// @note A positive `radius` will cause a clockwise rotation, while a negative `radius` will cause a counter-clockwise rotation
/// @see https://www.desmos.com/calculator/91cbd82e8b
void driveInArc(double radius, const double &midSpeed = 200) {
  if(radius == 0) return;
  const bool clockwise = radius > 0.0;
  radius = std::abs(radius);

  // Calculate wheel speeds based on center speed and arc geometry
  const double outerRatio =  (radius + (DRIVE_WIDTH / 2)) / radius;
  const double innerRatio = (radius - (DRIVE_WIDTH / 2)) / radius;
  const double outerSpeed = midSpeed * outerRatio;
  const double innerSpeed = midSpeed * innerRatio;

  double leftSpeed, rightSpeed;
  
  // Clockwise, more speed on the left
  if(clockwise) {
    leftSpeed = outerSpeed;
    rightSpeed = innerSpeed;
  }
  // Counter-clockwise, more speed on the right
  else {
    rightSpeed = outerSpeed;
    leftSpeed = innerSpeed;
  }

  driveLeft.moveVelocity(leftSpeed); 
  driveRight.moveVelocity(rightSpeed);
}

/// @brief Makes the robot drive in an arc motion based on a given `radius` for a given `angle`
/// @param radius The radius of the arc of the motion in \b inches measured from the center of rotation of the robot to the reference point in the right when positive and in the left when negative
/// @param angle The angle of the arc we want to cover in \b degrees, a negative angle will cause the robot to go in reverse
/// @note A positive `radius` will cause a rotation with reference to a point to the right, while a negative `radius` will cause a rotation with reference to a point to the left
/// @note A positive `angle` will cause a forward movement, while a negative `angle` will cause a backwards movement
/// @see https://www.desmos.com/calculator/91cbd82e8b
void driveAngleOfArc(const double &radius = DRIVE_WIDTH, const double &angle = 90) {
  if(angle == 0) { return; }
  if(radius == 0) {
    turn(angle);
    return;
  }
  const short sign = angle / std::abs(angle);
  const double distance = std::abs((2 * radius * M_PI) * (angle / 360));
  double midSpeed;
  double traveledDist = 0, remainingDist = distance;
  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;
  const double rightEncStartPos = encoderRight.get_position(); //! Temporary
  const double leftEncStartPos = encoderLeft.get_position(); //! Temporary
  // TODO: uncomment the two lines that use `odometry::getTraveledDistance()` to track the distance after that method is implemented and remove "//! Temporary" lines
  // const double startDist = odometry::getTraveledDistance();
  while(traveledDist < distance){
    // traveledDist = odometry::getTraveledDistance() - startDist;
    const double rightEncDist = (std::abs(encoderRight.get_position() - rightEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    const double leftEncDist = (std::abs(encoderLeft.get_position() - leftEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    traveledDist = (rightEncDist + leftEncDist) / 2; //! Temporary
    remainingDist = distance - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    midSpeed = forwardProfile.update(remainingDist, dt);
    lastTime = now;

    driveInArc(radius, sign * midSpeed);

    pros::delay(20);
  }

  driveFull.moveVelocity(0);
}

/// @brief Calculates the angle of a `point` with respect to a circle with a given `center` in the range [0, 360)
/// @param point The point whose angle in the circle we want to calculate
/// @param center The center of the circle in which the point resides
/// @return The angle of that `point` in the circle in \b degrees
double getAngleInCircle(Vector point, Vector center)
{
  if (point == center) { return 0; } // Should never happen

  // Avoid 0 division
  if (point.GetX() == center.GetX() && point.GetY() > center.GetY())
  {
    return 90;
  }
  else if (point.GetX() == center.GetX() && point.GetY() < center.GetY())
  {
    return 270;
  }
  // Check edge cases
  else if (point.GetY() == center.GetY() && point.GetX() > center.GetX())
  {
    return 0;
  }
  else if (point.GetY() == center.GetY() && point.GetX() < center.GetX())
  {
    return 180;
  }

  // Get the angle
  const double theta = std::atan((point.GetY() - center.GetY()) / (point.GetX() - center.GetX())) * 180 / M_PI;

  // Normalize the angle depending on the quadrant its on
  double normalizer = 0;

  // First quadrant
  if (point.GetX() - center.GetX() > 0 && point.GetY() - center.GetY() > 0)
  {
    normalizer = 0;
  }
  // Second quadrant
  else if (point.GetX() - center.GetX() < 0 && point.GetY() - center.GetY() > 0)
  {
    normalizer = 180;
  }
  // Third quadrant
  else if (point.GetX() - center.GetX() < 0 && point.GetY() - center.GetY() < 0)
  {
    normalizer = 180;
  }
  // Fourth quadrant
  else if (point.GetX() - center.GetX() > 0 && point.GetY() - center.GetY() < 0)
  {
    normalizer = 360;
  }
  return std::fmod(theta + normalizer, 360);
}

/// @brief Calculates the angle of the arc between two points given the center of the circle and the two points
/// @param a The first point in the arc
/// @param b The other point in the arc
/// @param center The center of the circle
/// @return The angle of the arc from one point to another in \b degrees
/// @details The arc whose angle we are measuring starts from whichever point is closest to the 0º mark (positive x-axis going counter-clockwise)
double getAngleOfArc(const Vector &a, const Vector &b, const Vector &center)
{
  const double theta_a = getAngleInCircle(a, center);
  const double theta_b = getAngleInCircle(b, center);

  return std::max(theta_a, theta_b) - std::min(theta_a, theta_b);
}

/// @brief Makes the robot drive in an arc motion to a specified point in the field
/// @param x The x coordinate of the point we want to go to in \b meters
/// @param y The y coordinate of the point we want to go to in \b meters
/// @note Odometry must be working for global positioning on the field
/// @see https://www.desmos.com/calculator/5abb373276
void driveInArcTo(const double &x, const double &y){
  // Get the current pose
  Vector position = odometry::GetPosition();
  position.SetPosition(inchesToMeters(position.GetX()), inchesToMeters(position.GetY()));
  double heading = odometry::GetDegrees(); //? should this come in the same format as the GPS heading?
  Vector target = Vector().SetPosition(x, y);

  // Convert the heading to traditional math coordinates
  heading = (90 - heading); //? only do the `(90 - heading)` part if the heading comes in gps coordinates
  if (heading < 0) { heading += 360; }
  heading *=  M_PI / 180;

  // (heading - π/2) % π cannot be 0 because tan(heading) would not be defined
  const bool isTanHeadingDefined = std::fmod(heading - M_PI_2, M_PI) != 0;

  // Calculate slopes of tangent to circular path and secant that cuts through current point and desired point
  double m_t = isTanHeadingDefined ? std::tan(heading) : DBL_MAX;
  double m_s = (position.GetX() != x) ? (position.GetY() - y) / (position.GetX() - x) : DBL_MAX;

  // Avoid 0 division later by switching to a very small value if a 0 slope arises
  m_t = m_t == 0 ? DBL_MIN : m_t;
  m_s = m_s == 0 ? DBL_MIN : m_s;

  // Get midpoint of the secant
  Vector midpoint = Vector().SetPosition((position.GetX() + x) / 2, (position.GetY() + y) / 2);

  // Calculate the position of the center of the circular path
  double centerX = (midpoint.GetY() - position.GetY() - (position.GetX() / m_t) + (midpoint.GetX() / m_s)) / ((-1 / m_t) + (1 / m_s));
  double centerY = ((-1 / m_t) * (centerX - position.GetX())) + position.GetY();
  Vector center = Vector().SetPosition(centerX, centerY);

  // Get the radius using the pythagorean theorem
  double radius = std::hypot(position.GetX() - center.GetX(), position.GetY() - center.GetY());

  // Determine the angle with some geometry and trigonometry
  double angle = getAngleOfArc(position, target, center);

  // Use a projection to determine which way we are turning
  const double projectionStep = 0.001;
  const Vector projection = Vector().SetPosition(position.GetX() + (projectionStep * std::cos(heading)),
                                                 position.GetY() + (projectionStep * std::sin(heading)));
  const double projectionAngle = getAngleInCircle(projection, center);
  
  const double positionAngle = getAngleInCircle(position, center);
  const double targetAngle = getAngleInCircle(target, center);
  
  // If going clockwise, the center is to the right (positive radius) and to the left in a counter-clockwise movement (negative radius)
  const bool clockwise = (targetAngle < projectionAngle && projectionAngle < positionAngle) || (projectionAngle < positionAngle && positionAngle < targetAngle) || (positionAngle < targetAngle && targetAngle < projectionAngle);
  if (!clockwise) { radius *= -1; }
  
  // Check if we have to go the long way around
  const bool longWay = (getAngleOfArc(projection, target, center) > angle) || (positionAngle < targetAngle && targetAngle < projectionAngle);
  if (longWay) { angle = 360 - angle; }

  driveAngleOfArc(metersToInches(radius), angle);
}

/// @brief Function wrapper for test function that is to be executed through the GUI
/// @return 1 for successful execution
/// @note Usually the tests in here use `potentiometer.get_value()` to tune a parameter in a function as well as testing the function itself
int testAdjustable(){
  driveInArcTo(inchesToMeters(TILE_WIDTH / 2), inchesToMeters(TILE_WIDTH / 2));
  return 1;
}

/// @brief Function wrapper for test functions that are to be executed through the GUI
/// @return 1 for successful execution
/// @note Choose between 3 tests depending on the result of `potentiometer.get_value()`
int testMultiple(){
  int choice = potentiometer.get_value();
  // UP
  if(choice > 2550){
    driveInArcTo(-inchesToMeters(TILE_WIDTH / 2), inchesToMeters(TILE_WIDTH / 2));
  }
  // MIDDLE
  else if (choice > 1100){
    driveInArcTo(-inchesToMeters(TILE_WIDTH / 2), -inchesToMeters(TILE_WIDTH / 2));
  }
  // DOWN
  else {
    driveInArcTo(inchesToMeters(TILE_WIDTH / 2), -inchesToMeters(TILE_WIDTH / 2));
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
  move(-3);
  grabGoal();
  scoreRing();
  move(10);
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
  move(6);
  scoreRing();
  enableGate();

  // Get the next ring in our side
  turnToTarget(-.6, -1.2);
  move(6);
  RemoveTop();
  move(-6);
  alignRobotTo(COLOR);
  driveTillPickUp();

  // Get the last ring in that line
  move(-6);
  turnToTarget(-1.2, -1.2);
  move(6);
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
  move(6);
  scoreRing();
  enableGate();
  
  // Get the next ring in our side
  turnToTarget(.6, 1.2);
  move(6);
  RemoveTop();
  move(-6);
  alignRobotTo(COLOR);
  driveTillPickUp();
  
  // Get the last ring in that line
  move(-6);
  turnToTarget(1.2, 1.2);
  move(6);
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
  move(4);
  pickUpRing(1500);
  move(-4);
  move(4);
  pickUpRing(1500);
  move(-4);
  move(4);
  pickUpRing(1500);
  move(-4);
  
}

/**
  Will take the closest to us, in our side and take th rings that are below

  START IN THE POSITIVE SIDE OF THE FIELD, LOOKING AT THE STAKE WHEN WE ARE RED.

  RED-NEGATIVE SIDE
*/
int safeRingRoutine() {
  findAndGrabGoal(8); // 8 inches from stake
  turnToTarget(-1.2,-1.2);
  goToTarget(-1.2,-1.2);
  driveIntoRing(RED);
  turnToTarget(-1.5,0);
  goToTarget(-1.5,0);
  driveIntoRing(RED);
  turnToTarget(1.2,-1.2);
  goToTarget(1.2,-1.2);
  driveIntoRing(RED);
}

int safeRingRoutine2() {
  findAndGrabGoal(6); //6 inches from stake
  turnToTarget(-1.2,-1.2);
  goToTarget(-1.2,-1.2);
  driveIntoRing(RED);
  turnToTarget(-1.5,0);
  goToTarget(-1.5,0);
  driveIntoRing(RED);
  turnToTarget(-1.2,1.2);
  goToTarget(-1.2,1.2);
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
  move(6);
  scoreRing(2000);
  enableGate();

  // go to ring on the bottom
  goToTarget(1.2, -0.55);
  RemoveTop();
  driveIntoRing(COLOR);

  // then the one below that one
  goToTarget(1.2, -1.1);
  driveIntoRing(COLOR);

  // drive into the corner and try to grab the rings
  goToTarget(1.7, -1.7);
  RemoveTop();
  driveIntoRing(COLOR);
  turnToTarget(1.7, -1.7);
  RemoveTop();
  driveIntoRing(COLOR);

  move(-6);
  turnToTarget(1.8, 1.8);
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
  turnToTarget(-1.2, -1.2);
  driveIntoRing();
  move(-12);

  // Grab ring in (-1.8, -1.8)
  turnToTarget(-1.8, -1.8);
  move(6);
  driveIntoRing();
  move(-12);

  // Grab ring in (-.6, -.6)
  goToTarget(-.9, -.9);
  driveIntoRing();
  move(-24);

  // Grab ring in (0, -1.2)
  turnToTarget(0, -1.2);
  
  
}



#else

int greenBotRedSide(){
  move(-6);
  grabGoal();
  scoreRing();
  enableGate();

  turnToTarget(-1.2,1.2);
  moveTilesStraight(1.3);
  driveTillPickUp();

  turnToTarget(-0.6,1.2);
  moveTilesStraight(1);
  driveTillPickUp();

  turnToTarget(-1.5,0);
  moveTilesStraight(1.3);
  driveTillPickUp();
}

void simple_Auto_Red(){
    move(-.5);
    grabGoal();
    scoreRing();
    enableGate();
  
    turnToTarget(-1.2,1.2);
    moveTilesStraight(1.3);
    driveIntoRing(COLOR);// change
  
    turnToTarget(-0.6,1.2);
    moveTilesStraight(1);
    driveIntoRing(COLOR);//change
  
    turnToTarget(-1.5,0);
    moveTilesStraight(1.3);
    driveIntoRing(COLOR);//change
  
}

void Auto_with_indexer(){
  moveTilesStraight(1.5);
  moveIndexer();
  moveTilesStraight(-.5);
  turn(180);
  grabGoal();
  scoreRing();
  enableGate();

  turnToTarget(-0.6,1.2);
  moveTilesStraight(1);
  driveIntoRing(COLOR); //channge 

  turnToTarget(-1.2,1.2);
  moveTilesStraight(1.3);
  driveIntoRing(COLOR);// change

  turnToTarget(-1.5,0);
  moveTilesStraight(1.3);
  driveIntoRing(COLOR);//change


}


/**
 * \author Solimar
 */
int SkillsGreenBotSoli(){
  //First Grab Nearest stake 
  move(-1.3);
  grabGoal();
  
  //Then attempt to grab the red rings towards the corner 
  turnToTarget(-1.8, 1.8);
  driveIntoRing(COLOR);
  driveIntoRing(COLOR);
  
  //turn towards remaining going up
  //turn towards red on the line
  //drop steak
  goToTarget(-1.5, -0.3);
  turnToTarget(-1.2, 0.0);
  FollowWithTurret(RED);
  dropGoal();
  
  //pick up second steak 
  turnToTarget(0.6, 0.6);
  turn(180);
  moveTilesDiag(-1);
  grabGoal();
  //pick up reds around whilst also tumbando los stacks red-blue
  turnToTarget(1.8, 1.8);
  driveIntoRing(COLOR);
  driveIntoRing(COLOR);
  
  //turn towards remaining going up
  //turn towards red on the line
  //drop steak
  goToTarget(1.5, -0.3);
  turnToTarget(-1.2, 0.0);
  FollowWithTurret(RED);
  dropGoal();
}

/**
 * \author Jorge G
 */
int SkillsGreenBotJorge(){
  raceToGoal(50); //preg a kev
  turnToTarget(0, -1.2);
  goToTarget(0, -1.2);
  FollowWithTurret(RED);
  //First ring ^
  turnToTarget(-0.6, 0.6);
  goToTarget(-0.6, 0.6);
  FollowWithTurret(RED);
  // Second ring ^
  turnToTarget(-1.2, 1.2);
  goToTarget(-1.2, 1.2);
  FollowWithTurret(RED);
  // Third Ring ^
  turnToTarget(-1.8, 1.8);
  goToTarget(-1.8, 1.8);
  FollowWithTurret(RED); 
  // Fourth Ring ^
  turnToTarget(0, 1.2);
  goToTarget(0, 1.2);
  FollowWithTurret(RED); 
  // Fifth Ring ^
  dropGoal();
  // release stake
  turnToTarget(0.6, 0.6);
  move(12); // measure
  findAndGrabGoal(10); //measure
  // grab stake at (0.6, 0.6)
  turnToTarget(0, 1.5);
  goToTarget(0, 1.5);
  FollowWithTurret(RED); 
  // 2nd stake first ring
  turnToTarget(0.6, 1.2);
  goToTarget(0.6, 1.2);
  FollowWithTurret(RED);   
  // Second Ring ^
  turnToTarget(1.2, 1.2);
  goToTarget(1.2, 1.2);
  FollowWithTurret(RED);  
  // Third Ring ^
  turnToTarget(1.2, 0.6);
  goToTarget(1.2, 0.6);
  FollowWithTurret(RED);  
  // Fourth Ring ^
  turnToTarget(1.8, 1.8);
  goToTarget(1.8, 1.8);
  FollowWithTurret(RED); 
  // Fifth Ring ^
  move(-3);
  FollowWithTurret(BLUE);
  dropGoal();
}
#endif

};  // namespace aon


