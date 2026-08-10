#include <cmath>
#include <vector>

#include "aon/auton/native-tests.hpp"
#include "aon/globals.hpp"
#include "aon/math/misc/misc.hpp"
#include "aon/tools/moving-average.hpp"

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

  // A 500 ms sampling window suppresses single-frame vision width spikes.
  for(int i = 0; i < 100; i++){
    distance = ekf.filter(orbit.groundDistanceToDisk((orbit.getLargestObject()).width));
    pros::delay(5);
  }

  return distance;
}

/// @brief Drives forward until a ring hits the distance sensor
/// @param distance The distance from the robot to a ring
void driveTillPickUp(const double &distance = getDistanceToRing()){
  // Mechanism compression may require extra travel after first detection.
  const double additional_distance = 0;
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
  // Five pixels avoids hunting as vision readings alternate around center.
  const double tolerance = 5;
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
void distanceSensorSpeed(double RPM){
  MovingAverage mav(50);
  while(true) {
    drivetrain.motors(RPM);
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
    const double pos = odometry.imuSensor().get_heading();
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

};  // namespace aon
