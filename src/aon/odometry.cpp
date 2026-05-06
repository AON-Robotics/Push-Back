#include "../../include/aon/odometry/odometry.hpp"

namespace aon {

Odometry::Odometry(short left, short right, short back, short gps, short gyro)
    : conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
      encoderLeft(abs(left), (left / abs(left) != 1)),
      encoderRight(abs(right), (right / abs(right) != 1)),
      encoderBack(abs(back), (back / abs(back) != 1)),
      gps(gps, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET,
          GPS_Y_OFFSET)
#if GYRO_ENABLED
      ,
      gyroscope(gyro)
#endif
{
}

Odometry::Odometry(const Odometry& other)
    : conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
      encoderLeft(other.encoderLeft),
      encoderRight(other.encoderRight),
      encoderBack(other.encoderBack),
      gps(other.gps)
#if GYRO_ENABLED
      ,
      gyroscope(other.gyroscope)
#endif
{
}

/// @brief Get current X position in \b inches
/// @return Returns current X position in \b inches
double Odometry::getX() {
  p_mutex.take(1);
  double currentX = position.GetX();
  p_mutex.give();
  return currentX;
}

/// @brief Get current Y position in \b inches
/// @return Returns current Y position in \b inches
double Odometry::getY() {
  p_mutex.take(1);
  double currentY = position.GetY();
  p_mutex.give();
  return currentY;
}

/// @brief Get a vector with the current position
/// @return Returns new vector with current position
Vector Odometry::getPosition() {
  p_mutex.take(1);
  Vector pos = position;
  p_mutex.give();
  return pos;
}

/// @brief Set position in \b inches
/// @param x The x coordinate in the field in \b inches
/// @param y The y coordinate in the field in \b inches
void Odometry::SetPosition(double x, double y) {
  p_mutex.take(1);
  position.SetPosition(x, y);
  p_mutex.give();
}

/// @brief Get current pose's angle in \b degrees
/// @return Returns current pose's angle in \b degrees
double Odometry::getDegrees() {
  orientation_mutex.take(1);
  double currentDegrees = orientation.GetDegrees();
  orientation_mutex.give();
  return currentDegrees;
}

/// @brief Set current pose's angle in \b degrees
/// @param degrees Input value to set the current angle to
void Odometry::setDegrees(double degrees) {
  orientation_mutex.take(1);
  orientation.SetDegrees(degrees);
  orientation_mutex.give();
  deltaTheta = 0.0;
}

/// @brief Get current pose's angle in \b radians
/// @return Returns current pose's angle in \b radians
double Odometry::getRadians() {
  orientation_mutex.take(1);
  const double currentRadian = orientation.GetRadians();
  orientation_mutex.give();
  return currentRadian;
}

/// @brief Set current pose's angle in \b radians
/// @param radians Input value to set the current angle to
/// @warning Sets angles in units of \b radians. INPUT MUST BE IN \b RADIANS
void Odometry::setRadians(double radians) {
  orientation_mutex.take(1);
  orientation.SetRadians(radians);
  orientation_mutex.give();
}

/// @brief Get current position in the X-axis, Y-axis, and  angle in \b degrees
Pose Odometry::getPose() { return Pose(getX(), getY(), getDegrees()); }


/// @brief Resets the Odometry values with `INITIAL_ODOMETRY_X`,Y and T
/// constants.
void Odometry::resetInitial() {
  /*
  ATTENTION
  We need to know where the robot start (coordinates), for the odometry knows
  where the robot is at all times. Maybe using gps or a const variable
  */
  // Normal initial
  resetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

/// @brief Initialization function to put everything to 0
void Odometry::initialize() {
  encoderLeft.set_position(0);
  encoderRight.set_position(0);
  encoderBack.set_position(0);

  encoderLeft.reset();
  encoderRight.reset();
  encoderBack.reset();

  // Set initial position with gps (need test with field)
  // INITIAL_ODOMETRY_X = gps.get_x_position();
  // INITIAL_ODOMETRY_Y = gps.get_y_position();
  resetInitial();

  while (true) {
    update();
    pros::delay(10);
  }
}

/// @brief Fundamental function for Odometry.
/// @details Uses changes in encoder (right and left) and gyro to calculate position
void Odometry::update() { // TODO: implement odometer functions both for linear and rotational movement
  /// Read encoder values, divided by 100 to convert centidegrees to degrees
  encoderRight_data.currentValue = encoderRight.get_position() / 100.0;
  encoderLeft_data.currentValue = encoderLeft.get_position() / 100.0;
  // encoderBack_data.currentValue = encoderBack.get_position() / 100.0;

  // Convert to distances
  encoderRight_data.currentDistance =
      encoderRight_data.currentValue * conversionFactor;
  encoderLeft_data.currentDistance =
      encoderLeft_data.currentValue * conversionFactor;
  // encoderBack_data.currentDistance = encoderBack_data.currentValue *
  // conversionFactor;

  // Calculate deltas
  encoderRight_data.delta =
      encoderRight_data.currentValue - encoderRight_data.prevValue;
  encoderLeft_data.delta =
      encoderLeft_data.currentValue - encoderLeft_data.prevValue;
  // encoderBack_data.delta = encoderBack_data.currentValue -
  // encoderBack_data.prevValue;

  encoderRight_data.deltaDistance =
      encoderRight_data.currentDistance - encoderRight_data.previousDistance;
  encoderLeft_data.deltaDistance =
      encoderLeft_data.currentDistance - encoderLeft_data.previousDistance;
  // encoderBack_data.deltaDistance = encoderBack_data.currentDistance -
  // encoderBack_data.previousDistance;

  // Calculate delta theta if we dont have gyro
  double deltaTheta =
      (encoderLeft_data.deltaDistance - encoderRight_data.deltaDistance) /
      (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER +
       DISTANCE_LEFT_TRACKING_WHEEL_CENTER);

// If we have gyro, get value and calculate delta
#if GYRO_ENABLED
  // Read gyro value
  gyro_data.currentDegrees = gyroscope.get_heading();
  gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);

  // Normalize angle to prevent overshoot when using turn function
  if (gyro_data.currentDegrees > 180) {
    gyro_data.currentDegrees -= 360;
  } else if (gyro_data.currentDegrees <= -180) {
    gyro_data.currentDegrees += 360;
  }
  // Calculate delta
  gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
  gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);

  // Save current data for future calculations
  gyro_data.prevDegrees = gyro_data.currentDegrees;

  // Right now, confidence gyro 1.0, encoder confidence 0 (must sum 1)
  deltaTheta = (1 - GYRO_CONFIDENCE) * deltaTheta +
               GYRO_CONFIDENCE * gyro_data.deltaRadians;
#endif

  // Updating angle
  double previousTheta = getRadians();
  setRadians(getRadians() + deltaTheta);

  // Calculations simple trigonometry, i.e., mine :)
  // If we are rotating
  if (std::abs(deltaTheta * (180 / M_PI)) > 0.01) {
    // If turning in its own axis
    if ((encoderLeft_data.deltaDistance * encoderRight_data.deltaDistance) <=
        0) {
      deltaDlocal.SetPosition(0.0, 0.0);
    }
    // If we are going in a arc
    else {
      // Calculate the radius of rotation for each wheel
      double sign = (deltaTheta > 0) ? 1 : -1;
      double radiusLeft = (encoderLeft_data.deltaDistance / deltaTheta) -
                          sign * DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
      double radiusRight = (encoderRight_data.deltaDistance / deltaTheta) +
                           sign * DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;

      // Calculate radius
      double averageR = (radiusLeft + radiusRight) / 2;

      // Update position using trigonometry
      deltaDlocal.SetPosition(averageR * std::sin(deltaTheta),
                              averageR * (1 - std::cos(deltaTheta)));
    }
  }
  // If the robot is moving straight forward or backward, average encoder values for distance
  else {
    double deltaD =
        (encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) /
        2.0;
    deltaDlocal.SetPosition(deltaD, 0);
  }

  // Odometry copy from
  // https://medium.com/%40nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
  // Super accurate and use less calculations, but mine seems cooler :)
  double deltaD =
      (encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / 2.0;
  changeWeb.SetPosition(
      changeWeb.GetX() + (deltaD * cos(previousTheta + deltaTheta / 2)),
      changeWeb.GetY() + (deltaD * sin(previousTheta + deltaTheta / 2)));

  // Updating global position using 2D matrix transformation (previous way to
  // update to global coordinates)
  SetPosition(getX() + deltaDlocal.GetX() * std::cos(getRadians()) -
                  deltaDlocal.GetY() * std::sin(getRadians()),
              getY() + deltaDlocal.GetX() * std::sin(getRadians()) +
                  deltaDlocal.GetY() * std::cos(getRadians()));

  // Save current values as previous for future updates
  encoderLeft_data.prevValue = encoderLeft_data.currentValue;
  encoderRight_data.prevValue = encoderRight_data.currentValue;

  encoderRight_data.previousDistance = encoderRight_data.currentDistance;
  encoderLeft_data.previousDistance = encoderLeft_data.currentDistance;

  gyro_data.prevDegrees = gyro_data.currentDegrees;
}

/// @brief resets Odometry values using the particular parameters
/// @param x X position in \b inches
/// @param y Y position in \b inches
/// @param theta Angular position in \b degrees
void Odometry::resetCurrent(double x, double y, double theta) {
  const double currentAngleRight = encoderRight.get_position() / 100.0;
  const double currentAngleLeft = encoderLeft.get_position() / 100.0;
  const double currentAngleBack = encoderBack.get_position() / 100.0;
  const double currentAngleGyro = gyroscope.get_heading();
  std::cout << "currentAngleGyro: " << currentAngleGyro << "\n";

  // Reset encoder's struct variables
  encoderRight_data = {
      currentAngleRight,                     // current position in degrees
      currentAngleRight,                     // previous position in degrees
      0,                                     // delta in degrees
      currentAngleRight * conversionFactor,  // current position in inches
      currentAngleRight * conversionFactor,  // previous position in inches
      0.0};                                  // delta in inches

  encoderLeft_data = {
      currentAngleLeft,                     // current position in degrees
      currentAngleLeft,                     // previous position in degrees
      0,                                    // delta in degrees
      currentAngleLeft * conversionFactor,  // current position in inches
      currentAngleLeft * conversionFactor,  // previous position in inches
      0.0};                                 // delta in inches

  encoderBack_data = {
      currentAngleBack,                     // current position in degrees
      currentAngleBack,                     // previous position in degrees
      0,                                    // delta in degrees
      currentAngleBack * conversionFactor,  // current position in inches
      currentAngleBack * conversionFactor,  // previous position in inches
      0.0};                                 // delta in inches

  gyro_data = {0,     // current value degrees
               0,     // previous value degrees
               0,     // current radians
               0.0,   // delta degrees
               0.0};  // delta radians

  // Preset odometry values
  deltaTheta = 0.0;
  deltaDlocal.SetPosition(0.0, 0.0);

  // Other odometry we could use, less calculations
  changeWeb.SetPosition(0.0, 0.0);

  setDegrees(theta);
  SetPosition(x, y);
#if GYRO_ENABLED
  gyroscope.tare();
  pros::delay(3000);
#endif
}

/// @brief Returns position of the robot in the field
/// @returns The GPS coordinates as a `Vector`
Vector Odometry::gpsPosition() {
  pros::delay(2000);
  pros::c::gps_status_s_t status = gps.get_status();
  Vector current = Vector().SetPosition(status.x, status.y);

  return current;
}

/// @brief Simple debug function that prints odometry values
/// @details Blocking function that helps check if there are any issues with
/// odometry
/// @note Requires initialize pros::lcd and calling the odometry::Initialize
/// function
void Odometry::debug() {
  while (true) {
    pros::lcd::print(0, "X: %.3f", getX());
    pros::lcd::print(1, "Y: %.3f", getY());
    // pros::lcd::print(0, "X: %.3f, Y: %.3f", getX(), getY());
    pros::lcd::print(
        2, "Left : %.3f, %.3f, %.3f", encoderLeft_data.currentDistance,
        encoderLeft_data.previousDistance, encoderLeft_data.deltaDistance);
    pros::lcd::print(
        3, "Right: %.3f, %.3f, %.3f", encoderRight_data.currentDistance,
        encoderRight_data.previousDistance, encoderRight_data.deltaDistance);
    pros::lcd::print(4, "Heading: %.3f", getDegrees());
    pros::lcd::print(5, "Mine:   X: %.3f | Y: %.3f", getX(), getY());
    pros::lcd::print(6, "Web:    X: %.3f | Y: %.3f", changeWeb.GetX(),
                     changeWeb.GetDegrees());

    update();
    pros::delay(10);
  }
}

}  // namespace aon
