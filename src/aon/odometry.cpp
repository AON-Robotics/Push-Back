#include "../include/aon/odometry/odometry.hpp"

namespace aon {

Odometry::Odometry(short left, short right, short back, short gps, short gyro)
    : conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
      encoderLeft(abs(left), (left / abs(left) != 1)),
      encoderRight(abs(right), (right / abs(right) != 1)),
#if ENCODER_BACK_ENABLED
      encoderBack(abs(back), (back / abs(back) != 1)),
#endif
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
#if ENCODER_BACK_ENABLED
      encoderBack(other.encoderBack),
#endif
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
void Odometry::setPosition(double x, double y) {
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
  #if ENCODER_BACK_ENABLED
  encoderBack.set_position(0);
  #endif

  encoderLeft.reset();
  encoderRight.reset();
  #if ENCODER_BACK_ENABLED
  encoderBack.reset();
  #endif

  // Set initial position with gps (need test with field)
  // INITIAL_ODOMETRY_X = gps.get_x_position();
  // INITIAL_ODOMETRY_Y = gps.get_y_position();
  resetInitial();

  while (true) {
    update();
    pros::delay(20);
  }
}

/**
 * @brief Fundamental function for Odometry, use encoders and gyro to calculate position and orientation.
 *
 * @details Uses changes in encoder (right and left) and gyro to calculate position
 * 
 * */
void Odometry::update() {
  // Read encoder values, divided by 100 to convert centidegrees to degrees
  encoderRight_data.currentValue = encoderRight.get_position() / 100.0; 
  encoderLeft_data.currentValue = encoderLeft.get_position() / 100.0; 
        
  // Convert to distances
  encoderRight_data.currentDistance = encoderRight_data.currentValue * conversionFactor;
  encoderLeft_data.currentDistance = encoderLeft_data.currentValue * conversionFactor;
        
  // Calculate deltas
  encoderRight_data.delta = encoderRight_data.currentValue - encoderRight_data.prevValue;
  encoderLeft_data.delta = encoderLeft_data.currentValue - encoderLeft_data.prevValue;
        
  encoderRight_data.deltaDistance = encoderRight_data.currentDistance - encoderRight_data.previousDistance;
  encoderLeft_data.deltaDistance = encoderLeft_data.currentDistance - encoderLeft_data.previousDistance;
        
  // Take information from the back encoder
  #if ENCODER_BACK_ENABLED
  encoderBack_data.currentValue = encoderBack.get_position() / 100.0;
  encoderBack_data.currentDistance = encoderBack_data.currentValue * conversionFactor;
  encoderBack_data.delta = encoderBack_data.currentValue - encoderBack_data.prevValue;
  encoderBack_data.deltaDistance = encoderBack_data.currentDistance - encoderBack_data.previousDistance;
  #endif

  // Calculate delta theta if we dont have gyro
  double deltaTheta = (encoderRight_data.deltaDistance - encoderLeft_data.deltaDistance) / (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER + DISTANCE_LEFT_TRACKING_WHEEL_CENTER);

  // If we have gyro, get value and calculate delta
  #if GYRO_ENABLED 
  // Read gyro value
  gyro_data.currentDegrees = -gyroscope.get_heading(); // CCW positive
  gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);

  // Calculate delta
  gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
  
  // Normalize angle
  if (gyro_data.deltaDegrees >  180) gyro_data.deltaDegrees -= 360;
  if (gyro_data.deltaDegrees < -180) gyro_data.deltaDegrees += 360;
  
  gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);


        // std::cout << "Current distance | Prev value | Delta\n";
        // std::cout << encoderLeft_data.currentDistance << " | " << encoderLeft_data.previousDistance << " | " << encoderLeft_data.deltaDistance << "\n";   
        // std::cout << encoderRight_data.currentDistance << " | " << encoderRight_data.previousDistance << " | " << encoderRight_data.deltaDistance << "\n";   
        // std::cout << encoderBack_data.currentDistance << " | " << encoderBack_data.previousDistance << " | " << encoderBack_data.deltaDistance << "\n";   
        
  // Save current data for future calculations
  gyro_data.prevDegrees = gyro_data.currentDegrees;
        
  // Right now, confidence gyro 1.0, encoder confidence 0 (must sum 1) 
  deltaTheta = (1 - GYRO_CONFIDENCE) * deltaTheta + GYRO_CONFIDENCE * gyro_data.deltaRadians;
  #endif

  // Updating angle
  double thetaMid = getRadians() + deltaTheta / 2.0; // Theta where change happen
  setRadians(getRadians() + deltaTheta);
        
  /* ------------------ no math ------------------ */
  // Eliminate rotation induced component
  double dL_corr = encoderLeft_data.deltaDistance -  DISTANCE_LEFT_TRACKING_WHEEL_CENTER  * deltaTheta;
  double dR_corr = encoderRight_data.deltaDistance - DISTANCE_RIGHT_TRACKING_WHEEL_CENTER * deltaTheta;
  #if ENCODER_BACK_ENABLED
  double dS_corr = encoderBack_data.deltaDistance -  DISTANCE_BACK_TRACKING_WHEEL_CENTER  * deltaTheta;
  #endif

  double dx = (dL_corr + dR_corr) / 2.0;
  double dy = 0;
  #if ENCODER_BACK_ENABLED
  dy = dS_corr;
  #else
  changeMine.SetPosition(encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance / 2, 
                         0.0);
  #endif

  changeMine.SetPosition(changeMine.GetX() + (dx * cos(thetaMid) - dy * sin(thetaMid)), 
                         changeMine.GetY() + (dx * sin(thetaMid) + dy * cos(thetaMid)));
        
  pros::lcd::print(2, "no math: X: %0.3f | Y: %0.3f | H: %0.3f", changeMine.GetX(), changeMine.GetY(), changeMine.GetDegrees());

  // Calculations simple trigonometry
  // If we are rotating in the same place
  if (encoderLeft_data.deltaDistance * encoderRight_data.deltaDistance <= 0) {
    deltaDlocal.SetPosition(0.0, 0.0);
    #if ENCODER_BACK_ENABLED
    deltaDlocal.SetPosition(0.0, encoderBack_data.deltaDistance - DISTANCE_BACK_TRACKING_WHEEL_CENTER  * deltaTheta);
    #endif
  }
  // Else if we are rotating in a arc
  else if (std::abs(deltaTheta) > TURNING_THRESHOLD) {
    // Calculate change as an arc
    // Calculate the radius of rotation for each wheel
    double sign = (deltaTheta > 0) ? 1 : -1; 
    double radiusLeft  = (encoderLeft_data.deltaDistance / deltaTheta)  + sign * DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
    double radiusRight = (encoderRight_data.deltaDistance / deltaTheta) - sign * DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
    #if ENCODER_BACK_ENABLED
    double radiusBack = (encoderBack_data.deltaDistance / deltaTheta) - sign * DISTANCE_BACK_TRACKING_WHEEL_CENTER;
    std::cout << "Radius Right: " << radiusRight << ", Left: " << radiusLeft << ", Back: " << radiusBack << "\n";
    #endif
    std::cout << "Radius Right: " << radiusRight << ", Left: " << radiusLeft << "\n";
  
    // Calculate radius
    double averageR = (radiusLeft + radiusRight) / 2;
    pros::lcd::print(6, "Radius: %0.3f", averageR);
    // Update position using trigonometry
    
    deltaDlocal.SetPosition(averageR * std::sin(deltaTheta), averageR * (1 - std::cos(deltaTheta))); 
    
    #if ENCODER_BACK_ENABLED
    deltaDlocal.SetPosition(averageR * std::sin(deltaTheta), encoderBack_data.deltaDistance - DISTANCE_BACK_TRACKING_WHEEL_CENTER * deltaTheta); 
    #endif
  }
        
  // Else if the robot is moving straight forward or backward or sideways, average encoder values for distance    
  else {
    // std::cout << "Not turning\n";
    double dL_corr = encoderLeft_data.deltaDistance - DISTANCE_LEFT_TRACKING_WHEEL_CENTER * deltaTheta;
    double dR_corr = encoderRight_data.deltaDistance - DISTANCE_RIGHT_TRACKING_WHEEL_CENTER * deltaTheta;
    double deltaD = (dL_corr + dR_corr) / 2.0; // movement in X axis
    deltaDlocal.SetPosition(deltaD, 0);
    
    // If we have encoder back
    #if ENCODER_BACK_ENABLED 
    double deltaY = encoderBack_data.deltaDistance - DISTANCE_BACK_TRACKING_WHEEL_CENTER * deltaTheta;
    deltaDlocal.SetPosition(deltaD, deltaY);  
    #endif
  }

  // Updating global position using 2D matrix transformation (previous way to update to global coordinates)
  position.SetPosition(getX() + deltaDlocal.GetX() * std::cos(thetaMid) - deltaDlocal.GetY() * std::sin(thetaMid), 
                       getY() + deltaDlocal.GetX() * std::sin(thetaMid) + deltaDlocal.GetY() * std::cos(thetaMid));

  std::cout << "X: " << getX() << ", Y: " << getY() << ", T: " << getDegrees() << "\n";
  pros::lcd::print(1, "X: %0.3f | Y: %0.3f | H: %0.3f", getX(), getY(), getDegrees());

  // Save current values as previous for future updates
  encoderLeft_data.prevValue = encoderLeft_data.currentValue;
  encoderRight_data.prevValue = encoderRight_data.currentValue;

  encoderRight_data.previousDistance = encoderRight_data.currentDistance;
  encoderLeft_data.previousDistance = encoderLeft_data.currentDistance;
  #if ENCODER_BACK_ENABLED
  encoderBack_data.previousDistance = encoderBack_data.currentDistance;
  #endif
}

/// @brief resets Odometry values using the particular parameters
/// @param x X position in \b inches
/// @param y Y position in \b inches
/// @param theta Angular position in \b degrees
void Odometry::resetCurrent(double x, double y, double theta) {
  const double currentAngleRight = encoderRight.get_position() / 100.0;
  const double currentAngleLeft = encoderLeft.get_position() / 100.0;
  #if ENCODER_BACK_ENABLED
  const double currentAngleBack = encoderBack.get_position() / 100.0;
  #endif
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
  #if ENCODER_BACK_ENABLED
  encoderBack_data = {
    currentAngleBack,                     // current position in degrees
    currentAngleBack,                     // previous position in degrees
    0,                                    // delta in degrees
    currentAngleBack * conversionFactor,  // current position in inches
    currentAngleBack * conversionFactor,  // previous position in inches
    0.0};                                 // delta in inches
  #endif
  
  gyro_data = {0,     // current value degrees
               0,     // previous value degrees
               0,     // current radians
               0.0,   // delta degrees
               0.0};  // delta radians

  // Preset odometry values
  deltaTheta = 0.0;
  deltaDlocal.SetPosition(0.0, 0.0);

  // Other odometry we could use, less calculations
  changeMine.SetPosition(0.0, 0.0);

  setDegrees(theta);
  setPosition(x, y);
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


// Testing 


/**
* \brief Simple debug function that prints odometry values
*
* \details Blocking function that helps check if there are any issues with
* odometry
*
* \note Requires initialize pros::lcd and calling the odometry::Initialize
*       function
* */
void Odometry::debug(){ 
  while (true) {
    // pros::lcd::print(1, "X: %0.3f", GetX());
    // pros::lcd::print(2, "Y: %0.3f", GetY());
    // pros::lcd::print(0, "X: %0.3f, Y: %0.3f", GetX(), GetY());
    pros::lcd::print(0, "Left : %0.3f, %0.3f, %0.3f", encoderLeft_data.currentDistance, encoderLeft_data.previousDistance, encoderLeft_data.deltaDistance);
    pros::lcd::print(1, "Right: %0.3f, %0.3f, %0.3f", encoderRight_data.currentDistance, encoderRight_data.previousDistance, encoderRight_data.deltaDistance);
    #if ENCODER_BACK_ENABLED
    pros::lcd::print(2, "Back: %0.3f, %0.3f, %0.3f", encoderBack_data.currentDistance, encoderBack_data.previousDistance, encoderBack_data.deltaDistance);
    #endif
    pros::lcd::print(3, "Heading: %0.3f", getDegrees());
    pros::lcd::print(4, "Original: X: %0.3f | Y: %0.3f", getX(), getY());
    pros::lcd::print(5, "Not math: X: %0.3f | Y: %0.3f", changeMine.GetX(), changeMine.GetY());
    // pros::lcd::print(5, "Web:        X: %0.3f | Y: %0.3f", changeWeb.GetX(), changeWeb.GetY()); 
    // pros::lcd::print(6, "No Math:    X: %0.3f | Y: %0.3f", changeMine.GetX(), changeMine.GetY()); 
    // pros::lcd::print(7, "Video:       X: %0.3f | Y: %0.3f", changeVideo.GetX(), changeVideo.GetY()); 

    update();
    pros::delay(20);
  }
}

}  // namespace aon
