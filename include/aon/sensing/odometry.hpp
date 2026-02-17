#pragma once

#ifndef AON_SENSING_ODOMETRY_HPP_
#define AON_SENSING_ODOMETRY_HPP_

#include <cmath>
#include "../constants.hpp"
#include "../../api.h"
#if GYRO_ENABLED
#include "../../okapi/api.hpp"
#endif
#include "../tools/vector.hpp"
/**
 * \namespace aon::odometry
 *
 * \brief Odometry namespace to calculate the position we are in the field
 *
 * \par Requisites:
 *    1. Declare the following global constants in `constants.hpp`:
 *       - TRACKING_WHEEL_DIAMETER (measure this)
 *       - DISTANCE_RIGHT_ENCODER (measure this)
 *       - DISTANCE_LEFT_ENCODER (measure this)
 *       - DISTANCE_BACK_TRACKING_WHEEL_CENTER (measure this)
 *       - GYRO_CONFIDENCE (testing)    } should
 *       - ENCODER_CONFIDENCE (testing) } sum 1
 *       - DEGREES_PER_REVOLUTION    (= 360)
 *       - INITIAL_ODOMETRY_X ( = 0)
 *       - INITIAL_ODOMETRY_Y ( = 0)
 *       - INITIAL_ODOMETRY_THETA ( = 0)
 *       - GYRO_ENABLED ( = true | = false)
 *    2. Have available the `vector.hpp` header file
 *    3. Have available pros and okapilib
 *    4. Have available the `globals.hpp` header file with `encoderMid`,
 *    and `encoderBack` objects instantiated and `gyroscope` if
 *    the GYRO_ENABLED is true.
 *
 *  \par Instructions
 *    1. Call the `Initialize` function
 *    2. Call the `Update` function as frequently as possible to calculate the
 *    pose.
 *    3. Coordinate gyro with encoder back. Make sure when turning left, they are
 *       positive.
 * */
namespace aon::odometry {

#if USING_BIG_ROBOT
inline pros::Rotation encoderRight(4, true);
inline pros::Rotation encoderLeft(12, false);
inline pros::Rotation encoderBack(2, true);
#else
inline pros::Rotation encoderRight(1, false);
inline pros::Rotation encoderLeft(9, true);
inline pros::Rotation encoderBack(21, false);
#endif
#if GYRO_ENABLED
#if USING_BIG_ROBOT
inline pros::Imu gyroscope(11);
#else
inline pros::Imu gyroscope(20);
#endif
#endif
inline pros::Gps gps(0, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET);

  // ============================================================================
  //   __   __        _      _    _
  //   \ \ / /_ _ _ _(_)__ _| |__| |___ ___
  //    \ V / _` | '_| / _` | '_ \ / -_|_-<
  //     \_/\__,_|_| |_\__,_|_.__/_\___/__/
  //
  // ============================================================================

  /**
  * \struct STRUCT_encoder
  *
  * \brief Store encoder data from current and previous odometry
  * iterations.
  * */
  struct STRUCT_encoder {
    //> Values in centidegrees
    double currentValue;     //> Current value in degrees
    double prevValue;        //> Previous value in degrees
    double delta;            //> previous_value - current_value

    //> All this 3 variables are in inches
    double currentDistance;  //> Distance that tracking wheel has traveled
    double previousDistance; //> Previous distance that tracking wheel has traveled
    double deltaDistance;    //> previous_distance - current_distance
  };

  /**
   * \struct STRUCT_encoder
   *
   * \brief Store gyro data from current and previous odometry
   * iterations.
   * */
  struct STRUCT_gyro {
    double currentDegrees;
    double prevDegrees;

    double currentRadians;
      
    double deltaRadians;
    double deltaDegrees;
  };
    
  //> Calculate delta angle each iteration using tracking wheel data
  inline double deltaTheta;
  //> Stores the change in position in local reference plane
  inline Vector deltaDlocal;
  //> Final calculated orientation in both \b radians and \b degrees
  inline Angle orientation;
  //> Final calculated position in \b inches
  inline Vector position;
  //> Conversion factor
  inline const double conversionFactor = M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;
    
  //> Mutex for absolute position
  inline pros::Mutex p_mutex;
  //> Mutex for orientation to prevent race condition when retrieving value
  inline pros::Mutex orientation_mutex;

  // from the web
  inline Vector changeWeb;

  //> Encoder back struct instance
  inline STRUCT_encoder encoderBack_data;
  //> Encoder right struct instance
  inline STRUCT_encoder encoderRight_data;
  //> Encoder left struct instance
  inline STRUCT_encoder encoderLeft_data;
  //> Gyro struct instance
  inline STRUCT_gyro gyro_data;

  // ============================================================================
  //     ___     _   _                __       ___      _   _
  //    / __|___| |_| |_ ___ _ _ ___ / _|___  / __| ___| |_| |_ ___ _ _ ___
  //   | (_ / -_)  _|  _/ -_) '_(_-< > _|_ _| \__ \/ -_)  _|  _/ -_) '_(_-<
  //    \___\___|\__|\__\___|_| /__/ \_____|  |___/\___|\__|\__\___|_| /__/
  //
  // ============================================================================

  /**
   * \brief Get current X position in \b inches
   *
   * \returns Returns current X position in \b inches
   */
  inline double GetX() {
    p_mutex.take(1);
    const double currentX = position.GetX();
    p_mutex.give();
    return currentX;
  }
  
  /**
   * \brief Get current Y position in \b inches
   *
   * \returns Returns current Y position in \b inches
   */
  inline double GetY() {
    p_mutex.take(1);
    const double currentY = position.GetY();
    p_mutex.give();
    return currentY;
  }
  
  /**
   * \brief Set Y position in \b inches
   * 
   * \param value Input value to set current Y
   */

  inline void SetPosition(double x, double y) {
    p_mutex.take(1);
    position.SetPosition(x, y);
    p_mutex.give();
  }

  /**
   * \brief Get current pose's angle in \b degrees
   *
   * \returns Returns current pose's angle in \b degrees
   */
  inline double GetDegrees() {
    orientation_mutex.take(1);
    const double currentDegrees = orientation.GetDegrees();
    orientation_mutex.give();
    return currentDegrees;
  }
  
  /**
   * \brief Set current pose's angle in \b degrees
   *
   * \param degrees Input value to set the current angle to
   */
  inline void SetDegrees(const double degrees) {
    orientation_mutex.take(1);
    orientation.SetDegrees(degrees);
    orientation_mutex.give();
    deltaTheta = 0.0;
  }
  
  /**
   * \brief Get current pose's angle in \b radians
   *
   * \returns Returns current pose's angle in \b radians
   */
  inline double GetRadians() {
    orientation_mutex.take(1);
    const double currentRadian = orientation.GetRadians();
    orientation_mutex.give();
    return currentRadian;
  }
  
  /**
   * \brief Set current pose's angle in \b radians
   *
   * \param radians Input value to set the current angle to
   *
   * \warning Sets angles in units of \b radians. INPUT MUST BE IN \b RADIANS
   */
  inline void SetRadians(const double radians) {
    orientation_mutex.take(1);
    orientation.SetRadians(radians);
    orientation_mutex.give();
    // deltaTheta = 0.0;
  }

  /**
   * \brief Get a vector with the current position
   * 
   * \return Returns new vector with current position
   */
  inline Vector GetPosition() {
    p_mutex.take(1);
    Vector pos = position;
    p_mutex.give();
    return pos;
  }  

  // ============================================================================
//    __  __      _        ___             _   _
//   |  \/  |__ _(_)_ _   | __|  _ _ _  __| |_(_)___ _ _  ___
//   | |\/| / _` | | ' \  | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_|  |_\__,_|_|_||_| |_| \_,_|_||_\__|\__|_\___/_||_/__/
//
// ============================================================================

/**
 * \brief resets Odometry values using the particular parameters
 *
 * \param x X position in \b inches
 * \param y Y position in \b inches
 * \param theta Angular position in \b degrees
 */
inline void ResetCurrent(const double x, const double y, const double theta) {

  const double currentAngleRight = encoderRight.get_position() / 100.0;
  const double currentAngleLeft = encoderLeft.get_position() / 100.0;
  const double currentAngleBack = encoderBack.get_position() / 100.0;
  const double currentAngleGyro = gyroscope.get_heading();
  std::cout << "currentAngleGyro: " << currentAngleGyro << "\n";

  
  // Reset encoder's struct variables
  encoderRight_data = {currentAngleRight,                     // current position in degrees
                       currentAngleRight,                     // previuos position in degrees
                       0,                                     // delta in degrees
                       currentAngleRight * conversionFactor,  // current position in inches 
                       currentAngleRight * conversionFactor,  // previuos position in inches
                       0.0};                                  // delta in inches
    
  encoderLeft_data = {currentAngleLeft,                       // current position in degrees
                      currentAngleLeft,                       // previuos position in degrees
                      0,                                      // delta in degrees
                      currentAngleLeft * conversionFactor,    // current position in inches 
                      currentAngleLeft * conversionFactor,    // previuos position in inches
                      0.0};                                   // delta in inches

  encoderBack_data = {currentAngleBack,                       // current position in degrees
                      currentAngleBack,                       // previuos position in degrees
                      0,                                      // delta in degrees
                      currentAngleBack * conversionFactor,    // current position in inches 
                      currentAngleBack * conversionFactor,    // previuos position in inches
                      0.0};                                   // delta in inches

  gyro_data = {0,                                             // current value degrees
               0,                                             // prevuios value degrees
               0,                                             // current radians
               0.0,                                           // delta degrees
               0.0};                                          // delta radians
    
  // Preset odometry values
  deltaTheta = 0.0;
  deltaDlocal.SetPosition(0.0, 0.0);

  // Other odometry we could use, less calculations
  changeWeb.SetPosition(0.0, 0.0);
  
  SetDegrees(theta);
  SetPosition(x, y);
  #if GYRO_ENABLED
  gyroscope.tare();
  pros::delay(3000);
  #endif

}

//> Resets the Odometry values with `INITIAL_ODOMETRY_X`,Y and T constants.
inline void ResetInitial() {
  /*
    ATTENTION
    We need to know where the robot start (coordinates), for the odometry knows where the
    robot is at all times. Maybe using gps or a const variable
  */
  // Normal initial
  ResetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}


/**
 * \brief Initialization function to put everything to 0
 */
inline void Initialize() {
  
  encoderLeft.set_position(0);
  encoderRight.set_position(0);
  encoderBack.set_position(0);


  encoderLeft.reset();
  encoderRight.reset();
  encoderBack.reset();

  
  // Set initial position with gps (need test with field)
  // INITIAL_ODOMETRY_X = gps.get_x_position();
  // INITIAL_ODOMETRY_Y = gps.get_y_position();
    
  ResetInitial();
}

/**
 * \brief Fundamental function for Odometry.
 *
 * \details Uses changes in encoder (right and left) and gyro to calculate position
 * 
 * */

inline void Update() {
  /// Read encoder values, divided by 100 to convert centidegrees to degrees
  encoderRight_data.currentValue = encoderRight.get_position() / 100.0; 
  encoderLeft_data.currentValue = encoderLeft.get_position() / 100.0; 
  // encoderBack_data.currentValue = encoderBack.get_position() / 100.0;

  // Convert to distances
  encoderRight_data.currentDistance = encoderRight_data.currentValue * conversionFactor;
  encoderLeft_data.currentDistance = encoderLeft_data.currentValue * conversionFactor;
  // encoderBack_data.currentDistance = encoderBack_data.currentValue * conversionFactor;

  // Calculate deltas
  encoderRight_data.delta = encoderRight_data.currentValue - encoderRight_data.prevValue;
  encoderLeft_data.delta = encoderLeft_data.currentValue - encoderLeft_data.prevValue;
  // encoderBack_data.delta = encoderBack_data.currentValue - encoderBack_data.prevValue;

  encoderRight_data.deltaDistance = encoderRight_data.currentDistance - encoderRight_data.previousDistance;
  encoderLeft_data.deltaDistance = encoderLeft_data.currentDistance - encoderLeft_data.previousDistance;
  // encoderBack_data.deltaDistance = encoderBack_data.currentDistance - encoderBack_data.previousDistance;

  // Calculate delta theta if we dont have gyro
  double deltaTheta = (encoderLeft_data.deltaDistance - encoderRight_data.deltaDistance) / (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER + DISTANCE_LEFT_TRACKING_WHEEL_CENTER);

  // If we have gyro, get value and calculate delta
  #if GYRO_ENABLED 
  // Read gyro value
  gyro_data.currentDegrees = gyroscope.get_heading();
  gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);

  // Normalize angle to prevent overshoot when using turn function
  if (gyro_data.currentDegrees > 180) {
    gyro_data.currentDegrees -= 360;
  }
  else if (gyro_data.currentDegrees <= -180) {
    gyro_data.currentDegrees += 360;
  }

  // Calculate delta
  gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;  
  gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);

  // Save current data for future calculations
  gyro_data.prevDegrees = gyro_data.currentDegrees;
  
  // Right now, confidence gyro 1.0, encoder confidence 0 (must sum 1) 
  //# deltaTheta = (1 - GYRO_CONFIDENCE) * deltaTheta + GYRO_CONFIDENCE * gyro_data.deltaDegrees;
  deltaTheta = (1 - GYRO_CONFIDENCE) * deltaTheta + GYRO_CONFIDENCE * gyro_data.deltaRadians;
  #endif
  
  // Updating angle
  double previousTheta = GetRadians();
  //# SetDegrees(GetDegrees() + deltaTheta);
  SetRadians(GetRadians() + deltaTheta);
  
  // Calculations simple trigonometry, i.e., mine :)
  // If we are rotating
  if (std::abs(deltaTheta * (180/M_PI)) > 0.01) {
    // If turning in its own axis
    if ((encoderLeft_data.deltaDistance * encoderRight_data.deltaDistance) <= 0) {
      deltaDlocal.SetPosition(0.0, 0.0);
    }
    // If we are going in a arc
    else {
      // Calculate the radius of rotation for each wheel
      double sign = (deltaTheta > 0) ? 1 : -1; 
      double radiusLeft  = (encoderLeft_data.deltaDistance / deltaTheta)  - sign * DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
      double radiusRight = (encoderRight_data.deltaDistance / deltaTheta) + sign * DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
      
      // Calculate radius
      double averageR = (radiusLeft + radiusRight) / 2;
      
      // Update position using trigonometry
      deltaDlocal.SetPosition(averageR * std::sin(deltaTheta), averageR * (1 - std::cos(deltaTheta)));      
    }
  }
  // If the robot is moving straight forward or backward, average encoder values for distance    
  else {
    double deltaD = (encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / 2.0;
    deltaDlocal.SetPosition(deltaD, 0);
  } 
  
  // Odometry copy from https://medium.com/%40nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
  // Super accurate and use less calculations, but mine seems cooler :)
  double deltaD = (encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / 2.0;
  changeWeb.SetPosition(changeWeb.GetX() + (deltaD * cos(previousTheta + deltaTheta / 2)),
                        changeWeb.GetY() + (deltaD * sin(previousTheta + deltaTheta / 2)));


  // Updating global position using 2D matrix transformation (previous way to update to global coordinates)
  SetPosition(GetX() + deltaDlocal.GetX() * std::cos(GetRadians()) - deltaDlocal.GetY() * std::sin(GetRadians()), 
              GetY() + deltaDlocal.GetX() * std::sin(GetRadians()) + deltaDlocal.GetY() * std::cos(GetRadians()));  
              

  // Save current values as previous for future updates
  encoderLeft_data.prevValue = encoderLeft_data.currentValue;
  encoderRight_data.prevValue = encoderRight_data.currentValue;

  encoderRight_data.previousDistance = encoderRight_data.currentDistance;
  encoderLeft_data.previousDistance = encoderLeft_data.currentDistance;

  gyro_data.prevDegrees = gyro_data.currentDegrees;
}

/// @brief Returns position of the robot in the field
/// @returns The GPS coordinates as a `Vector`
inline Vector gpsPosition(){
  pros::delay(2000);
  pros::c::gps_status_s_t status = gps.get_status();
  Vector current = Vector().SetPosition(status.x, status.y);

  return current;
}

// ============================================================================
//   _______ _                        _ 
//  |__   __| |                      | |
//     | |  | |__  _ __ ___  __ _  __| |
//     | |  | '_ \| '__/ _ |/ _` |/ _` |
//     | |  | | | | | |  __| (_| | (_| |
//     |_|  |_| |_|_|  \___|\__,_|\__,_|
//
// ============================================================================
/**
 * \brief Function for odometry thread
 */
inline void Odometry(){
  while(true){
    Update();
    pros::delay(20);
  }
}

// ============================================================================
//    _____       _
//   |_   _|__ __| |_ ___
//     | |/ -_|_-<  _(_-<
//     |_|\___/__/\__/__/
//
// ============================================================================

/**
 * \brief Simple debug function that prints odometry values
 *
 * \details Blocking function that helps check if there are any issues with
 * odometry
 *
 * \note Requires initialize pros::lcd and calling the odometry::Initialize
 *       function
 * */
inline void Debug() {
  while (true) {
    // pros::lcd::print(1, "X: %0.3f", GetX());
    // pros::lcd::print(2, "Y: %0.3f", GetY());
    // pros::lcd::print(0, "X: %0.3f, Y: %0.3f", GetX(), GetY());
    pros::lcd::print(0, "Left : %0.3f, %0.3f, %0.3f", encoderLeft_data.currentDistance, encoderLeft_data.previousDistance, encoderLeft_data.deltaDistance);
    pros::lcd::print(1, "Right: %0.3f, %0.3f, %0.3f", encoderRight_data.currentDistance, encoderRight_data.previousDistance, encoderRight_data.deltaDistance);
    pros::lcd::print(2, "Heading: %0.3f", GetDegrees());
    pros::lcd::print(3, "Mine:   X: %0.3f | Y: %0.3f", GetX(), GetY());
    pros::lcd::print(4, "Web:    X: %0.3f | Y: %0.3f", changeWeb.GetX(), changeWeb.GetY());   

    odometry::Update();
    pros::delay(20);
  }
}

}  // namespace aon::odometry


namespace aon {

class Pose {
 public:
  /// @brief Position of the robot on the x-axis in \b `inches` with respect to
  /// the field using (0,0) as the center of the field
  double x;
  /// @brief Position of the robot on the y-axis in \b `inches` with respect to
  /// the field using (0,0) as the center of the field
  double y;
  /// @brief Orientation of the robot in \b `degrees` with respect to angle 0º
  /// in the VEX Field
  double theta;

  Pose(double x = 0, double y = 0, double theta = 0)
      : x(x), y(y), theta(theta) {}

  double distanceTo(Pose other) {
    return std::hypot(other.x - this->x, other.y - this->y);
  }

  double angleTo(Pose other) { return this->theta - other.theta; }

  Vector pathTo(Pose other) {
    return Vector().SetPosition(other.x - this->x,
                                other.y - this->y);
  }
};

/// @brief The sensing system for the position of the robot on the VEX field,
/// the reference pose (0,0,0) is located in the center of the field facing
/// towards the 0º marked wall of the field. A positive x change means going
/// towards that 0º marked wall; a positive y change means going towards the red
/// alliance, or the 270º marked wall; and a positive theta change means
/// rotating counterclockwise.
class Odometry {

  /// @brief Struct that contains current, previous and delta distances in \b inches from encoders
  struct EncoderData {
    double currentDistance;
    double previousDistance;
    double deltaDistance;
  };

/// @brief Struct that contains current, previous and delta distances in \b degrees from gryo
  struct GyroData {
    double currentDegrees;
    double prevDegrees;
    double deltaDegrees;
  };

 private:
  /// @brief To convert from degrees to inches (or whatever unit `TRACKING_WHEEL_DIAMETER` uses)
  const double conversionFactor = M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;
  
  pros::Rotation rightEncoder;
  pros::Rotation leftEncoder;
  pros::Rotation backEncoder;
  pros::IMU gyroscope;
  Pose pose;

  EncoderData rightData;
  EncoderData leftData;
  EncoderData backData;

  GyroData gyroData;

 public:
  Odometry(const short& rightPort = 0, const short& leftPort = 0,
           const short& backPort = 0, const short& gyroPort = 0)
      : rightEncoder(rightPort),
        leftEncoder(leftPort),
        backEncoder(backPort),
        gyroscope(gyroPort) {}

  Pose getPose() { return this->pose; }
  double getX() { return this->pose.x; }
  double getY() { return this->pose.y; }
  double getTheta() { return this->pose.theta; }

  void reset(const double& x, const double& y, const double& theta) {
    const double currentAngleRight = rightEncoder.get_position() / 100.0;
    const double currentAngleLeft = leftEncoder.get_position() / 100.0;
    const double currentAngleBack = backEncoder.get_position() / 100.0;
    const double currentAngleGyro = gyroscope.get_heading();

    // Reset encoder's struct variables
    rightData = {
        currentAngleRight * conversionFactor,  // current position in inches
        currentAngleRight * conversionFactor,  // previous position in inches
        0.0};                                  // delta in inches

    leftData = {
        currentAngleLeft * conversionFactor,  // current position in inches
        currentAngleLeft * conversionFactor,  // previous position in inches
        0.0};                                 // delta in inches

    backData = {
        currentAngleBack * conversionFactor,  // current position in inches
        currentAngleBack * conversionFactor,  // previous position in inches
        0.0};                                 // delta in inches

    gyroData = {0,      // current value degrees
                 0,     // previous value degrees
                 0.0};  // delta degrees

    this->pose = Pose(x, y, theta);
    gyroscope.reset(true);
  }

  void initialize() {
    this->reset(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
    while (true) {
      this->update();
      pros::delay(10);
    }
  }
  
  void update() {
    pros::lcd::print(1, "X: %.2f", this->getX());
    pros::lcd::print(2, "Y: %.2f", this->getY());
    pros::lcd::print(3, "Theta: %.2f", this->getTheta());
    // Read encoder values, divided by 100 to convert centidegrees to degrees and convert to distances
    rightData.currentDistance = rightEncoder.get_position() / 100 * conversionFactor;
    leftData.currentDistance = leftEncoder.get_position() / 100 * conversionFactor;
    backData.currentDistance = backEncoder.get_position() / 100 * conversionFactor;
    
    // Calculate deltas
    rightData.deltaDistance = rightData.currentDistance - rightData.previousDistance;
    leftData.deltaDistance = leftData.currentDistance - leftData.previousDistance;
    backData.deltaDistance = backData.currentDistance - backData.previousDistance;
    
    const double deltaThetaEncoders = ((backData.deltaDistance / DISTANCE_BACK_TRACKING_WHEEL_CENTER) + (rightData.deltaDistance / DISTANCE_RIGHT_TRACKING_WHEEL_CENTER) - (leftData.deltaDistance / DISTANCE_LEFT_TRACKING_WHEEL_CENTER)) / 3;
    
    //? The -1 is to flip for our convention of positive rotation being counterclockwise
    // TODO: check that that -1 does what it is supposed to
    gyroData.currentDegrees = -1 * gyroscope.get_rotation();
    gyroData.deltaDegrees = gyroData.currentDegrees - gyroData.prevDegrees;
    
    // Overall idea, displacement in x and displacement in y form a vector, which we rotate by the new heading, then we make that heading our current heading and add the x and y to the current position.
    const double xChange = (rightData.deltaDistance + leftData.deltaDistance) / 2;
    const double yChange = backData.deltaDistance;
    Vector change = Vector().SetPosition(xChange, yChange);
    
    //? is this line even needed
    const double deltaHeading = (gyroData.deltaDegrees * GYRO_CONFIDENCE) + (deltaThetaEncoders * (1 - GYRO_CONFIDENCE));
    
    change.SetDegrees(gyroData.currentDegrees);
    
    this->pose.x += change.GetX();
    this->pose.y += change.GetY();
    this->pose.theta = gyroData.currentDegrees;
    
    // Save current values as previous for future updates
    rightData.previousDistance = rightData.currentDistance;
    leftData.previousDistance = leftData.currentDistance;
    backData.previousDistance = backData.currentDistance;

    gyroData.prevDegrees = gyroData.currentDegrees;
  }

  long double averageDistanceForward() {
    return (rightEncoder.get_position() + leftEncoder.get_position()) / 2;
  }

  long double rotation() { return gyroscope.get_rotation(); }

  long double averageDistanceSideways() { return backEncoder.get_position(); }
};

}  // namespace aon


#endif  // AON_SENSING_ODOMETRY_HPP_
