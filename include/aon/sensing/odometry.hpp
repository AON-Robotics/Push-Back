#ifndef AON_SENSING_ODOMETRY_HPP_
#define AON_SENSING_ODOMETRY_HPP_

#include <cmath>
#include "../constants.hpp"
#include "../../api.h"
#if GYRO_ENABLED
#include "../../okapi/api.hpp"
#endif
#include "../tools/vector.hpp"
#include "../globals.hpp"

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
  double deltaTheta;
  //> Stores the change in position in local reference plane
  Vector deltaDlocal;
  //> Final calculated orientation in both \b radians and \b degrees
  Angle orientation;
  //> Final calculated position in \b inches
  Vector position;
  //> Conversion factor
  const double conversionFactor = M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;
    
  //> Mutex for absolute position
  pros::Mutex p_mutex;
  //> Mutex for orientation to prevent race condition when retrieving value
  pros::Mutex orientation_mutex;

  // from the web
  Vector changeWeb;

  // //> Encoder back struct instance
  STRUCT_encoder encoderBack_data;
  //> Encoder right struct instance
  STRUCT_encoder encoderRight_data;
  //> Encoder left struct instance
  STRUCT_encoder encoderLeft_data;
  
  //> Gyro struct instance
  STRUCT_gyro gyro_data;


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

void Update() {
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
  deltaTheta = (1 - GYRO_CONFIDENCE) * deltaTheta + GYRO_CONFIDENCE * gyro_data.deltaRadians;
  #endif
  
  // Updating angle
  double previousTheta = GetRadians();
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

#endif  // AON_SENSING_ODOMETRY_HPP_
