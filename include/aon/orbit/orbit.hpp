#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"
#include "../constants.hpp"
#include "../controls/pid/pid.hpp"
#include "../tools/vector.hpp"

namespace aon {
enum Colors { RED = 1, BLUE, STAKE };

class Orbit {
 private:
  pros::Rotation encoder;
  pros::Vision vision_sensor;
  const double height = 12.5;
  const bool limited = true;
  const int leftLimit = 210;
  const int rightLimit = 90;
  okapi::Motor motor;
  aon::PID PID = aon::PID(0.25, 0, 0);

  Colors COLOR = RED;

  volatile bool following = false;
  volatile bool braking = true;
  volatile bool scanning = false;
  pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(
      RED, 8973, 11143, 10058, -2119, -1053, -1586, 5.4, 0);
  pros::vision_signature_s_t BLUE_SIG = pros::Vision::signature_from_utility(
      BLUE, -3050, -2000, -2500, 8000, 11000, 9500, 5.4, 0);
  pros::vision_signature_s_t STAKE_SIG = pros::Vision::signature_from_utility(
      STAKE, -2247, -1833, -2040, -5427, -4727, -5077, 4.600, 0);

 public:

  Orbit();
  Orbit(int rotationPort, bool reversedEncoder, int visionPort, int port);


  auto getLargestObject() {
    return this->vision_sensor.get_by_sig(0, this->getColor());
  }

  ///@brief returns the leftlimit (private attribute)
  const int getLeftLimit() { return leftLimit; }

  ///@brief returns the rightlimit (private attribute)
  const int getRightLimit() { return rightLimit; }

  ///@brief sets the color
  void setColor(Colors color) { this->COLOR = color; }

  /// @brief returns the angle using the encoder
  double getAngle() { return (this->encoder.get_angle()) / 100; }

  /// @brief returns height (private attribute)
  const double getHeight() { return height; }

  ///@brief Returns COLOR
  Colors getColor() { return COLOR; }

  /// @brief  Returns true if the orbit IS following
  bool isFollowing() { return following; };

  /// @brief  Returns true if the orbit IS braking
  bool isBraking() { return braking; };

  /// @brief  Returns true if the orbit IS scanning
  bool isScanning() { return scanning; };

  /// @brief  Adds the colors to the vision sensor
  void configure();

  /// @brief Begins ORBIT following cycle
  void activateFollow();

  /// @brief Ends ORBIT following cycle
  void deactivateFollow();

  /// @brief Begins ORBIT scanning cycle
  void activateScan();

  /// @brief Ends ORBIT scanning cycle
  void deactivateScan();

  /// @brief Sets the ORBIT to brake if not scanning
  void brake();

  /// @brief Releases the ORBIT from braking to allow other functions to use it
  void release();

  /// @brief Async task to align ORBIT only to the item with the globally set
  /// `COLOR` signature
  void follow();

  /// @brief Rotates the ORBIT a given angle, starting from wherever it
  /// currently is. (Relative Rotation)
  /// @param givenAngle Angle in degrees we wish to rotate ORBIT.
  /// @details `turretEncoder.get_angle()` is divided by 100 for scaling
  /// purposes.
  void rotateRelative(const double &givenAngle);

  /// @brief Rotates the ORBIT to a given angle, with respect to 0 degrees
  /// facing forward. (Absolute Rotation)
  /// @param targetAngle Angle in degrees we wish to rotate ORBIT. within [-180,
  /// 180] or [0, 360]
  /// @details `turretEncoder.get_angle()` is divided by 100 for scaling
  /// purposes.
  void rotateAbsolute(double targetAngle);

  // Testing:

  /// @brief ORBIT async task scanning test function
  // To scan, make the ORBIT go from one side of its maximum FOV to the other,
  // if the ORBIT is not limited, make it go from 175° to 185° (going the long
  // way) if at any point the ORBIT detects an object, start following it and
  // stop scanning
  void scan();

  /// @brief checks if the turret is alligned (is facing foward)
  bool isAligned(const double &tolerance);

  /// @brief returns the amount for which the turret is not alligned
  double difference();

  /// @brief Calculates the distance the robot would have to travel to get to an
  /// object
  /// @param pixels The pixels reported by the vision sensor viewing an object
  /// (preferably width of that object)
  /// @return The distance in \b inches that the robot is from the object,
  /// probably to pass into the `move()` function
  double groundDistanceToDisk(const double &pixels);

  /// @brief Converts the amount of `pixels` seen from the vision sensor, to the
  /// corresponding \b inches
  /// @param pixels The pixels reported by the vision sensor viewing an object
  /// (preferably width of that object)
  /// @return The corresponding amount of \b inches
  double pixelsToInches(const double &pixels);

  /// @brief Calculates the distance in \b inches of an object based on its
  /// width in \b pixels from the vision sensor
  /// @param width The width of the object detected by the sensor in \b pixels
  /// @return The distance from the vision sensor to the object in \b inches
  /// @note This function assumes the entire object is in view, this may be
  /// changed later
  /// @details The funcion uses `pixelsToInches()` as a crucial part of the
  /// calculations
  /// @details The math is explained inside and the formulas are from optical
  /// geometry
  double widthToDistance(const double &width);

  /// @brief Calculates the distance to a ring of the specified `color` using a EKF
  /// @param color The color of the ring we wish to track
  /// @return The filtered distance to that ring
  /// @note Takes half a second (0.5s) to complete
  double getDistanceToRing(Colors color);

  /// @brief Stops the motor to avoid damage
  void stop();
};
}  // namespace aon
