#pragma once

#include <algorithm>
#include <cmath>
#include "../../constants.hpp"
#include "../../tools/vector.hpp"
#include "../pose.hpp"

namespace aon::math {

/// @brief Determines the linear speed of the robot given drivetrain motors' RPM
/// @param rpm The RPM for which to calculate the velocity (default current RPM)
/// @returns The speed in \b in/s at which the robot would move at the given RPM
/// @note Test the accuracy precision of the `getActualVelocity()` method,
/// @note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
/// @note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`.
inline double linearSpeed(const double& rpm = MAX_RPM) {
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double rps = rpm / 60;
  double speed = MOTOR_TO_DRIVE_RATIO * circumference * rps;
  return speed;
}

/// @brief Determines the rotational speed of the robot given drivetrain motors' RPM
/// @param rpm The RPM for which to calculate the velocity (default current RPM)
/// @return The speed in \b deg/s at which the robot would move at the given RPM
inline double rotationalSpeed(const double& rpm) {
  const double drive_width = 10.5;
  const double drive_length = 8.25;
  const double ROBOT_RADIUS = hypot(drive_width, drive_length) / 2;
  double wheelCircumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double rps = rpm / 60;
  double tangentialSpeed = MOTOR_TO_DRIVE_RATIO * wheelCircumference * rps;
  return (tangentialSpeed * 180 / M_PI) / ROBOT_RADIUS;
}

/// @brief Calculates time for the robot to reach a given distance
/// @param distance Distance from the robot to the target (remains constant) in \b inches
/// @returns The approximate time necessary to reach the target (overestimation) in \b seconds
inline double estimateTimetoTarget(const double& distance,
                                   const double& rpm = MAX_RPM) {
  double time = 4 * distance / linearSpeed(rpm);
  return time;
}

/// @brief Calculates time for the robot to turn an angle
/// @param radians Angle remaining from the robot's current angle to the target (remains constant) in \b radians
/// @details The arc length formula is used as s = theta * radius (theta in radians)
/// @returns The approximate time necessary to reach the target (overestimation) in \b seconds
inline double getTimetoTurnRad(const double& radians,
                               const double& rpm = MAX_RPM / 4) {
  double arcLength = radians * AVG_DRIVETRAIN_RADIUS;  // Of the turn (inches)
  double time = 6 * arcLength / linearSpeed(rpm);  // Calculated time (seconds)
  return time;
}

/// @brief Calculates time for the robot to turn an angle
/// @param degrees Angle remaining from the robot's current angle to the target (remains constant) in \b degrees
/// @returns The approximate time necessary to reach the target (overestimation) in \b seconds
inline double getTimetoTurnDeg(const double& degrees) {
  return getTimetoTurnRad(degrees * M_PI / 180);
}

/// @brief Calculates the angle of a `point` with respect to a circle with a given `center` in the range [0, 360)
/// @param point The point whose angle in the circle we want to calculate
/// @param center The center of the circle in which the point resides
/// @return The angle of that `point` in the circle in \b degrees
inline double getAngleInCircle(Vector point, Vector center) {
  if (point == center) {
    return 0;
  }  // Should never happen

  // Avoid 0 division
  if (point.GetX() == center.GetX() && point.GetY() > center.GetY()) {
    return 90;
  } else if (point.GetX() == center.GetX() && point.GetY() < center.GetY()) {
    return 270;
  }
  // Check edge cases
  else if (point.GetY() == center.GetY() && point.GetX() > center.GetX()) {
    return 0;
  } else if (point.GetY() == center.GetY() && point.GetX() < center.GetX()) {
    return 180;
  }

  // Get the angle
  const double theta = std::atan((point.GetY() - center.GetY()) /
                                 (point.GetX() - center.GetX())) *
                       180 / M_PI;

  // Normalize the angle depending on the quadrant its on
  double normalizer = 0;

  // First quadrant
  if (point.GetX() - center.GetX() > 0 && point.GetY() - center.GetY() > 0) {
    normalizer = 0;
  }
  // Second quadrant
  else if (point.GetX() - center.GetX() < 0 &&
           point.GetY() - center.GetY() > 0) {
    normalizer = 180;
  }
  // Third quadrant
  else if (point.GetX() - center.GetX() < 0 &&
           point.GetY() - center.GetY() < 0) {
    normalizer = 180;
  }
  // Fourth quadrant
  else if (point.GetX() - center.GetX() > 0 &&
           point.GetY() - center.GetY() < 0) {
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
inline double getAngleOfArc(const Vector& a, const Vector& b,
                            const Vector& center) {
  const double theta_a = getAngleInCircle(a, center);
  const double theta_b = getAngleInCircle(b, center);

  return std::max(theta_a, theta_b) - std::min(theta_a, theta_b);
}

/// @brief Conversion from \b meters to \b inches
/// @param meters The \b meters to be converted
/// @returns The distance in \b inches
inline double metersToInches(const double& meters) { return meters * 39.3701; }

/// @brief Conversion from \b inches to \b meters
/// @param inches The \b inches to be converted
/// @returns The distance in \b meters
inline double inchesToMeters(const double& inches) { return inches / 39.3701; }

/// @brief Calculates the error percentage for an actual given the expected value
/// @param expected Usually a calculated value
/// @param actual Usually the measured value
/// @returns The error percentage for the measured value
inline double getErrorPercentage(const double& expected, const double& actual) {
  return ((expected - actual) / expected) * 100;
}

/// @brief Calculate percent difference of two values
/// @param a Value of one number
/// @param b Valuer of the other number
/// @return The percent difference between them
inline double getPercentDifference(const double& a, const double& b) {
  return (std::abs(a - b) / ((a + b) / 2)) * 100;
}

/// @brief Gets the distance between two points in the field
/// @param target The target location
/// @param current The current location
/// @returns The distance between the two points
inline double findDistance(Vector target, Vector current) {
  double distInMeters = (target - current).GetMagnitude();
  return metersToInches(distInMeters);
}

/// @brief Determines the angle needed to be turned in order to face a specific point in the field
/// @param target The point we wish to face
/// @param current Where the robot is now
/// @returns The angle the robot needs to turn in order to face the target location
/// @note The result must be passed into functions such as `drivetrain.turn()` and `drivetrain.turnPID()` as negative because of the GPS convention
inline double calculateTurn(Vector target, Pose current) {
  Vector position = Vector().SetPos(current.x, current.y);
  // Get and change the heading to the common cartesian plane
  double heading = 90 - current.theta;

  // Limiting the heading to the 0-360 range
  if (heading < 0)
    heading += 360;
  else if (heading > 360)
    heading -= 360;

  // This number is in respect to the common cartesian plane if odometry
  // position is used
  double toTarget = (target - position).GetDegrees();

  // Limiting the the target to the 0-360 range
  if (toTarget < 0)
    toTarget += 360;
  else if (toTarget >= 360)
    toTarget -= 360;

  double angle = toTarget - heading;  // Calculate the angle to turn

  // Limiting the heading to the -180-180 range
  if (angle > 180)
    angle -= 360;
  else if (angle < -180)
    angle += 360;

  return angle;
}

/// @brief Estimates the length of a `std::vector` of `aon::Pose`s
/// @param path The `std::vector` of `aon::Pose`s to consider
/// @return The euclidean length of the `path` given
inline double length(const std::vector<Pose>& path) {
  if (path.size() < 2) return 0;

  double result = 0;
  for(size_t i = 0; i < path.size() - 1; i++) {
    result += path[i].distanceTo(path[i + 1]);
  }
  return result;
}

}  // namespace aon::math
