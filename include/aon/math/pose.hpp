#pragma once

namespace aon {

class Pose {
 public:
  /// @brief Position of the robot on the x-axis in \b `inches` with respect to
  /// the field using (0,0) as the center of the field
  double x;
  /// @brief Position of the robot on the y-axis in \b `inches` with respect to
  /// the field using (0,0) as the center of the field
  double y;
  /// @brief Orientation of the robot in \b `degrees` with respect to angle 90º
  /// in the VEX Field
  double theta;

  Pose(double x = 0, double y = 0, double theta = 0)
      : x(x), y(y), theta(theta) {}
    

  Pose operator+(const Pose& other) const {
    return Pose(x + other.x, y + other.y, theta + other.theta);
  }

  Pose operator-(const Pose& other) const {
    return Pose(x - other.x, y - other.y, theta - other.theta);
  }

  Pose operator*(const double& scalar) const {
    return Pose(x * scalar, y * scalar, theta * scalar);
  }

  Pose operator/(const double& scalar) const {
    return Pose(x / scalar, y / scalar, theta / scalar);
  }


  Pose& operator+=(const Pose& other) {
    x += other.x; y += other.y; theta += other.theta;
    return *this;
  }

  Pose& operator-=(const Pose& other) {
    x -= other.x; y -= other.y; theta -= other.theta;
    return *this;
  }


  bool operator==(const Pose& other) const {
    return x == other.x && y == other.y && theta == other.theta;
  }

  bool operator!=(const Pose& other) const {
    return !(*this == other);
  }

  /// @brief Computes the Euclidean distance between this pose and another pose (ignores orientation).
  /// @param other The target pose to measure distance to.
  /// @return Distance in inches between the two positions.
  double distanceTo(const Pose& other) const {
    return std::hypot(other.x - x, other.y - y);
  }

  /// @brief Computes a combined error metric between this pose and another pose.
  /// @param other The target pose to compare against.
  /// @return Sum of absolute differences in x, y, and theta (|dx| + |dy| + |dtheta|).
  double aggregateError(const Pose& other) const {
    return std::abs(other.x - x) + std::abs(other.y - y) + std::abs(other.theta - theta);
  }
};
}  // namespace aon
