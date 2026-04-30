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
    
  // TODO: implement operators (specifically `operator-` to get errors between all attributes)
};
}  // namespace aon
