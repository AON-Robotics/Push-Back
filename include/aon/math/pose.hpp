#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../odometry/odometry.hpp"

#include "../math/misc/misc.hpp"
#include "../controls/pid/pid.hpp"
#include <cfloat> 

namespace aon{
    class Pose {
 public:
  /// @brief Position of the robot on the x-axis in \b `inches` with respect to the field using (0,0) as the center of the field
  double x;
  /// @brief Position of the robot on the y-axis in \b `inches` with respect to the field using (0,0) as the center of the field
  double y;
  /// @brief Orientation of the robot in \b `radians` with respect to angle 90º in the VEX Field
  double theta;

  Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
};
}
