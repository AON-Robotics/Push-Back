#pragma once

#ifndef AON_AUTONOMY_TRAJECTORY_BUILDER_HPP_
#define AON_AUTONOMY_TRAJECTORY_BUILDER_HPP_

#include <vector>

#include "aon/controls/holonomic-pure-pursuit/hpp.hpp"

namespace aon::autonomy {

struct XYPoint {
  double x_in = 0.0;
  double y_in = 0.0;
};

struct TrajectoryBuildConfig {
  double rdp_eps_in = 0.25;
  int smooth_iterations = 14;
  double smooth_alpha = 0.22;
  double sample_ds_in = 1.0;

  // Optional footprint-vs-wall clearance guard.
  // When enabled, path points are clamped to:
  // [field_min + (robot_radius + wall_clearance), field_max - (...)].
  bool enforce_field_clearance = false;
  double field_min_x_in = -72.0;
  double field_max_x_in = 72.0;
  double field_min_y_in = -72.0;
  double field_max_y_in = 72.0;
  double robot_radius_in = 0.0;
  double wall_clearance_in = 0.0;
};

aon::hpp::Trajectory buildTrajectoryFromPoints(
    const std::vector<XYPoint> &points,
    double end_heading_rad,
    const TrajectoryBuildConfig &cfg = {});

aon::hpp::Trajectory buildTrajectoryFromPoseToPose(
    const aon::hpp::Pose &start,
    const aon::hpp::Pose &target,
    const TrajectoryBuildConfig &cfg = {});

}  // namespace aon::autonomy

#endif  // AON_AUTONOMY_TRAJECTORY_BUILDER_HPP_
