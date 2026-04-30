#pragma once

#include "../math/pose.hpp"
#include <utility>
#include <math.h>
#include <float.h>
#include "./s-curve-profile.hpp"

namespace aon {

// TODO: add option for holonomic drives, if necessary
class PurePursuit {
 private:
  // Motion Profiles
  MotionProfile linearProfile;
  MotionProfile angularProfile;

  int lookaheadOffset;  // Tune

  int deadband;  // Tune

  double turningThreshold;  // Tune

 public:
  PurePursuit(MotionProfile linearProfile, MotionProfile angularProfile,
              int lookaheadOffset, int deadband, double turningThreshold)
      : linearProfile(linearProfile), angularProfile(angularProfile) {
    this->linearProfile.setFinalVelocity(0);
    this->angularProfile.setFinalVelocity(0);
    this->lookaheadOffset = lookaheadOffset;
    this->deadband = deadband;
    this->turningThreshold = turningThreshold;
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose`
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> go(Pose target, Pose current) {
    // Basic Pure Pursuit-style controller (simplified for single target)

    // Extract positions
    double dx = target.x - current.x;
    double dy = target.y - current.y;

    // linearError to target
    double linearError = std::hypot(dx, dy);

    // Desired heading
    double targetAngle = std::atan2(dy, dx) * 180 / M_PI;

    // Current heading (convert to radians)
    double currentHeading = current.theta;

    // Heading angularError (normalize to [-180, 180])
    double angularError = targetAngle - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    // Linear and angular velocities
    // TODO: experiment using an S-Curve Motion Profile
    double linearVel =
        linearProfile.update(linearError) * linearError / abs(linearError);
    double angularVel =
        angularProfile.update(angularError) * angularError / abs(angularError);

    // Convert to tank drive velocities
    // left = v + w, right = v - w
    double left = linearVel + angularVel;
    double right = linearVel - angularVel;

    // Deadband to avoid infinite loop
    if (std::abs(left) < deadband && std::abs(linearError) <= 0.5 &&
        std::abs(angularError) <= 2)
      left = 0;
    if (std::abs(right) < deadband && std::abs(linearError) <= 0.5 &&
        std::abs(angularError) <= 2)
      right = 0;

    return {left, right};
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose` to align heading
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> turn(Pose target, Pose current) {
    // Current heading
    double currentHeading = current.theta;

    // Desired final heading
    double targetHeading = target.theta;

    // Compute angular error (normalize to [-pi, pi])
    double angularError = targetHeading - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    // Pure turning: no linear velocity
    // TODO: experiment using an S-Curve Motion Profile
    double angularVel =
        angularProfile.update(angularError) * angularError / abs(angularError);

    double left = angularVel;
    double right = -angularVel;

    // Deadband (symmetric for turning)
    if (std::abs(left) < deadband && std::abs(angularError) <= 2) left = 0;
    if (std::abs(right) < deadband && std::abs(angularError) <= 2) right = 0;

    return {left, right};
  }

  /// @brief Calculates the command to follow the given path with the robot and aligns heading of the last `Pose`
  /// @param path The path to follow
  /// @param current The `Pose` the robot is currently at
  /// @return The command of left and right motors to follow the path
  /// @note All headings that are not of the last `Pose` will be ignored
  std::pair<double, double> follow(std::vector<Pose> path, Pose current) {
    if (path.empty()) return {0, 0};

    // Find closest point on path
    int closestIndex = 0;
    double minDist = DBL_MAX;

    for (int i = 0; i < path.size(); i++) {
      double dx = path[i].x - current.x;
      double dy = path[i].y - current.y;
      double dist = std::hypot(dx, dy);

      if (dist < minDist) {
        minDist = dist;
        closestIndex = i;
      }
    }

    // Lookahead index (simple fixed offset)
    int lookaheadIndex =
        std::min((size_t)(closestIndex + lookaheadOffset), path.size() - 1);

    Pose target = path[lookaheadIndex];

    double distToEnd = std::hypot(target.x - current.x, target.y - current.y);

    if (lookaheadIndex >= path.size() - 1 && distToEnd < turningThreshold) {
      return turn(target, current);
    }

    // Reuse single-point controller
    return go(target, current);
  }
};

}  // namespace aon
