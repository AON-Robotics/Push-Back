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

  double linearDeadband;  // Tune
  double angularDeadband;  // Tune

 public:
  PurePursuit(MotionProfile linearProfile, MotionProfile angularProfile,
              int lookaheadOffset, double linearDeadband, double angularDeadband)
      : linearProfile(linearProfile), angularProfile(angularProfile) {
    this->linearProfile.setFinalVelocity(0);
    this->angularProfile.setFinalVelocity(0);
    this->lookaheadOffset = lookaheadOffset;
    this->linearDeadband = linearDeadband;
    this->angularDeadband = angularDeadband;
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose`
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> go(Pose target, Pose current, double dt = 0.02) {
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

    double linearSign = (linearError == 0) ? 0 : (linearError / std::abs(linearError));
    double angularSign = (angularError == 0) ? 0 : (angularError / std::abs(angularError));
    
    // Linear and angular velocities

    // TODO: check if using these three lines instead of the next one maintains accuracy while reducing angular oscillations
    // double angleFactor = std::cos(angularError * M_PI / 180.0);
    // angleFactor = std::clamp(angleFactor, 0.0, 1.0);
    // double linearVel = linearProfile.update(abs(linearError), dt) * * linearSign * angleFactor;
    double linearVel = linearProfile.update(std::abs(linearError), dt) * linearSign;
    
    const double circumference = DRIVE_WIDTH * M_PI;
    double angularArc = circumference * (std::abs(angularError) / 360.0);
    
    double angularVel = angularProfile.update(angularArc, dt) * angularSign;

    // TODO: try using these to see if there is any improvement but I (Kevin G) dont expect it
    // double curvature = (2 * sin(angularErrorRad)) / lookaheadDistance;
    // double angularVel = curvature * linearVel;

    // Convert to tank drive velocities
    // left = v + w, right = v - w
    double left = linearVel + angularVel;
    double right = linearVel - angularVel;

    // Deadband to avoid infinite loop
    if (std::abs(linearError) <= linearDeadband)
      return {0, 0};

    return {left, right};
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose` to align heading
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> turn(Pose target, Pose current, double dt = 0.02) {
    // TODO: determine if this is necessary
    // Current heading
    double currentHeading = current.theta;

    // Desired final heading
    double targetHeading = target.theta;

    // Compute angular error (normalize to [-pi, pi])
    double angularError = targetHeading - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    double sign = (angularError == 0) ? 0 : (angularError / abs(angularError));

    // Pure turning: no linear velocity
    double angularVel = angularProfile.update(std::abs(angularError), dt) * sign;

    // Deadband (symmetric for turning)
    if (std::abs(angularError) <= angularDeadband) return {0, 0};

    return {angularVel, -angularVel};
  }

  /// @brief Calculates the command to follow the given path with the robot and aligns heading of the last `Pose`
  /// @param path The path to follow
  /// @param current The `Pose` the robot is currently at
  /// @return The command of left and right motors to follow the path
  /// @note All headings that are not of the last `Pose` will be ignored
  std::pair<double, double> follow(std::vector<Pose> path, Pose current, double dt = 0.02) {
    if (path.empty()) return {0, 0};

    // Find closest point on path
    int closestIndex = 0;
    double minDist = DBL_MAX;

    for (int i = 0; i < path.size(); i++) {
      double dist = current.distanceTo(path[i]);

      if (dist < minDist) {
        minDist = dist;
        closestIndex = i;
      }
    }

    // Lookahead index (simple fixed offset)
    int lookaheadIndex =
        std::min((size_t)(closestIndex + lookaheadOffset), path.size() - 1);

    Pose target = path[lookaheadIndex];

    double distToEnd = current.distanceTo(target);

    // if (lookaheadIndex >= path.size() - 1 && distToEnd < linearDeadband) {
    //   return turn(target, current, dt);
    // }

    // Reuse single-point controller
    return go(target, current, dt);
  }
};

}  // namespace aon
