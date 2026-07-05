#pragma once

#include "../math/pose.hpp"
#include <utility>
#include <math.h>
#include <float.h>
#include "./s-curve-profile.hpp"

namespace aon {

class PurePursuit {
 private:
  MotionProfile linearProfile;
  MotionProfile angularProfile;

  int lookaheadOffset;
  double linearDeadband;
  double angularDeadband;

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
    double dx = target.x - current.x;
    double dy = target.y - current.y;

    double linearError = std::hypot(dx, dy);

    double targetAngle = std::atan2(dy, dx) * 180 / M_PI;
    double currentHeading = current.theta;

    // Wrapping prevents the controller from choosing the long way around.
    double angularError = targetAngle - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    double linearSign = (linearError == 0) ? 0 : (linearError / std::abs(linearError));
    double angularSign = (angularError == 0) ? 0 : (angularError / std::abs(angularError));
    
    double linearVel = linearProfile.update(std::abs(linearError), dt) * linearSign;
    
    const double circumference = DRIVE_WIDTH * M_PI;
    double angularArc = circumference * (std::abs(angularError) / 360.0);
    
    double angularVel = angularProfile.update(angularArc, dt) * angularSign;

    // Differential-drive mixing turns translational and angular requests into
    // independently commandable wheel velocities.
    double left = linearVel + angularVel;
    double right = linearVel - angularVel;

    // A zero command signals completion to blocking drivetrain callers.
    if (std::abs(linearError) <= linearDeadband)
      return {0, 0};

    return {left, right};
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose` to align heading
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> turn(Pose target, Pose current, double dt = 0.02) {
    double currentHeading = current.theta;
    double targetHeading = target.theta;

    // Use the shortest angular displacement across the 0/360 boundary.
    double angularError = targetHeading - currentHeading;
    while (angularError > 180) angularError -= 360;
    while (angularError < -180) angularError += 360;

    double sign = (angularError == 0) ? 0 : (angularError / abs(angularError));

    double angularVel = angularProfile.update(std::abs(angularError), dt) * sign;

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

    int closestIndex = 0;
    double minDist = DBL_MAX;

    for (int i = 0; i < path.size(); i++) {
      double dist = current.distanceTo(path[i]);

      if (dist < minDist) {
        minDist = dist;
        closestIndex = i;
      }
    }

    // This fixed sample offset assumes paths use consistent point spacing.
    int lookaheadIndex =
        std::min((size_t)(closestIndex + lookaheadOffset), path.size() - 1);

    Pose target = path[lookaheadIndex];

    return go(target, current, dt);
  }
};

}  // namespace aon
