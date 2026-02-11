// src/aon/auton/test-hpp.cpp

#include "aon/controls/holonomic-pure-pursuit/hpp.hpp"
#include "aon/sensing/odometry.hpp"
#include "aon/globals.hpp"   // <-- THIS is where the global drivetrain is defined
#include "api.h"

namespace aon {

void testHPPRoutine() {
  // 1) Reset odometry at known start pose (inches, inches, degrees)
  odometry::ResetCurrent(0.0, 0.0, 0.0);

  // 2) Simple path in inches (L-shape)
  std::vector<hpp::Point> path = {
    {0.0, 0.0},
    {24.0, 0.0},
    {24.0, 24.0}
  };

  hpp::Config cfg;
  cfg.lookahead     = 10.0;
  cfg.cruise        = 0.6;
  cfg.kHeading      = 1.2;
  cfg.slowRadius    = 10.0;
  cfg.posTol        = 2.0;
  cfg.headingTolDeg = 8.0;

  const int dtMs = 20;
  const int timeoutMs = 6000;
  int elapsed = 0;

  const bool faceFinal = false;
  const double finalHeading = 0.0; // radians (only used if faceFinal = true)

  while (elapsed < timeoutMs) {
    hpp::Pose pose{
      odometry::GetX(),
      odometry::GetY(),
      odometry::GetRadians()
    };

    if (hpp::isFinished(path, pose, cfg, faceFinal, finalHeading)) break;

    hpp::Command cmd = hpp::update(path, pose, cfg, faceFinal, finalHeading);

    // Use the global drivetrain from globals.hpp
    ::drivetrain.driveRobotCentric(cmd.vx, cmd.vy, cmd.omega, 0.7);

    pros::delay(dtMs);
    elapsed += dtMs;
  }

  ::drivetrain.driveRobotCentric(0, 0, 0, 0.0);
  pros::delay(200);
}

} // namespace aon