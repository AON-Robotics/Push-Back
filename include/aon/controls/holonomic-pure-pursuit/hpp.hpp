#pragma once
#include <vector>
#include <cmath>
#include "../../tools/vector.hpp"

namespace aon::hpp {

struct Pose {
  double x;      // inches
  double y;      // inches
  double theta;  // radians
};

struct Command {
  double vx;     // robot-centric [-1,1]
  double vy;     // robot-centric [-1,1]
  double omega;  // robot-centric [-1,1]
};

struct Config {
  double lookahead = 12.0;      // inches
  double cruise = 0.6;          // base speed magnitude in [0,1]
  double kHeading = 1.5;        // heading P-gain
  double slowRadius = 10.0;     // inches: start slowing down near end
  double posTol = 2.0;          // inches: stop distance
  double headingTolDeg = 5.0;   // degrees: optional stop heading tolerance
};

struct Point {
  double x;
  double y;
};

inline double wrapRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline double dist(const Point& a, const Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx*dx + dy*dy);
}

/// Find a lookahead point using circle-segment intersection (polyline path).
/// Returns true if found; otherwise returns last point.
bool findLookahead(const std::vector<Point>& path, const Pose& pose, double lookahead, Point& out);

/// Compute one HPP update step.
Command update(const std::vector<Point>& path, const Pose& pose, const Config& cfg, bool faceFinalHeading = false, double finalHeadingRad = 0.0);

/// Stop condition: near final point (and optionally heading)
bool isFinished(const std::vector<Point>& path, const Pose& pose, const Config& cfg, bool requireHeading = false, double finalHeadingRad = 0.0);

} // namespace aon::hpp