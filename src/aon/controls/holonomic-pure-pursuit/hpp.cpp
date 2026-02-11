#include "../../../include/aon/controls/holonomic-pure-pursuit/hpp.hpp"

namespace aon::hpp {

static bool circleSegmentIntersection(
  const Point& C, double r,
  const Point& A, const Point& B,
  Point& out
) {
  // Parametric: P(t) = A + t(B-A), t in [0,1]
  const double dx = B.x - A.x;
  const double dy = B.y - A.y;

  const double fx = A.x - C.x;
  const double fy = A.y - C.y;

  const double a = dx*dx + dy*dy;
  const double b = 2.0 * (fx*dx + fy*dy);
  const double c = fx*fx + fy*fy - r*r;

  double disc = b*b - 4*a*c;
  if (disc < 0) return false;

  disc = std::sqrt(disc);

  const double t1 = (-b - disc) / (2*a);
  const double t2 = (-b + disc) / (2*a);

  // Choose the intersection that is further along the segment (bigger t)
  double t = -1.0;
  if (t2 >= 0.0 && t2 <= 1.0) t = t2;
  else if (t1 >= 0.0 && t1 <= 1.0) t = t1;
  else return false;

  out = { A.x + t*dx, A.y + t*dy };
  return true;
}

bool findLookahead(const std::vector<Point>& path, const Pose& pose, double lookahead, Point& out) {
  if (path.empty()) return false;
  if (path.size() == 1) { out = path[0]; return true; }

  const Point C{pose.x, pose.y};
  Point best = path.back();
  bool found = false;

  // Search from start to end; last valid intersection becomes the chosen one (furthest along path).
  for (size_t i = 0; i + 1 < path.size(); i++) {
    Point hit;
    if (circleSegmentIntersection(C, lookahead, path[i], path[i+1], hit)) {
      best = hit;
      found = true;
    }
  }

  out = best;
  return found;
}

static Command zeroCmd() { return {0,0,0}; }

Command update(const std::vector<Point>& path, const Pose& pose, const Config& cfg, bool faceFinalHeading, double finalHeadingRad) {
  if (path.size() < 2) return zeroCmd();

  // 1) Lookahead point
  Point look;
  findLookahead(path, pose, cfg.lookahead, look);

  // 2) Vector to lookahead in FIELD frame
  const double dxF = look.x - pose.x;
  const double dyF = look.y - pose.y;

  // 3) Convert field -> robot frame by rotating by -theta
  // Robot X = forward, Robot Y = right
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);

  const double dxR =  c*dxF + s*dyF;
  const double dyR = -s*dxF + c*dyF;

  // 4) Direction normalize (robot-centric)
  const double mag = std::sqrt(dxR*dxR + dyR*dyR);
  double dirX = 0.0, dirY = 0.0;
  if (mag > 1e-6) {
    dirX = dxR / mag;
    dirY = dyR / mag;
  }

  // 5) Speed schedule: slow down near end
  const Point end = path.back();
  const double dEnd = dist(Point{pose.x, pose.y}, end);

  double speed = cfg.cruise;
  if (dEnd < cfg.slowRadius && cfg.slowRadius > 1e-6) {
    speed = cfg.cruise * (dEnd / cfg.slowRadius);
  }

  // 6) Heading control (omega). Two options:
  // - If faceFinalHeading: hold a specific final heading
  // - Else: face the motion direction (toward lookahead)
  double desiredHeading = pose.theta;
  if (faceFinalHeading) {
    desiredHeading = finalHeadingRad;
  } else {
    // Face the lookahead direction in FIELD frame
    desiredHeading = std::atan2(dyF, dxF);
  }

  const double headingError = wrapRad(desiredHeading - pose.theta);
  double omega = cfg.kHeading * headingError; // proportional
  // normalize omega into [-1,1] in a simple way (tune this)
  if (omega > 1.0) omega = 1.0;
  if (omega < -1.0) omega = -1.0;

  // 7) Output normalized commands
  Command cmd;
  cmd.vx = dirX * speed;
  cmd.vy = dirY * speed;
  cmd.omega = omega;
  return cmd;
}

bool isFinished(const std::vector<Point>& path, const Pose& pose, const Config& cfg, bool requireHeading, double finalHeadingRad) {
  if (path.empty()) return true;
  const Point end = path.back();
  const double dEnd = dist(Point{pose.x, pose.y}, end);
  if (dEnd > cfg.posTol) return false;

  if (!requireHeading) return true;

  const double err = wrapRad(finalHeadingRad - pose.theta);
  const double errDeg = std::abs(err * 180.0 / M_PI);
  return errDeg <= cfg.headingTolDeg;
}

} // namespace aon::hpp