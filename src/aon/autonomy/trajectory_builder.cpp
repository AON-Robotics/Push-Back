#include "aon/autonomy/trajectory_builder.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace aon::autonomy {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapPi(double a_rad) {
  while (a_rad > kPi) a_rad -= 2.0 * kPi;
  while (a_rad < -kPi) a_rad += 2.0 * kPi;
  return a_rad;
}

XYPoint clampPointToClearance(const XYPoint &p, const TrajectoryBuildConfig &cfg) {
  if (!cfg.enforce_field_clearance) return p;

  const double pad = std::max(0.0, cfg.robot_radius_in + cfg.wall_clearance_in);
  const double min_x = cfg.field_min_x_in + pad;
  const double max_x = cfg.field_max_x_in - pad;
  const double min_y = cfg.field_min_y_in + pad;
  const double max_y = cfg.field_max_y_in - pad;

  XYPoint out = p;
  if (min_x <= max_x) out.x_in = std::clamp(out.x_in, min_x, max_x);
  if (min_y <= max_y) out.y_in = std::clamp(out.y_in, min_y, max_y);
  return out;
}

std::vector<XYPoint> clampPointsToClearance(const std::vector<XYPoint> &pts,
                                            const TrajectoryBuildConfig &cfg) {
  if (!cfg.enforce_field_clearance || pts.empty()) return pts;
  std::vector<XYPoint> out = pts;
  for (auto &p : out) {
    p = clampPointToClearance(p, cfg);
  }
  return out;
}

double pointLineDistance(const XYPoint &p, const XYPoint &a, const XYPoint &b) {
  const double vx = b.x_in - a.x_in;
  const double vy = b.y_in - a.y_in;
  const double wx = p.x_in - a.x_in;
  const double wy = p.y_in - a.y_in;
  const double vv = vx * vx + vy * vy;

  if (vv <= 1e-9) {
    const double dx = p.x_in - a.x_in;
    const double dy = p.y_in - a.y_in;
    return std::sqrt(dx * dx + dy * dy);
  }

  const double t = std::clamp((wx * vx + wy * vy) / vv, 0.0, 1.0);
  const double px = a.x_in + t * vx;
  const double py = a.y_in + t * vy;
  const double dx = p.x_in - px;
  const double dy = p.y_in - py;
  return std::sqrt(dx * dx + dy * dy);
}

void rdpRecursive(const std::vector<XYPoint> &pts,
                  const int i0,
                  const int i1,
                  const double eps,
                  std::vector<bool> &keep) {
  if (i1 <= i0 + 1) return;

  double max_dist = -1.0;
  int max_idx = -1;
  for (int i = i0 + 1; i < i1; ++i) {
    const double d = pointLineDistance(pts[i], pts[i0], pts[i1]);
    if (d > max_dist) {
      max_dist = d;
      max_idx = i;
    }
  }

  if (max_idx >= 0 && max_dist > eps) {
    keep[max_idx] = true;
    rdpRecursive(pts, i0, max_idx, eps, keep);
    rdpRecursive(pts, max_idx, i1, eps, keep);
  }
}

std::vector<XYPoint> rdpSimplify(const std::vector<XYPoint> &pts,
                                 const double eps) {
  if (pts.size() <= 2 || eps <= 0.0) return pts;

  std::vector<bool> keep(pts.size(), false);
  keep.front() = true;
  keep.back() = true;
  rdpRecursive(pts, 0, static_cast<int>(pts.size()) - 1, eps, keep);

  std::vector<XYPoint> out;
  out.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (keep[i]) out.push_back(pts[i]);
  }
  return out;
}

std::vector<XYPoint> smoothPolyline(const std::vector<XYPoint> &pts,
                                    const int iterations,
                                    const double alpha) {
  if (pts.size() < 5 || iterations <= 0 || alpha <= 0.0) return pts;

  std::vector<XYPoint> cur = pts;
  std::vector<XYPoint> nxt = pts;

  for (int it = 0; it < iterations; ++it) {
    nxt = cur;
    for (std::size_t i = 2; i + 2 < cur.size(); ++i) {
      nxt[i].x_in =
          cur[i].x_in + alpha * (0.5 * (cur[i - 1].x_in + cur[i + 1].x_in) - cur[i].x_in);
      nxt[i].y_in =
          cur[i].y_in + alpha * (0.5 * (cur[i - 1].y_in + cur[i + 1].y_in) - cur[i].y_in);
    }
    cur.swap(nxt);
  }

  return cur;
}

aon::hpp::Trajectory samplePolylineUniformS(const std::vector<XYPoint> &pts,
                                            const double ds_in,
                                            const double end_heading_rad) {
  aon::hpp::Trajectory traj;
  if (pts.size() < 2) return traj;

  const double step_ds = std::max(1e-3, ds_in);

  std::vector<double> cum_s(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x_in - pts[i - 1].x_in;
    const double dy = pts[i].y_in - pts[i - 1].y_in;
    cum_s[i] = cum_s[i - 1] + std::sqrt(dx * dx + dy * dy);
  }

  const double s_end = cum_s.back();
  traj.reserve(static_cast<std::size_t>(std::floor(s_end / step_ds)) + 2U);

  std::size_t seg = 0;
  for (double s = 0.0; s <= s_end; s += step_ds) {
    while (seg + 1 < cum_s.size() && cum_s[seg + 1] < s) ++seg;
    if (seg + 1 >= pts.size()) break;

    const double seg_len = std::max(1e-9, cum_s[seg + 1] - cum_s[seg]);
    const double t = std::clamp((s - cum_s[seg]) / seg_len, 0.0, 1.0);

    aon::hpp::TrajectorySample smp{};
    smp.x_in = pts[seg].x_in + t * (pts[seg + 1].x_in - pts[seg].x_in);
    smp.y_in = pts[seg].y_in + t * (pts[seg + 1].y_in - pts[seg].y_in);
    smp.h_in = std::atan2(pts[seg + 1].y_in - pts[seg].y_in,
                          pts[seg + 1].x_in - pts[seg].x_in);
    smp.s_in = s;
    traj.push_back(smp);
  }

  aon::hpp::TrajectorySample end{};
  end.x_in = pts.back().x_in;
  end.y_in = pts.back().y_in;
  end.h_in = wrapPi(end_heading_rad);
  end.s_in = s_end;
  // Keep legacy behavior: append explicit endpoint sample unconditionally.
  // This mirrors the previously proven path construction behavior in main.cpp.
  traj.push_back(end);

  return traj;
}

}  // namespace

aon::hpp::Trajectory buildTrajectoryFromPoints(
    const std::vector<XYPoint> &points,
    double end_heading_rad,
    const TrajectoryBuildConfig &cfg) {
  if (points.size() < 2) return {};

  const std::vector<XYPoint> bounded = clampPointsToClearance(points, cfg);
  const std::vector<XYPoint> reduced = rdpSimplify(bounded, cfg.rdp_eps_in);
  const std::vector<XYPoint> smooth =
      smoothPolyline(reduced, cfg.smooth_iterations, cfg.smooth_alpha);

  return samplePolylineUniformS(smooth, cfg.sample_ds_in, end_heading_rad);
}

aon::hpp::Trajectory buildTrajectoryFromPoseToPose(
    const aon::hpp::Pose &start,
    const aon::hpp::Pose &target,
    const TrajectoryBuildConfig &cfg) {
  const XYPoint p0{start.x_in, start.y_in};
  const XYPoint p3 = clampPointToClearance({target.x_in, target.y_in}, cfg);

  const double dx_axis = p3.x_in - p0.x_in;
  const double dy_axis = p3.y_in - p0.y_in;
  const double adx = std::abs(dx_axis);
  const double ady = std::abs(dy_axis);

  const bool nearly_axis = (ady < 0.75) || (adx < 0.75);
  const bool x_dominant = (adx > 2.0 * std::max(ady, 1e-6));
  const bool y_dominant = (ady > 2.0 * std::max(adx, 1e-6));

  if (nearly_axis || x_dominant || y_dominant) {
    return buildTrajectoryFromPoints({p0, p3}, target.theta_rad, cfg);
  }

  const double dx = p3.x_in - p0.x_in;
  const double dy = p3.y_in - p0.y_in;
  const double dist = std::sqrt(dx * dx + dy * dy);
  const double lead = std::clamp(0.35 * dist, 4.0, 18.0);

  const XYPoint p1{
      p0.x_in + lead * std::cos(start.theta_rad),
      p0.y_in + lead * std::sin(start.theta_rad),
  };

  const XYPoint p2_raw{
      p3.x_in - lead * std::cos(target.theta_rad),
      p3.y_in - lead * std::sin(target.theta_rad),
  };
  const XYPoint p2 = clampPointToClearance(p2_raw, cfg);

  return buildTrajectoryFromPoints({p0, p1, p2, p3}, target.theta_rad, cfg);
}

}  // namespace aon::autonomy
