#include "aon/controls/holonomic-pure-pursuit/hpp.hpp"

#include "EKF/EKF.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::hpp {

// ---------------- Helpers ----------------

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ---------------- Constructor ----------------

Controller::Controller(const Config& cfg)
  : cfg_(cfg), mp_(cfg.mp_cfg) {}

// ---------------- Trajectory Management ----------------

void Controller::setTrajectory(const Trajectory& traj) {
  traj_ = traj;
  // Reset hint indices — new trajectory, projection must restart from beginning
  proj_hint_idx_ = 0;
  samp_hint_idx_ = 0;
  last_s_robot_in_ = 0.0;
  have_last_s_robot_ = false;
  tank_turn_in_place_active_ = false;
  // Reset motion profile integrator so velocity ramps up from 0
  mp_.reset();
}

void Controller::clear() {
  traj_.clear();
  proj_hint_idx_ = 0;
  samp_hint_idx_ = 0;
  last_s_robot_in_ = 0.0;
  have_last_s_robot_ = false;
  tank_turn_in_place_active_ = false;
  mp_.reset();
}

// ---------------- Angle Utilities ----------------

double Controller::wrapPi(double a_rad) {
  // While loop instead of fmod: fmod has precision issues for values very close
  // to ±π due to floating-point rounding. This is safe and fast for angles
  // that stay within a few revolutions of [−π, π], which is always true here.
  static constexpr double kPi = 3.14159265358979323846;
  while (a_rad >  kPi) a_rad -= 2.0 * kPi;
  while (a_rad < -kPi) a_rad += 2.0 * kPi;
  return a_rad;
}

double Controller::angleDiff(double a_rad, double b_rad) {
  // Shortest-arc difference: result ∈ [−π, π]
  return wrapPi(a_rad - b_rad);
}

// ---------------- Projection — closest s on path ----------------

double Controller::projectPoseToS(const Trajectory& traj, const Pose& pose,
                                  double min_seg_len_in, double s_eps_in,
                                  std::size_t* io_hint_idx) {
  if (traj.empty()) return 0.0;
  if (traj.size() == 1) return traj.front().s_in;

  const double px = pose.x_in;
  const double py = pose.y_in;

  const std::size_t nSeg = traj.size() - 1;

  // Start scan from last known segment (amortized O(1) for forward motion)
  std::size_t start = 0;
  if (io_hint_idx && *io_hint_idx < nSeg) start = *io_hint_idx;

  double bestD2 = std::numeric_limits<double>::infinity();
  double bestS = traj.front().s_in;
  std::size_t bestIdx = 0;

  const double minSeg2 = min_seg_len_in * min_seg_len_in;

  // --- Scan all segments starting from hint (wraps around for robustness) ---
  for (std::size_t k = 0; k < nSeg; ++k) {
    const std::size_t i = (start + k) % nSeg;
    const auto& A = traj[i];
    const auto& B = traj[i + 1];

    const double ax = A.x_in, ay = A.y_in;
    const double bx = B.x_in, by = B.y_in;

    // Segment vector v = B − A
    const double vx = bx - ax;
    const double vy = by - ay;
    const double segLen2 = vx * vx + vy * vy;

    // Skip degenerate segments (prevents division-by-zero)
    if (segLen2 < minSeg2) continue;

    // Vector from A to robot: w = P − A
    const double wx = px - ax;
    const double wy = py - ay;

    // Projection parameter: t = dot(w, v) / |v|²,  clamped to [0, 1]
    // t=0 → closest point is A, t=1 → closest point is B
    double t = (wx * vx + wy * vy) / segLen2;
    t = clampd(t, 0.0, 1.0);

    // Closest point on segment: Q = A + t * v
    const double qx = ax + t * vx;
    const double qy = ay + t * vy;

    // Squared distance from robot to closest point: d² = |P − Q|²
    const double dx = px - qx;
    const double dy = py - qy;
    const double d2 = dx * dx + dy * dy;

    if (d2 < bestD2) {
      bestD2 = d2;
      // Arc-length of closest point: s = A.s + t * (B.s − A.s)
      const double ds = (B.s_in - A.s_in);
      bestS = A.s_in + t * ds;
      bestIdx = i;
    }
  }

  // Update hint for next call (amortized O(1) when robot moves forward)
  if (io_hint_idx) *io_hint_idx = bestIdx;

  // Clamp to valid arc-length range and snap near-endpoints to exact boundaries
  const double s0 = traj.front().s_in;
  const double s1 = traj.back().s_in;
  bestS = clampd(bestS, s0, s1);

  if (std::abs(bestS - s0) < s_eps_in) bestS = s0;
  if (std::abs(bestS - s1) < s_eps_in) bestS = s1;

  return bestS;
}

// ---------------- Sampling — interpolate at s ----------------

TrajectorySample Controller::sampleAtS(const Trajectory& traj, double s_target_in,
                                       double s_eps_in, std::size_t* io_hint_idx) {
  TrajectorySample out{};
  if (traj.empty()) return out;
  if (traj.size() == 1) return traj.front();

  const double s0 = traj.front().s_in;
  const double s1 = traj.back().s_in;

  // Clamp s_target to [s_start, s_end] — looking past the end returns the last sample
  const double s = clampd(s_target_in, s0, s1);

  // Start scan from last known segment (amortized O(1) for forward lookahead)
  std::size_t i = 0;
  if (io_hint_idx && *io_hint_idx < traj.size() - 1) i = *io_hint_idx;

  // Walk hint backward/forward until we find the bracket:
  //   traj[i].s ≤ s ≤ traj[i+1].s
  while (i > 0 && traj[i].s_in > s + s_eps_in) --i;
  while (i + 1 < traj.size() && traj[i + 1].s_in < s - s_eps_in) ++i;
  if (i + 1 >= traj.size()) i = traj.size() - 2;

  const auto& A = traj[i];
  const auto& B = traj[i + 1];

  // Linear interpolation weight: α = (s − A.s) / (B.s − A.s),   α ∈ [0, 1]
  const double ds = (B.s_in - A.s_in);
  double alpha = 0.0;
  if (std::abs(ds) > s_eps_in) alpha = (s - A.s_in) / ds;
  alpha = clampd(alpha, 0.0, 1.0);

  // Position: linear interpolation
  //   x = A.x + α*(B.x − A.x)
  //   y = A.y + α*(B.y − A.y)
  out.x_in = A.x_in + alpha * (B.x_in - A.x_in);
  out.y_in = A.y_in + alpha * (B.y_in - A.y_in);
  out.s_in = s;

  // Heading: shortest-arc interpolation to avoid 2π wraparound jumps
  //   dh = wrapPi(B.h − A.h)       ← shortest angular difference between samples
  //   h  = wrapPi(A.h + α * dh)    ← interpolated + rewrapped to [−π, π]
  const double dh = wrapPi(B.h_in - A.h_in);
  out.h_in = wrapPi(A.h_in + alpha * dh);

  if (io_hint_idx) *io_hint_idx = i;
  return out;
}

// ---------------- Finish Condition ----------------

bool Controller::isFinished(const Trajectory& traj, const Pose& pose, const Config& cfg,
                            double theta_ref_rad) {
  if (traj.empty()) return true;

  // Position check: Euclidean distance to trajectory endpoint
  const auto& end = traj.back();
  const double dx = end.x_in - pose.x_in;
  const double dy = end.y_in - pose.y_in;
  const double d = std::sqrt(dx * dx + dy * dy);

  if (d > cfg.pos_tol_in) return false;      // Not close enough yet
  if (!cfg.require_heading_for_finish) return true;

  // Optional heading check: |wrapPi(h_ref − θ)| < heading_tol_rad
  const double e = angleDiff(theta_ref_rad, pose.theta_rad);
  return std::abs(e) <= cfg.heading_tol_rad;
}

// ---------------- Main Step ----------------

StepOutput Controller::step(const StepInput& in) {
  // ================================================================
  //  Pure Pursuit step — deterministic, no timing, no I/O
  //
  //  [a] Project robot pose onto trajectory → s_robot
  //  [b] remaining = s_end − s_robot
  //  [c] v_des = mp_.update(remaining, dt)   ← jerk-limited S-curve
  //        v_cap = sqrt(2 * a_decel * remaining)  ensures we can stop
  //  [d] s_target = s_robot + lookahead_in
  //  [e] lookahead = sampleAtS(s_target)     ← {x_L, y_L, h_L}
  //  [f] Error in field frame:
  //        ex_f = x_L − x_robot
  //        ey_f = y_L − y_robot
  //  [g] Rotate to robot frame (rotation by −θ):
  //        ex_r =  cos(θ)*ex_f + sin(θ)*ey_f
  //        ey_r = −sin(θ)*ex_f + cos(θ)*ey_f
  //      Tank: ey_r = 0  (no lateral authority)
  //  [h] Normalize → unit vector (ux):
  //        mag = sqrt(ex_r²)
  //        ux = ex_r/mag
  //  [i] Velocity command:
  //        cmd.vx = ux * v_des
  //  [j] Heading P-control:
  //        h_err = wrapPi(h_ref − θ)
  //        omega = clamp(kHeading * h_err, ±omega_max_rps)
  //  [k] Finish: dist(pose, traj.back()) < pos_tol_in
  // ================================================================

  StepOutput out{};
  out.has_path = (traj_.size() >= 2);

  if (!out.has_path) {
    out.cmd = Command{0.0, 0.0};
    out.finished = true;
    tank_turn_in_place_active_ = false;
    return out;
  }

  // [a] Project robot pose onto trajectory → s_robot
  double s_robot =
    projectPoseToS(traj_, in.pose, cfg_.min_seg_len_in,
       cfg_.s_eps_in, &proj_hint_idx_);

  if (cfg_.enforce_monotonic_s) {
    if (have_last_s_robot_) {
      s_robot = std::max(s_robot, last_s_robot_in_);
      if (cfg_.max_s_robot_step_in > 0.0) {
        const double s_max = last_s_robot_in_ + cfg_.max_s_robot_step_in;
        s_robot = std::min(s_robot, s_max);
      }
    }
    last_s_robot_in_ = s_robot;
    have_last_s_robot_ = true;
  }
  out.s_robot_in = s_robot;

  // [b] + [c] Remaining distance → jerk-limited speed setpoint
  //   remaining = max(s_end − s_robot, dist_to_goal)
  //   v_cap = sqrt(2 * a_decel * remaining)  ← ensures the robot can always stop in time
  const double s_end = traj_.back().s_in;
  const auto& end = traj_.back();
  const double dx_goal = end.x_in - in.pose.x_in;
  const double dy_goal = end.y_in - in.pose.y_in;
  const double dist_goal_in = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
  const double remaining_arc_in = std::max(0.0, s_end - s_robot);
  const double remainingDist_in = cfg_.use_endpoint_distance_guard
      ? std::max(remaining_arc_in, dist_goal_in)
      : remaining_arc_in;
  const double v_des_ips = mp_.update(remainingDist_in, cfg_.dt_s);

  // [d] Adaptive lookahead: L = clamp(k_lookahead_s * v_des, L_min, L_max)
  // k_lookahead_s = 0.0 → fixed lookahead (backward-compatible)
  // Coupling with the S-curve: as v_des shrinks near the endpoint,
  // L shrinks too → the robot converges precisely without overshooting.
  const double adaptive_L = (cfg_.k_lookahead_s > 0.0)
      ? clampd(cfg_.k_lookahead_s * v_des_ips, cfg_.lookahead_min_in, cfg_.lookahead_in)
      : cfg_.lookahead_in;
  out.v_des_ips = v_des_ips;
  out.adaptive_lookahead_in = adaptive_L;

  // [e] Lookahead point at s_robot + adaptive_L
  const double s_target = s_robot + adaptive_L;
  out.s_target_in = s_target;

  const TrajectorySample look =
    sampleAtS(traj_, s_target, cfg_.s_eps_in, &samp_hint_idx_);
  out.lookahead = look;

  // [f] Error vector in field frame: lookahead point − robot position
  const double ex_f = look.x_in - in.pose.x_in;
  const double ey_f = look.y_in - in.pose.y_in;

  // Heading reference: from trajectory metadata (default) or external input
  const double h_traj = cfg_.use_traj_heading ? look.h_in : in.theta_ref_rad;
  double theta_ref_used = h_traj;

  // Cross-track correction: blend trajectory heading with geometric bearing to lookahead.
  // h_geo = atan2(ey_f, ex_f) = field-frame bearing from robot to the lookahead point.
  //   When on-path:  h_geo ≈ h_traj → blend ≈ h_traj (zero side effect on straight paths).
  //   When off-path: h_geo steers back toward lookahead → lateral recovery.
  if (cfg_.k_geo > 0.0) {
    const double mag_f = std::sqrt(ex_f * ex_f + ey_f * ey_f);
    if (mag_f > 1e-9) {
      const double h_geo = std::atan2(ey_f, ex_f);
      theta_ref_used = wrapPi(h_traj + cfg_.k_geo * angleDiff(h_geo, h_traj));
    }
  }
  const double h_err = angleDiff(theta_ref_used, in.pose.theta_rad);

  // [g] Rotate error to robot frame (rotation matrix by −θ):
  //   ex_r =  cos(θ)*ex_f + sin(θ)*ey_f
  //   ey_r = −sin(θ)*ex_f + cos(θ)*ey_f
  const double c = std::cos(in.pose.theta_rad);
  const double s = std::sin(in.pose.theta_rad);

  const double ex_r =  c * ex_f + s * ey_f;
  const double ey_r = -s * ex_f + c * ey_f;

  // Tank forward channel:
  // - vx comes from forward lookahead error only (ex_r)
  const double tx = ex_r;

  // [h] Normalize to unit direction vector (ux)
  //   mag = |tx|
  //   Guard against zero vector (robot already at or past lookahead point)
  const double mag = std::abs(tx);
  double ux = 0.0;
  if (mag > 1e-9) {
    ux = tx / mag;
  }

  // [i] Scale unit vector by speed setpoint and apply hard cap
  double v = std::max(0.0, v_des_ips);
  v = std::min(v, cfg_.v_max_ips);

  // [j] Heading P-control:
  //   h_err = wrapPi(h_ref − θ)          ← signed shortest-arc heading error
  //   omega = clamp(kHeading * h_err, ±omega_max_rps)
  if (cfg_.heading_slowdown_gain > 0.0) {
    const double abs_h_err = std::abs(h_err);
    const double speed_scale = 1.0 / (1.0 + cfg_.heading_slowdown_gain * abs_h_err * abs_h_err);
    v *= speed_scale;
  }

  // Optional: slow forward speed when lateral error is high.
  // This improves path fidelity for Tank mode by preventing "dive past then recover".
  if (cfg_.cross_track_slowdown_gain > 0.0) {
    const double speed_scale = 1.0 / (1.0 + cfg_.cross_track_slowdown_gain * ey_r * ey_r);
    v *= speed_scale;
  }

  // Near endpoint, force a proportional speed cap to avoid terminal overshoot/U-shape.
  if (cfg_.terminal_window_in > 0.0 && remainingDist_in < cfg_.terminal_window_in) {
    const double v_terminal_cap = std::max(cfg_.terminal_min_speed_ips,
                                           cfg_.terminal_kp * std::max(0.0, remainingDist_in));
    v = std::min(v, v_terminal_cap);
  }

  Command cmd{};
  cmd.vx_ips = ux * v;
  if (!cfg_.tank_allow_reverse) {
    cmd.vx_ips = std::max(0.0, cmd.vx_ips);
  }

  double omega_pp = 0.0;
  const double L2 = adaptive_L * adaptive_L;
  if (L2 > 1e-9) {
    omega_pp = (2.0 * cmd.vx_ips * ey_r) / L2;
  }

  double omega = cfg_.kHeading * h_err + cfg_.kappa_pp_gain * omega_pp;

  // Tank turn-in-place mode for large heading errors.
  // This is critical for sparse waypoint paths (e.g. ~6 in spacing):
  // pivot first, then drive forward to avoid arc-chasing zigzags.
  if (cfg_.tank_turn_in_place_on_large_heading_err &&
      cfg_.tank_turn_in_place_err_rad > 1e-9) {
    const double h_abs = std::abs(h_err);
    const double thr_on = cfg_.tank_turn_in_place_err_rad;
    const double thr_off = 0.6 * thr_on;

    if (tank_turn_in_place_active_) {
      if (h_abs <= thr_off) tank_turn_in_place_active_ = false;
    } else {
      if (h_abs >= thr_on) tank_turn_in_place_active_ = true;
    }

    if (tank_turn_in_place_active_) {
      cmd.vx_ips = 0.0;
      omega = cfg_.kHeading * h_err;
    }
  }

  omega = clampd(omega, -cfg_.omega_max_rps, cfg_.omega_max_rps);
  cmd.omega_rps = omega;

  out.cmd = cmd;

  // Telemetry fields
  out.ex_r_in   = ex_r;
  out.ey_r_in   = ey_r;
  out.h_err_rad = h_err;
  out.omega_pp_rps = omega_pp;

  // [k] Finish check — use raw trajectory heading (h_traj) for the finish heading tolerance,
  // not the blended theta_ref_used, so the finish condition reflects path intent.
  out.finished = isFinished(traj_, in.pose, cfg_, h_traj);
  return out;
}

// ---------------- EKF overloads ----------------

StepOutput Controller::step(const EKF& ekf, double theta_ref_rad) {
  // Extract pose from EKF state and delegate to the full step()
  const EKF::State s = ekf.getState();

  StepInput in;
  in.pose.x_in = s.x_in;
  in.pose.y_in = s.y_in;
  in.pose.theta_rad = s.theta_rad;
  in.theta_ref_rad = theta_ref_rad;

  return step(in);
}

StepOutput Controller::step(const EKF& ekf) {
  // When cfg_.use_traj_heading is true (default), theta_ref is taken from
  // lookahead.h_in inside step(StepInput), so the value passed here is ignored.
  return step(ekf, 0.0);
}

} // namespace aon::hpp
