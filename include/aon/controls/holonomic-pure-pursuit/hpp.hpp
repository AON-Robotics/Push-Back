// hpp.hpp
// ============================================================
//
//  Pure Pursuit Controller (Tank pipeline) — arc-length parameterized
//
//  Pipeline per step() call (50 Hz nominal):
//
//    [a] projectPoseToS   — find s_robot: closest arc-length on
//                           the trajectory to the robot's pose
//
//    [b] remaining = s_end − s_robot
//
//    [c] SCurveProfile2.update(remaining, dt)
//                         — jerk-limited S-curve → v_des [in/s]
//                           v_cap = sqrt(2 * a_decel * remaining)
//                           ensures the robot can always stop in time
//
//    [d] s_target = s_robot + lookahead_in
//
//    [e] sampleAtS(s_target) — lookahead point {x_L, y_L, h_L}
//                              via linear interpolation on segments
//
//    [f] Error in field frame:
//          ex_f = x_L − x_robot
//          ey_f = y_L − y_robot
//
//    [g] Rotate to robot frame (standard 2D rotation by −θ):
//          ex_r =  cos(θ)*ex_f + sin(θ)*ey_f
//          ey_r = −sin(θ)*ex_f + cos(θ)*ey_f
//        Tank mode: ey_r is forced to 0 (no lateral authority)
//
//    [h] Normalize → unit direction vector:
//          mag = sqrt(ex_r²)
//          ux = ex_r/mag
//
//    [i] Velocity command:
//          cmd.vx_ips = ux * v_des
//
//    [j] Heading proportional control:
//          h_err     = wrapPi(h_ref − θ)
//          cmd.omega = clamp(kHeading * h_err, ±omega_max_rps)
//
//    [k] Finish: dist(pose, traj.back()) < pos_tol_in
//               [+ |h_err| < heading_tol_rad if require_heading_for_finish]
//
//  Dependencies:
//    EKF::getState()          → pose {x_in, y_in, theta_rad}
//    SCurveProfile2::update()  → jerk-limited v_des
//    struct_master.hpp        → all tuning knobs via toHppConfig()
//
//  Units: inches [in], radians [rad], seconds [s]
//  Coordinate convention: +X forward, +Y right, +θ CCW
//
// ============================================================

#pragma once

#include <cstddef>
#include <vector>

#include "aon/controls/s-curve-profile2.hpp"

class EKF;

namespace aon::hpp {

/**
 * @brief Robot pose in the global (field) frame.
 *
 * Units:
 *  - x_in, y_in: inches
 *  - theta_rad: radians (CCW positive)
 */
struct Pose {
  double x_in = 0.0;
  double y_in = 0.0;
  double theta_rad = 0.0;
};

/**
 * @brief Robot-centric velocity command produced by the controller.
 *
 * Coordinate convention:
 *  - vx_ips: robot-forward (+X)
 *  - omega_rps: positive CCW yaw rate
 *
 * Units:
 *  - vx_ips: inches per second
 *  - omega_rps: radians per second
 */
struct Command {
  double vx_ips = 0.0;
  double omega_rps = 0.0;
};

/**
 * @brief A trajectory sample with arc-length parameterization.
 *
 * The controller assumes s_in is non-decreasing along the vector.
 *
 * Units:
 *  - x_in, y_in: inches    — position in field frame
 *  - s_in: inches          — cumulative arc-length from trajectory start
 *  - h_in: radians         — desired heading at this point (CCW positive)
 */
struct TrajectorySample {
  double x_in = 0.0;
  double y_in = 0.0;
  double h_in = 0.0;
  double s_in = 0.0;
};

using Trajectory = std::vector<TrajectorySample>;

/**
 * @brief Controller input for a single deterministic step.
 *
 * theta_ref_rad is used only if cfg.use_traj_heading == false.
 * When use_traj_heading is true (default), h_ref comes from the
 * lookahead sample's h_in metadata instead.
 */
struct StepInput {
  Pose pose;
  double theta_ref_rad = 0.0;
};

/**
 * @brief Controller output for a single step.
 *
 * - has_path:     true if a valid trajectory (≥2 samples) is loaded
 * - finished:     true if within pos_tol_in of the endpoint
 *                 (and heading if require_heading_for_finish)
 * - cmd:          robot-centric velocity command {vx, vy, omega}
 * - s_robot_in:   arc-length of the robot's closest point on the path
 * - s_target_in:  arc-length of the lookahead point (s_robot + lookahead_in)
 * - lookahead:    sampled trajectory point at s_target_in
 * - ex_r_in:      body-frame forward error to lookahead [in] (debug/telemetry)
 * - ey_r_in:      body-frame lateral error to lookahead [in] (debug/telemetry)
 * - h_err_rad:    heading error wrapPi(h_ref − θ) [rad]      (debug/telemetry)
 */
struct StepOutput {
  bool has_path = false;
  bool finished = false;
  Command cmd{};
  double s_robot_in  = 0.0;
  double s_target_in = 0.0;
  TrajectorySample lookahead{};
  // --- telemetry ---
  double ex_r_in  = 0.0;
  double ey_r_in  = 0.0;
  double h_err_rad = 0.0;
  double omega_pp_rps = 0.0;
  double v_des_ips = 0.0;
  double adaptive_lookahead_in = 0.0;
};

/**
 * @brief Configuration for Holonomic Pure Pursuit with integrated jerk-limited profiling.
 *
 * All parameters are wired from struct_master.hpp via PipelineConfig::toHppConfig().
 * Do not set these directly in main.cpp.
 */
struct Config {
  double lookahead_in = 12.0; ///< Maximum lookahead distance (L_max) [in].
                              ///< With adaptive: ceiling of L = clamp(k_lookahead_s*v, L_min, L_max).
                              ///< Larger → smoother on straights; smaller → tighter on curves.

  double k_lookahead_s    = 0.3;  ///< Adaptive lookahead gain [s].
                                   ///< L = clamp(k_lookahead_s × v_des_ips, lookahead_min_in, lookahead_in)
                                   ///< Example: v=50 in/s → L=15 in | v=10 in/s → L=3 in
                                   ///< Set to 0.0 to disable (fixed lookahead = lookahead_in).
  double lookahead_min_in = 2.0;  ///< Minimum lookahead when nearly stopped [in] (floor of L).
                                   ///< Must be > min_seg_len_in (0.25 in) to avoid degenerate projection.

  double min_seg_len_in = 0.25; ///< Segments shorter than this are skipped in projection [in].
                                ///< Prevents division-by-zero on degenerate (duplicate) waypoints.
  double s_eps_in = 1e-6;       ///< Floating-point epsilon for arc-length comparisons [in].

  double v_max_ips = 60.0;      ///< Hard speed cap applied after the motion profile [in/s].
                                ///< Acts as a safety ceiling on top of the S-curve output.

  double kHeading = 1.5;        ///< Proportional heading gain [rad/s per rad].
                                ///< omega = clamp(kHeading * h_err, ±omega_max_rps)
                                ///< Higher → faster heading correction but may oscillate.
  double omega_max_rps = 6.0;   ///< Maximum yaw rate command [rad/s].
  double kappa_pp_gain = 1.0;   ///< Gain for geometric PP curvature term (omega_pp).
  double heading_slowdown_gain = 0.0; ///< Optional speed reduction when heading error is large.
  double cross_track_slowdown_gain = 0.0; ///< Optional speed reduction from lateral error: 1/(1+k*ey_r^2).
  double terminal_window_in = 10.0; ///< Near-goal window where proportional terminal speed cap is applied [in].
  double terminal_kp = 1.0; ///< Terminal proportional cap gain [1/s]: v_cap = terminal_kp * remaining.
  double terminal_min_speed_ips = 0.0; ///< Floor for terminal cap to avoid dead-zone (set 0 for max precision stop).
  bool tank_allow_reverse = false; ///< Tank-only: allow negative vx when lookahead falls behind robot.
  bool tank_turn_in_place_on_large_heading_err = true; ///< Tank-only: force in-place turn on large heading error.
  double tank_turn_in_place_err_rad = 25.0 * 3.14159265358979323846 / 180.0; ///< Trigger threshold for turn-in-place.

  /**
   * @brief Use trajectory heading metadata as the heading reference.
   *
   * When true (default): h_ref = lookahead.h_in  → main.cpp stays clean.
   * When false:          h_ref = StepInput.theta_ref_rad  → caller controls heading.
   */
  bool use_traj_heading = true;

  double pos_tol_in = 2.0;               ///< Finish condition: Euclidean dist to endpoint [in].
  bool require_heading_for_finish = false; ///< If true, also require |h_err| < heading_tol_rad.
  double heading_tol_rad = 5.0 * 3.14159265358979323846 / 180.0; ///< Heading finish tolerance [rad] (~5°).

  /**
   * @brief Cross-track correction gain for Tank mode [0, 1].
   *
   * Blends the trajectory heading reference with the geometric bearing
   * from the robot to the lookahead point (field-frame atan2).
   *
   *  k_geo = 0.0 → heading-only control (original behavior, no cross-track)
   *  k_geo = 1.0 → pure geometric bearing (classic chord-to-lookahead steering)
   *  k_geo = 0.5 → recommended: 50% tangent alignment + 50% lateral recovery
   *
   * When on-path, h_geo ≈ look.h_in → blend ≈ look.h_in (zero side effect).
   * When off-path, h_geo steers back toward the lookahead → lateral recovery.
   */
  double k_geo = 0.5;
  bool use_endpoint_distance_guard = true; ///< If true, remaining = max(s_end-s_robot, dist_to_goal).
  bool enforce_monotonic_s = true; ///< If true, s_robot is clamped non-decreasing between steps.
  double max_s_robot_step_in = 3.0; ///< Max forward jump allowed for projected s_robot per control step [in].

  SCurveProfile2Config mp_cfg{};  ///< S-curve profile: max_velocity_rpm, accel, decel, jerk,
                                  ///< wheel diameter, gear ratio. Wired from struct_master.hpp.

  double dt_s = 0.02;            ///< Expected control period [s]. Must match the scheduler period
                                 ///< (20 ms at 50 Hz). Used by the motion profile integrator.
};

class Controller {
public:
  explicit Controller(const Config& cfg);

  /// Loads a new trajectory and resets the motion profile and hint indices.
  /// Trajectory must have s_in non-decreasing.
  void setTrajectory(const Trajectory& traj);

  /// Clears the trajectory and resets internal state. step() returns finished=true.
  void clear();

  /// Main step overload — takes a fully constructed StepInput.
  /// Called every cfg.dt_s (nominally 20 ms).
  StepOutput step(const StepInput& in);

  /// Convenience overload: extracts pose from EKF state, uses explicit theta_ref.
  StepOutput step(const EKF& ekf, double theta_ref_rad);

  /// Convenience overload: extracts pose from EKF state.
  /// Uses lookahead.h_in as h_ref when cfg.use_traj_heading == true (default).
  ///   auto out = hpp.step(ekf);   // typical usage in autonomous()
  StepOutput step(const EKF& ekf);

private:
  /// Wraps angle to [−π, π] using a while loop (avoids fmod precision issues near ±π).
  static double wrapPi(double a_rad);

  /// Returns (a − b) wrapped to [−π, π]. Used for heading error computation.
  static double angleDiff(double a_rad, double b_rad);

  /// Projects the robot pose onto the trajectory and returns the arc-length s_robot.
  ///
  /// For each segment A→B, finds the closest point Q on the segment to robot pose P:
  ///   v  = B − A
  ///   t  = dot(P−A, v) / |v|²,   clamped to [0, 1]
  ///   Q  = A + t * v
  ///   d² = |P − Q|²
  ///   s  = A.s + t * (B.s − A.s)
  ///
  /// The segment with minimum d² wins. Result is clamped to [s_start, s_end].
  /// io_hint_idx: initialized to last winning segment → amortized O(1) for forward motion.
  static double projectPoseToS(const Trajectory& traj, const Pose& pose,
                               double min_seg_len_in, double s_eps_in,
                               std::size_t* io_hint_idx);

  /// Samples the trajectory at a given arc-length via linear interpolation.
  ///
  /// Finds segment i such that traj[i].s ≤ s_target ≤ traj[i+1].s, then:
  ///   α = (s_target − A.s) / (B.s − A.s),   α ∈ [0, 1]
  ///   x = A.x + α*(B.x − A.x)
  ///   y = A.y + α*(B.y − A.y)
  ///   h = wrapPi(A.h + α * wrapPi(B.h − A.h))   ← shortest-arc to avoid wrapping jumps
  ///
  /// s_target is clamped to [s_start, s_end] before lookup.
  /// io_hint_idx is maintained for O(1) amortized forward scan.
  static TrajectorySample sampleAtS(const Trajectory& traj, double s_target_in,
                                    double s_eps_in, std::size_t* io_hint_idx);

  /// Returns true when the finish condition is met:
  ///   dist(pose, traj.back()) < pos_tol_in
  ///   [AND |wrapPi(theta_ref − θ)| < heading_tol_rad   if require_heading_for_finish]
  static bool isFinished(const Trajectory& traj, const Pose& pose, const Config& cfg,
                         double theta_ref_rad);

private:
  Config cfg_;
  Trajectory traj_;

  std::size_t proj_hint_idx_ = 0;  ///< Hint for projectPoseToS: last winning segment index.
  std::size_t samp_hint_idx_ = 0;  ///< Hint for sampleAtS: last winning segment index.
  double last_s_robot_in_ = 0.0;   ///< Last accepted s_robot for monotonic projection.
  bool have_last_s_robot_ = false; ///< Whether last_s_robot_in_ is initialized.
  bool tank_turn_in_place_active_ = false; ///< Hysteresis state for large-heading-error pivot mode.

  SCurveProfile2 mp_;  ///< Jerk-limited S-curve velocity profile (stateful integrator).
};

} // namespace aon::hpp
