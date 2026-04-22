#include "aon/autonomy/goto_pose.hpp"
#include "aon/autonomy/goto_pose_debug_csv.hpp"

#include "EKF/EKF.hpp"
#include "EKF/sensor_feeder.hpp"
#include "aon/controls/holonomic-pure-pursuit/hpp.hpp"
#include "aon/struct_master.hpp"
#include "aon/tank-drive/tank-drive.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

extern EKF ekf;
extern aon::SensorFeeder sensorFeeder;
extern aon::hpp::Config hppCfg;
extern aon::hpp::Controller hpp;
extern aon::TankDrive drivetrainTank2;

namespace aon::autonomy {
namespace {

constexpr double kPi = 3.14159265358979323846;
bool g_go_to_pose_csv_debug_enabled = false;

double degToRad(const double deg) {
  return deg * (kPi / 180.0);
}

double wrapPi(double a_rad) {
  while (a_rad > kPi) a_rad -= 2.0 * kPi;
  while (a_rad < -kPi) a_rad += 2.0 * kPi;
  return a_rad;
}

double angleDiff(double a_rad, double b_rad) {
  return wrapPi(a_rad - b_rad);
}

XYPoint clampPointToClearance(const XYPoint &p, const TrajectoryBuildConfig &cfg) {
  if (!cfg.enforce_field_clearance) return p;
  const double margin = std::max(0.0, cfg.robot_radius_in + cfg.wall_clearance_in);
  XYPoint out = p;
  out.x_in = std::clamp(out.x_in, cfg.field_min_x_in + margin, cfg.field_max_x_in - margin);
  out.y_in = std::clamp(out.y_in, cfg.field_min_y_in + margin, cfg.field_max_y_in - margin);
  return out;
}

std::uint32_t controlPeriodMs() {
  const double dt_ms = ::hppCfg.dt_s * 1000.0;
  if (!std::isfinite(dt_ms) || dt_ms <= 0.0) {
    return 20;
  }
  return static_cast<std::uint32_t>(std::max(1.0, std::round(dt_ms)));
}

const char *statusToString(const GoToPoseStatus status) {
  switch (status) {
    case GoToPoseStatus::reached:
      return "reached";
    case GoToPoseStatus::timed_out:
      return "timed_out";
    case GoToPoseStatus::no_path:
    default:
      return "no_path";
  }
}

bool runFinalHeadingLock(const double target_heading_rad,
                         const std::uint32_t period_ms,
                         const std::uint32_t max_lock_ms) {
  const double heading_tol =
      std::max(degToRad(0.8), std::min(::hppCfg.heading_tol_rad, degToRad(2.5)));
  const double kp = std::max(1.0, ::hppCfg.kHeading);
  const double omega_cap = std::max(0.8, ::hppCfg.omega_max_rps);
  const double omega_min = 0.22;

  std::uint32_t wake_ms = pros::millis();
  const std::uint32_t start_ms = wake_ms;
  std::uint32_t good_ticks = 0;

  while ((pros::millis() - start_ms) < max_lock_ms) {
    pros::Task::delay_until(&wake_ms, period_ms);
    ::sensorFeeder.step(::ekf);

    const EKF::State st = ::ekf.getState();
    const double err = wrapPi(target_heading_rad - st.theta_rad);
    const double abs_err = std::abs(err);

    // Hit-and-exit: once heading is inside tolerance, stop and leave immediately.
    if (abs_err <= heading_tol) {
      good_ticks++;
      if (good_ticks >= 2U) {
        ::drivetrainTank2.stop();
        return true;
      }
    } else {
      good_ticks = 0;
    }

    aon::hpp::Command cmd{};
    cmd.vx_ips = 0.0;
    cmd.omega_rps = std::clamp(kp * err, -omega_cap, omega_cap);
    if (std::abs(cmd.omega_rps) < omega_min) {
      cmd.omega_rps = (cmd.omega_rps >= 0.0) ? omega_min : -omega_min;
    }
    ::drivetrainTank2.setChassisVelocity(cmd);
  }

  ::drivetrainTank2.stop();
  return false;
}

}  // namespace

GoToPoseStatus goToPose(double x_in,
                        double y_in,
                        double heading_rad,
                        const GoToPoseOptions &opts) {
  const EKF::State st = ::ekf.getState();

  aon::hpp::Pose start{};
  start.x_in = st.x_in;
  start.y_in = st.y_in;
  start.theta_rad = st.theta_rad;

  aon::hpp::Pose target{};
  // Absolute field-frame targets:
  // x_in/y_in are interpreted directly in the global field frame.
  target.x_in = x_in;
  target.y_in = y_in;
  target.theta_rad = heading_rad;

  TrajectoryBuildConfig build_cfg = opts.build_cfg;
  if (!build_cfg.enforce_field_clearance && kRobotConfig.go_to_use_field_clearance) {
    build_cfg.enforce_field_clearance = true;
    build_cfg.field_min_x_in = kRobotConfig.field_min_x_in;
    build_cfg.field_max_x_in = kRobotConfig.field_max_x_in;
    build_cfg.field_min_y_in = kRobotConfig.field_min_y_in;
    build_cfg.field_max_y_in = kRobotConfig.field_max_y_in;
    build_cfg.robot_radius_in = kRobotConfig.robot_footprint_radius_in;
    build_cfg.wall_clearance_in = kRobotConfig.wall_clearance_in;
  }

  // Reverse-hold-heading mode:
  // If the target lies behind robot-forward (in current robot frame), use a
  // straight point-to-point path to avoid forward-hook curvature from the
  // pose-to-pose control-point builder.
  const double dx = target.x_in - start.x_in;
  const double dy = target.y_in - start.y_in;
  const double c0 = std::cos(start.theta_rad);
  const double s0 = std::sin(start.theta_rad);
  const double forward_component = c0 * dx + s0 * dy;  // local +X component
  const bool reverse_segment = forward_component < -1.0;

  aon::hpp::Trajectory run_traj;
  if (reverse_segment) {
    const XYPoint p0{start.x_in, start.y_in};
    const XYPoint p3 = clampPointToClearance({target.x_in, target.y_in}, build_cfg);
    run_traj = buildTrajectoryFromPoints({p0, p3}, target.theta_rad, build_cfg);
  } else {
    run_traj = buildTrajectoryFromPoseToPose(start, target, build_cfg);
  }

  const bool use_reverse_hold_heading = reverse_segment;

  if (use_reverse_hold_heading) {
    for (auto &smp : run_traj) {
      smp.h_in = target.theta_rad;
    }
  }
  const std::uint32_t period_ms = controlPeriodMs();

  // For reverse segments, align to target heading first so PP doesn't choose a
  // rotate-then-forward solution.
  if (use_reverse_hold_heading) {
    (void)runFinalHeadingLock(target.theta_rad, period_ms, 1400U);
  }

  std::cout << "GTP_BEGIN"
            << ",start_x," << start.x_in
            << ",start_y," << start.y_in
            << ",start_h," << start.theta_rad
            << ",target_x," << target.x_in
            << ",target_y," << target.y_in
            << ",target_h," << target.theta_rad
            << ",forward_component," << forward_component
            << ",reverse_hold_heading," << static_cast<int>(use_reverse_hold_heading)
            << ",timeout_ms," << opts.timeout_ms
            << ",period_ms," << period_ms
            << ",field_clearance," << static_cast<int>(build_cfg.enforce_field_clearance)
            << ",robot_radius_in," << build_cfg.robot_radius_in
            << ",wall_clearance_in," << build_cfg.wall_clearance_in
            << ",traj_pts," << run_traj.size()
            << '\n';

  if (run_traj.size() < 2) {
    ::drivetrainTank2.stop();
    ::hpp.clear();
    std::cout << "GTP_END"
              << ",status," << statusToString(GoToPoseStatus::no_path)
              << ",reason,traj_too_short\n";
    return GoToPoseStatus::no_path;
  }

  ::hpp.setTrajectory(run_traj);

  std::uint32_t wake_ms = pros::millis();
  const std::uint32_t start_ms = wake_ms;
  const bool csv_debug_enabled = g_go_to_pose_csv_debug_enabled;
  if (csv_debug_enabled) {
    goToPoseDebugCsvBegin(start.x_in,
                          start.y_in,
                          start.theta_rad,
                          target.x_in,
                          target.y_in,
                          target.theta_rad,
                          opts.timeout_ms);
  }
  const auto endCsvDebug = [&](const char *status) {
    if (!csv_debug_enabled) return;
    goToPoseDebugCsvEnd(status, pros::millis() - start_ms);
  };

  std::uint32_t tick = 0;
  const double s_end = run_traj.back().s_in;
  const double pos_guard_in = std::max(2.8, ::hppCfg.pos_tol_in);
  const double remaining_guard_in =
      std::max(pos_guard_in, std::min(::hppCfg.terminal_window_in, 3.5));

  while (true) {
    pros::Task::delay_until(&wake_ms, period_ms);
    tick++;

    ::sensorFeeder.step(::ekf);
    const aon::hpp::StepOutput out = ::hpp.step(::ekf);
    const EKF::State live = ::ekf.getState();
    const EKF::DebugSnapshot ekf_dbg = ::ekf.getDebugSnapshot();
    const aon::SensorFeeder::StepDebug &sensor_dbg = ::sensorFeeder.last_debug();
    const double dx_goal = target.x_in - live.x_in;
    const double dy_goal = target.y_in - live.y_in;
    const double dist_goal_in = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
    const std::uint32_t elapsed_ms = pros::millis() - start_ms;

    if ((tick % 25U) == 0U) {
      const double remaining_in = std::max(0.0, s_end - out.s_robot_in);
      std::cout << "GTP_TICK"
                << ",tick," << tick
                << ",s_robot," << out.s_robot_in
                << ",s_target," << out.s_target_in
                << ",remaining," << remaining_in
                << ",dist_goal_in," << dist_goal_in
                << ",has_path," << static_cast<int>(out.has_path)
                << ",finished," << static_cast<int>(out.finished)
                << '\n';
    }

    const double remaining_in = std::max(0.0, s_end - out.s_robot_in);

    if (csv_debug_enabled) {
      GoToPoseDebugFrame frame{};
      frame.t_ms = elapsed_ms;
      frame.dt_s = sensor_dbg.dt_s;
      frame.left_delta_in = sensor_dbg.left_delta_in;
      frame.right_delta_in = sensor_dbg.right_delta_in;
      frame.ds_in = sensor_dbg.ds_in;
      frame.dtheta_enc_rad = sensor_dbg.dtheta_enc_rad;
      frame.imu_theta_rad = sensor_dbg.imu_theta_rad;
      frame.imu_dtheta_rad = sensor_dbg.imu_dtheta_rad;

      frame.x_pred_in = ekf_dbg.x_pred_in;
      frame.y_pred_in = ekf_dbg.y_pred_in;
      frame.theta_pred_rad = ekf_dbg.theta_pred_rad;

      frame.x_est_in = live.x_in;
      frame.y_est_in = live.y_in;
      frame.theta_est_rad = live.theta_rad;

      frame.Pxx_pred = ekf_dbg.Pxx_pred;
      frame.Pyy_pred = ekf_dbg.Pyy_pred;
      frame.Ptt_pred = ekf_dbg.Ptt_pred;
      frame.Pxx_est = ekf_dbg.Pxx_est;
      frame.Pyy_est = ekf_dbg.Pyy_est;
      frame.Ptt_est = ekf_dbg.Ptt_est;

      frame.innovation_theta = ekf_dbg.innovation_theta;
      frame.innovation_x = ekf_dbg.innovation_x;
      frame.innovation_y = ekf_dbg.innovation_y;
      frame.NIS_theta = ekf_dbg.NIS_theta;

      frame.dx_update_in = ekf_dbg.dx_update_in;
      frame.dy_update_in = ekf_dbg.dy_update_in;
      frame.dtheta_update_rad = ekf_dbg.dtheta_update_rad;

      frame.s_robot_in = out.s_robot_in;
      frame.s_target_in = out.s_target_in;
      frame.ex_r_in = out.ex_r_in;
      frame.ey_r_in = out.ey_r_in;
      frame.h_err_rad = out.h_err_rad;
      frame.lookahead_x_in = out.lookahead.x_in;
      frame.lookahead_y_in = out.lookahead.y_in;

      frame.vx_cmd_ips = out.cmd.vx_ips;
      frame.omega_cmd_rps = out.cmd.omega_rps;
      frame.omega_pp_rps = out.omega_pp_rps;
      frame.v_des_ips = out.v_des_ips;
      frame.adaptive_lookahead_in = out.adaptive_lookahead_in;

      goToPoseDebugCsvPush(frame);
      goToPoseDebugCsvFlushIfNeeded();
    }

    if (!out.has_path) {
      ::drivetrainTank2.stop();
      ::hpp.clear();
      const EKF::State end = ::ekf.getState();
      std::cout << "GTP_END"
                << ",status," << statusToString(GoToPoseStatus::no_path)
                << ",elapsed_ms," << (pros::millis() - start_ms)
                << ",end_x," << end.x_in
                << ",end_y," << end.y_in
                << ",end_h," << end.theta_rad
                << '\n';
      endCsvDebug("no_path");
      return GoToPoseStatus::no_path;
    }

    if (out.finished) {
      ::drivetrainTank2.stop();
      ::hpp.clear();
      const std::uint32_t elapsed_ms = pros::millis() - start_ms;
      std::uint32_t lock_budget_ms = 2500U;
      if (opts.timeout_ms > 0 && elapsed_ms < opts.timeout_ms) {
        lock_budget_ms = std::min(lock_budget_ms, opts.timeout_ms - elapsed_ms);
      }
      const bool heading_ok = runFinalHeadingLock(target.theta_rad, period_ms, lock_budget_ms);
      const EKF::State end = ::ekf.getState();
      if (!heading_ok) {
        std::cout << "GTP_END"
                  << ",status," << statusToString(GoToPoseStatus::timed_out)
                  << ",reason,heading_lock_timeout"
                  << ",elapsed_ms," << (pros::millis() - start_ms)
                  << ",end_x," << end.x_in
                  << ",end_y," << end.y_in
                  << ",end_h," << end.theta_rad
                  << '\n';
        endCsvDebug("timed_out");
        return GoToPoseStatus::timed_out;
      }
      std::cout << "GTP_END"
                << ",status," << statusToString(GoToPoseStatus::reached)
                << ",elapsed_ms," << (pros::millis() - start_ms)
                << ",end_x," << end.x_in
                << ",end_y," << end.y_in
                << ",end_h," << end.theta_rad
                << '\n';
      endCsvDebug("reached");
      return GoToPoseStatus::reached;
    }

    // As soon as XY goal is reached, leave HPP and finish heading with dedicated lock.
    // This prevents end-of-path heading hunting from polluting the next segment start.
    if (dist_goal_in <= pos_guard_in || remaining_in <= remaining_guard_in) {
      ::drivetrainTank2.stop();
      ::hpp.clear();
      const std::uint32_t elapsed_ms = pros::millis() - start_ms;
      std::uint32_t lock_budget_ms = 2500U;
      if (opts.timeout_ms > 0 && elapsed_ms < opts.timeout_ms) {
        lock_budget_ms = std::min(lock_budget_ms, opts.timeout_ms - elapsed_ms);
      }
      const bool heading_ok = runFinalHeadingLock(target.theta_rad, period_ms, lock_budget_ms);
      const EKF::State end = ::ekf.getState();
      if (!heading_ok) {
        std::cout << "GTP_END"
                  << ",status," << statusToString(GoToPoseStatus::timed_out)
                  << ",reason,heading_lock_timeout"
                  << ",elapsed_ms," << (pros::millis() - start_ms)
                  << ",end_x," << end.x_in
                  << ",end_y," << end.y_in
                  << ",end_h," << end.theta_rad
                  << '\n';
        endCsvDebug("timed_out");
        return GoToPoseStatus::timed_out;
      }
      std::cout << "GTP_END"
                << ",status," << statusToString(GoToPoseStatus::reached)
                << ",reason,terminal_guard"
                << ",elapsed_ms," << (pros::millis() - start_ms)
                << ",end_x," << end.x_in
                << ",end_y," << end.y_in
                << ",end_h," << end.theta_rad
                << '\n';
      endCsvDebug("reached");
      return GoToPoseStatus::reached;
    }

    ::drivetrainTank2.setChassisVelocity(out.cmd);

    if (opts.timeout_ms > 0 && (pros::millis() - start_ms) >= opts.timeout_ms) {
      ::drivetrainTank2.stop();
      ::hpp.clear();
      const EKF::State end = ::ekf.getState();
      std::cout << "GTP_END"
                << ",status," << statusToString(GoToPoseStatus::timed_out)
                << ",elapsed_ms," << (pros::millis() - start_ms)
                << ",end_x," << end.x_in
                << ",end_y," << end.y_in
                << ",end_h," << end.theta_rad
                << '\n';
      endCsvDebug("timed_out");
      return GoToPoseStatus::timed_out;
    }
  }
}

GoToPoseStatus goToPoseDeg(double x_in,
                           double y_in,
                           double heading_deg,
                           const GoToPoseOptions &opts) {
  return goToPose(x_in, y_in, degToRad(heading_deg), opts);
}

void setGoToPoseCsvDebugEnabled(const bool enable) {
  g_go_to_pose_csv_debug_enabled = enable;
}

bool isGoToPoseCsvDebugEnabled() {
  return g_go_to_pose_csv_debug_enabled;
}

}  // namespace aon::autonomy
