#include "aon/autonomy/goto_pose_debug_csv.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>

namespace aon::autonomy {
namespace {

constexpr std::size_t kBufferSize = 2048;
constexpr std::size_t kFlushEveryTicks = 25;
// Emit one CSV row per control tick.
constexpr std::size_t kSampleEveryTicks = 1;

std::array<GoToPoseDebugFrame, kBufferSize> g_buffer{};
std::size_t g_head = 0;
std::size_t g_tail = 0;
std::size_t g_count = 0;
std::size_t g_ticks_since_flush = 0;
std::size_t g_ticks_since_sample = 0;
std::uint64_t g_samples = 0;
std::uint64_t g_dropped = 0;
std::uint64_t g_input_ticks = 0;
bool g_active = false;

double finiteOrNaN(const double v) {
  return std::isfinite(v) ? v : std::numeric_limits<double>::quiet_NaN();
}

void printHeader() {
  std::cout
      << "CSVDBG_HEADER,"
      << "t_ms,dt,"
      << "left_delta,right_delta,ds,dtheta_enc,imu_theta,imu_dtheta,"
      << "x_pred,y_pred,theta_pred,"
      << "x_est,y_est,theta_est,"
      << "Pxx_pred,Pyy_pred,Ptt_pred,Pxx_est,Pyy_est,Ptt_est,"
      << "innovation_theta,innovation_x,innovation_y,NIS_theta,"
      << "dx_update,dy_update,dtheta_update,"
      << "s_robot,s_target,ex_r,ey_r,h_err,lookahead_x,lookahead_y,"
      << "vx_cmd,omega_cmd,"
      << "omega_pp,v_des,adaptive_lookahead\n";
}

void printFrame(const GoToPoseDebugFrame &f) {
  std::cout << std::fixed << std::setprecision(6)
            << "CSVDBG,"
            << f.t_ms << ','
            << finiteOrNaN(f.dt_s) << ','
            << finiteOrNaN(f.left_delta_in) << ','
            << finiteOrNaN(f.right_delta_in) << ','
            << finiteOrNaN(f.ds_in) << ','
            << finiteOrNaN(f.dtheta_enc_rad) << ','
            << finiteOrNaN(f.imu_theta_rad) << ','
            << finiteOrNaN(f.imu_dtheta_rad) << ','
            << finiteOrNaN(f.x_pred_in) << ','
            << finiteOrNaN(f.y_pred_in) << ','
            << finiteOrNaN(f.theta_pred_rad) << ','
            << finiteOrNaN(f.x_est_in) << ','
            << finiteOrNaN(f.y_est_in) << ','
            << finiteOrNaN(f.theta_est_rad) << ','
            << finiteOrNaN(f.Pxx_pred) << ','
            << finiteOrNaN(f.Pyy_pred) << ','
            << finiteOrNaN(f.Ptt_pred) << ','
            << finiteOrNaN(f.Pxx_est) << ','
            << finiteOrNaN(f.Pyy_est) << ','
            << finiteOrNaN(f.Ptt_est) << ','
            << finiteOrNaN(f.innovation_theta) << ','
            << finiteOrNaN(f.innovation_x) << ','
            << finiteOrNaN(f.innovation_y) << ','
            << finiteOrNaN(f.NIS_theta) << ','
            << finiteOrNaN(f.dx_update_in) << ','
            << finiteOrNaN(f.dy_update_in) << ','
            << finiteOrNaN(f.dtheta_update_rad) << ','
            << finiteOrNaN(f.s_robot_in) << ','
            << finiteOrNaN(f.s_target_in) << ','
            << finiteOrNaN(f.ex_r_in) << ','
            << finiteOrNaN(f.ey_r_in) << ','
            << finiteOrNaN(f.h_err_rad) << ','
            << finiteOrNaN(f.lookahead_x_in) << ','
            << finiteOrNaN(f.lookahead_y_in) << ','
            << finiteOrNaN(f.vx_cmd_ips) << ','
            << finiteOrNaN(f.omega_cmd_rps) << ','
            << finiteOrNaN(f.omega_pp_rps) << ','
            << finiteOrNaN(f.v_des_ips) << ','
            << finiteOrNaN(f.adaptive_lookahead_in)
            << '\n';
}

}  // namespace

void goToPoseDebugCsvBegin(const double start_x_in,
                           const double start_y_in,
                           const double start_theta_rad,
                           const double target_x_in,
                           const double target_y_in,
                           const double target_theta_rad,
                           const std::uint32_t timeout_ms) {
  g_head = 0;
  g_tail = 0;
  g_count = 0;
  g_ticks_since_flush = 0;
  g_ticks_since_sample = 0;
  g_samples = 0;
  g_dropped = 0;
  g_input_ticks = 0;
  g_active = true;

  std::cout << "CSVDBG_BEGIN"
            << ",start_x," << start_x_in
            << ",start_y," << start_y_in
            << ",start_theta," << start_theta_rad
            << ",target_x," << target_x_in
            << ",target_y," << target_y_in
            << ",target_theta," << target_theta_rad
            << ",timeout_ms," << timeout_ms << '\n';
  printHeader();
}

void goToPoseDebugCsvPush(const GoToPoseDebugFrame &frame) {
  if (!g_active) return;
  g_input_ticks++;
  g_ticks_since_sample++;
  if (g_ticks_since_sample < kSampleEveryTicks) return;
  g_ticks_since_sample = 0;

  g_buffer[g_head] = frame;
  g_head = (g_head + 1U) % kBufferSize;

  if (g_count < kBufferSize) {
    g_count++;
  } else {
    g_tail = (g_tail + 1U) % kBufferSize;
    g_dropped++;
  }
  g_samples++;
}

void goToPoseDebugCsvFlushIfNeeded() {
  if (!g_active) return;
  g_ticks_since_flush++;
  if (g_ticks_since_flush >= kFlushEveryTicks) {
    goToPoseDebugCsvFlushAll();
    g_ticks_since_flush = 0;
  }
}

void goToPoseDebugCsvFlushAll() {
  if (!g_active) return;

  while (g_count > 0U) {
    printFrame(g_buffer[g_tail]);
    g_tail = (g_tail + 1U) % kBufferSize;
    g_count--;
  }
  std::cout.flush();
}

void goToPoseDebugCsvEnd(const char *status, const std::uint32_t elapsed_ms) {
  if (!g_active) return;
  goToPoseDebugCsvFlushAll();
  std::cout << "CSVDBG_END"
            << ",status," << (status ? status : "unknown")
            << ",elapsed_ms," << elapsed_ms
            << ",ticks_seen," << g_input_ticks
            << ",samples," << g_samples
            << ",dropped," << g_dropped << '\n';
  g_active = false;
}

}  // namespace aon::autonomy
