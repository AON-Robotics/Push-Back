#pragma once

#ifndef AON_AUTONOMY_GOTO_POSE_DEBUG_CSV_HPP_
#define AON_AUTONOMY_GOTO_POSE_DEBUG_CSV_HPP_

#include <cstdint>

namespace aon::autonomy {

struct GoToPoseDebugFrame {
  std::uint32_t t_ms{0};
  double dt_s{0.0};

  double left_delta_in{0.0};
  double right_delta_in{0.0};
  double ds_in{0.0};
  double dtheta_enc_rad{0.0};
  double imu_theta_rad{0.0};
  double imu_dtheta_rad{0.0};

  double x_pred_in{0.0};
  double y_pred_in{0.0};
  double theta_pred_rad{0.0};

  double x_est_in{0.0};
  double y_est_in{0.0};
  double theta_est_rad{0.0};

  double Pxx_pred{0.0};
  double Pyy_pred{0.0};
  double Ptt_pred{0.0};
  double Pxx_est{0.0};
  double Pyy_est{0.0};
  double Ptt_est{0.0};

  double innovation_theta{0.0};
  double innovation_x{0.0};
  double innovation_y{0.0};
  double NIS_theta{0.0};

  double dx_update_in{0.0};
  double dy_update_in{0.0};
  double dtheta_update_rad{0.0};

  double s_robot_in{0.0};
  double s_target_in{0.0};
  double ex_r_in{0.0};
  double ey_r_in{0.0};
  double h_err_rad{0.0};
  double lookahead_x_in{0.0};
  double lookahead_y_in{0.0};

  double vx_cmd_ips{0.0};
  double omega_cmd_rps{0.0};

  double omega_pp_rps{0.0};
  double v_des_ips{0.0};
  double adaptive_lookahead_in{0.0};
};

void goToPoseDebugCsvBegin(double start_x_in,
                           double start_y_in,
                           double start_theta_rad,
                           double target_x_in,
                           double target_y_in,
                           double target_theta_rad,
                           std::uint32_t timeout_ms);

void goToPoseDebugCsvPush(const GoToPoseDebugFrame &frame);
void goToPoseDebugCsvFlushIfNeeded();
void goToPoseDebugCsvFlushAll();
void goToPoseDebugCsvEnd(const char *status, std::uint32_t elapsed_ms);

}  // namespace aon::autonomy

#endif  // AON_AUTONOMY_GOTO_POSE_DEBUG_CSV_HPP_
