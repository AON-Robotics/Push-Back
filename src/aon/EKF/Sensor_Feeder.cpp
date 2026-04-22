#include "EKF/sensor_feeder.hpp"

#include "pros/rtos.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <utility>

#include "aon/constants.hpp"

namespace aon {

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kMetersToInches = 39.37007874015748;

double quietNaN() {
  return std::numeric_limits<double>::quiet_NaN();
}

SensorFeeder::SensorFeeder(pros::Rotation& rotL_in,
                           pros::Rotation& rotR_in,
                           pros::Rotation* rotLat_in,  // nullptr => Tank
                           pros::Imu& imu_in,
                           pros::Gps* gps_in,          // nullptr => no GPS
                           const EKFConfig& cfg)
    : cfg_(cfg),
      rotL(&rotL_in),
      rotR(&rotR_in),
      rotLat(rotLat_in),
      imu(&imu_in),
      gps(gps_in) {}

void SensorFeeder::reset(EKF* ekf) {
  if (ekf != nullptr) {
    // Force sensor references to absolute zero for this run.
    rotL->reset_position();
    rotR->reset_position();
    if (rotLat) rotLat->reset_position();

    double x0_in = cfg_.initial_x_in;
    double y0_in = cfg_.initial_y_in;
    double theta0_rad = degToRad(cfg_.init_heading_deg);

    EKF::State s{};
    s.x_in = x0_in;
    s.y_in = y0_in;
    s.theta_rad = theta0_rad;
    ekf->setState(s);
    std::printf("INIT_POSE,%.3f,%.3f,%.3f,src(hardcoded)\n", x0_in, y0_in, cfg_.init_heading_deg);
    std::fflush(stdout);

    double P0[3][3] = {
      {25.0, 0.0, 0.0},
      {0.0, 25.0, 0.0},
      {0.0, 0.0, (10.0 * kPi / 180.0) * (10.0 * kPi / 180.0)}
    };
    ekf->setP(P0);
  }

  prev_ms_ = pros::millis();
  last_gps_ms_ = prev_ms_;

  prev_rotL_cd_ = rotL->get_position();
  prev_rotR_cd_ = rotR->get_position();
  prev_rotLat_cd_ = rotLat ? rotLat->get_position() : 0.0;

  last_dt_s_ = 0.0;
  last_vx_inps_ = 0.0;
  last_vy_inps_ = 0.0;
  last_omega_radps_ = 0.0;
  last_theta_meas_rad_ = degToRad(cfg_.init_heading_deg);
  last_gps_x_in_ = 0.0;
  last_gps_y_in_ = 0.0;
  last_debug_ = {};
  last_debug_.imu_theta_rad = last_theta_meas_rad_;
  last_debug_.dtheta_enc_rad = quietNaN();

  first_run_ = false;
}

void SensorFeeder::applyEkfDefaults(EKF& ekf) const {
  ekf.setProcessNoise(cfg_.q_x, cfg_.q_y, cfg_.q_theta);
  ekf.setDefaultMeasurementNoise(cfg_.r_gps_x_default_in2,
                                 cfg_.r_gps_y_default_in2,
                                 cfg_.r_theta_default_rad2);
}

void SensorFeeder::step(EKF& ekf) {
  const bool isTank = (rotLat == nullptr);

  // Step 0: dt
  const uint32_t now_ms = pros::millis();
  if (first_run_) {
    reset();
    return;
  }

  const uint32_t dt_ms = now_ms - prev_ms_;
  const double dt_s = static_cast<double>(dt_ms) / 1000.0;

  if (dt_s < cfg_.dt_min_s || dt_s > cfg_.dt_max_s) {
    // Timing guard: resync if the loop timing is out of bounds.
    prev_ms_ = now_ms;
    prev_rotL_cd_ = rotL->get_position();
    prev_rotR_cd_ = rotR->get_position();
    prev_rotLat_cd_ = rotLat ? rotLat->get_position() : 0.0;
    last_dt_s_ = 0.0;
    last_debug_.dt_s = dt_s;
    last_debug_.left_delta_in = 0.0;
    last_debug_.right_delta_in = 0.0;
    last_debug_.ds_in = 0.0;
    last_debug_.dtheta_enc_rad = quietNaN();
    last_debug_.imu_dtheta_rad = 0.0;
    last_debug_.vx_inps = 0.0;
    last_debug_.omega_radps = 0.0;
    return;
  }

  prev_ms_ = now_ms;
  last_dt_s_ = dt_s;

  // Step 1: Encoder deltas -> distances
  double dL_in = rotDeltaIn(*rotL, prev_rotL_cd_);
  double dR_in = rotDeltaIn(*rotR, prev_rotR_cd_);
  double dLat_in = 0.0;

  if (!isTank && rotLat) {
    dLat_in = rotDeltaIn(*rotLat, prev_rotLat_cd_);
  }

  last_debug_.dt_s = dt_s;
  last_debug_.left_delta_in = dL_in;
  last_debug_.right_delta_in = dR_in;
  last_debug_.ds_in = (dL_in + dR_in) * 0.5;
  if (cfg_.track_width_in > 1e-9) {
    last_debug_.dtheta_enc_rad = (dR_in - dL_in) / cfg_.track_width_in;
  } else {
    last_debug_.dtheta_enc_rad = quietNaN();
  }

  // Step 2: Compute body velocities
  const double vx_inps = ((dL_in + dR_in) * 0.5) / dt_s;

  double vy_meas_inps = 0.0;
  if (!isTank) {
    vy_meas_inps = dLat_in / dt_s;
  }

  last_vx_inps_ = vx_inps;
  last_vy_inps_ = (isTank ? 0.0 : vy_meas_inps);

  // Step 3: IMU heading + yaw rate
  const double theta_meas_rad = readThetaRad();
  const double omega_radps = readYawRateRadps();
  last_theta_meas_rad_ = theta_meas_rad;
  last_omega_radps_ = omega_radps;
  last_debug_.imu_theta_rad = theta_meas_rad;
  last_debug_.imu_dtheta_rad = omega_radps * dt_s;
  last_debug_.vx_inps = vx_inps;
  last_debug_.omega_radps = omega_radps;

  // Step 4: Lateral wheel X-offset compensation (Holonomic only)
  double vy_center_inps = 0.0;
  if (!isTank) {
    vy_center_inps = vy_meas_inps - (omega_radps * cfg_.lateral_wheel_x_offset_in);
    last_vy_inps_ = vy_center_inps;
  }

  // Step 5: EKF Predict
  // Tank: predict x/y/theta from forward speed and IMU yaw rate.
  // H-Drive: same, but with body-frame vx/vy.
  if (isTank) {
    ekf.predictTank(vx_inps, omega_radps, dt_s);
  } else {
    ekf.predictHolonomic(vx_inps, vy_center_inps, omega_radps, dt_s);
  }

  // Step 6: IMU heading correction
  // Keep IMU as the absolute heading measurement, but fuse it formally
  // through the EKF measurement update instead of overwriting theta directly.
  if (cfg_.use_imu_heading_update) {
    const double r_theta_in2 =
        (cfg_.r_theta_override_rad2 > 0.0) ? cfg_.r_theta_override_rad2 : -1.0;
    ekf.updateHeading(theta_meas_rad, r_theta_in2);
  }

  // Step 7: EKF Update (GPS) with rate limiting
  if (cfg_.use_gps_update && gps) {
    const uint32_t gps_elapsed_ms = now_ms - last_gps_ms_;
    if (cfg_.gps_min_period_ms == 0 || gps_elapsed_ms >= cfg_.gps_min_period_ms) {
      double x_in = 0.0;
      double y_in = 0.0;
      double r_x_in2 = 0.0;
      double r_y_in2 = 0.0;

      if (readGpsXY(x_in, y_in, r_x_in2, r_y_in2)) {
        ekf.updateGPS(x_in, y_in, r_x_in2, r_y_in2);
        last_gps_ms_ = now_ms;
        last_gps_x_in_ = x_in;
        last_gps_y_in_ = y_in;
      }
    }
  }
}

double SensorFeeder::wrapPi(double rad) {
  double wrapped = rad;
  while (wrapped > kPi) wrapped -= 2.0 * kPi;
  while (wrapped < -kPi) wrapped += 2.0 * kPi;
  return wrapped;
}

double SensorFeeder::degToRad(double deg) {
  return deg * (kPi / 180.0);
}

double SensorFeeder::rotDeltaIn(pros::Rotation& r, double& prev_centideg) {
  const double now_cd = r.get_position();
  const double d_cd = now_cd - prev_centideg;
  prev_centideg = now_cd;

  double d_rad = (d_cd / 100.0) * (kPi / 180.0);
  d_rad *= cfg_.rot_to_wheel_ratio;
  return cfg_.track_wheel_radius_in * d_rad;
}

double SensorFeeder::readThetaRad() {
  const auto st = imu->get_status();
  if (st == pros::c::E_IMU_STATUS_CALIBRATING) {
    return last_theta_meas_rad_;
  }

  const double theta_deg = imu->get_rotation();
  // Guard: PROS_ERR_F is a huge finite value that would cause wrapPi()
  // to loop millions of times and hang the control loop.
  if (!std::isfinite(theta_deg)) return last_theta_meas_rad_;
  const double theta_rad =
      degToRad(theta_deg) * static_cast<double>(cfg_.imu_theta_sign);
  return wrapPi(theta_rad);
}

double SensorFeeder::readYawRateRadps() {
  const auto st = imu->get_status();
  if (st == pros::c::E_IMU_STATUS_CALIBRATING) {
    return 0.0;
  }

  const auto gyro_rate = imu->get_gyro_rate();
  if (!std::isfinite(gyro_rate.z)) return last_omega_radps_;
  const double omega_radps =
      degToRad(gyro_rate.z) * static_cast<double>(cfg_.imu_omega_sign);
  return omega_radps;
}

bool SensorFeeder::readGpsXY(double& x_in,
    double& y_in,
    double& r_x_in2,
    double& r_y_in2) {
  if (!gps) return false;

  const auto status = gps->get_status();
  if (!std::isfinite(status.x) || !std::isfinite(status.y)) return false;

  // Raw GPS in meters
  double x_m = status.x;
  double y_m = status.y;

  // Apply mounting offset in METERS (sensor position relative to robot center)
  double x_m_center = x_m + GPS_X_OFFSET;
  double y_m_center = y_m + GPS_Y_OFFSET;

  // Convert to inches
  double x_in_raw = x_m_center * kMetersToInches;
  double y_in_raw = y_m_center * kMetersToInches;

  // Handle rotated GPS mounting
  if (cfg_.gps_swap_xy) {
  std::swap(x_in_raw, y_in_raw);
  }

  // Handle flipped axes
  x_in_raw *= static_cast<double>(cfg_.gps_x_sign);
  y_in_raw *= static_cast<double>(cfg_.gps_y_sign);

  // Apply final small alignment biases (inches)
  x_in = x_in_raw + cfg_.gps_x_bias_in;
  y_in = y_in_raw + cfg_.gps_y_bias_in;

  // -------------------------------
  // Noise model (R matrix)
  // -------------------------------
  double R_in2 = cfg_.r_gps_x_default_in2;
  if (R_in2 <= 0.0) R_in2 = 16.0;  // fallback = 4" RMS

  if (cfg_.gps_use_rms_error) {
  const double rms_m = gps->get_error();  // meters RMS (scalar)
  if (!std::isfinite(rms_m) || rms_m <= 0.0) return false;

  // Reject bad GPS fixes
  if (rms_m > cfg_.gps_rms_max_m) return false;

  const double rms_m_clamped =
  std::fmax(cfg_.gps_rms_min_m, std::fmin(rms_m, cfg_.gps_rms_max_m));

  const double rms_in = rms_m_clamped * kMetersToInches;
  const double R_from_rms_in2 = rms_in * rms_in;

  // Conservative: do not trust GPS more than baseline unless explicitly desired
  R_in2 = std::fmax(R_in2, R_from_rms_in2);

  // Hard clamp
  R_in2 = std::fmax(cfg_.gps_r_floor_in2,
  std::fmin(R_in2, cfg_.gps_r_ceiling_in2));
  } else {
    if (cfg_.r_gps_x_fixed_in2 > 0.0) {
    R_in2 = cfg_.r_gps_x_fixed_in2;
  }
  }

  r_x_in2 = R_in2;
  r_y_in2 = R_in2;
  return true;
  }

}  // namespace aon
