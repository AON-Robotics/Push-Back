#include "../../../include/EKF/Sensor_Feeder.hpp"

#include "pros/rtos.hpp"
#include <cmath>

#include "aon/constants.hpp"

namespace aon {

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kMetersToInches = 39.37007874015748;

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

void SensorFeeder::reset() {
  prev_ms_ = pros::millis();
  last_gps_ms_ = prev_ms_;

  prev_rotL_cd_ = rotL->get_position();
  prev_rotR_cd_ = rotR->get_position();
  prev_rotLat_cd_ = rotLat ? rotLat->get_position() : 0.0;

  last_dt_s_ = 0.0;
  last_vx_inps_ = 0.0;
  last_vy_inps_ = 0.0;
  last_omega_radps_ = 0.0;
  last_theta_meas_rad_ = 0.0;
  last_gps_x_in_ = 0.0;
  last_gps_y_in_ = 0.0;

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

  // Step 4: Lateral wheel X-offset compensation (Holonomic only)
  double vy_center_inps = 0.0;
  if (!isTank) {
    vy_center_inps = vy_meas_inps - (omega_radps * cfg_.lateral_wheel_x_offset_in);
    last_vy_inps_ = vy_center_inps;
  }

  // Step 5: EKF Predict
  if (isTank) {
    ekf.predictTank(vx_inps, omega_radps, dt_s);
  } else {
    ekf.predictHolonomic(vx_inps, vy_center_inps, omega_radps, dt_s);
  }

  // Step 6: EKF Update (heading)
  if (cfg_.use_imu_heading_update) {
    double r_theta_rad2 = cfg_.r_theta_default_rad2;
    if (cfg_.r_theta_override_rad2 > 0.0) {
      r_theta_rad2 = cfg_.r_theta_override_rad2;
    }
    ekf.updateHeading(theta_meas_rad, r_theta_rad2);
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
  const double theta_deg = imu->get_rotation();
  const double theta_rad =
      degToRad(theta_deg) * static_cast<double>(cfg_.imu_theta_sign);
  return wrapPi(theta_rad);
}

double SensorFeeder::readYawRateRadps() {
  const auto gyro_rate = imu->get_gyro_rate();
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

double x_m = status.x;
double y_m = status.y;

x_in = (x_m * kMetersToInches) + GPS_X_OFFSET;
y_in = (y_m * kMetersToInches) + GPS_Y_OFFSET;

// Default/fallback variance (in^2)
double R_in2 = cfg_.r_gps_x_default_in2;
if (R_in2 <= 0.0) R_in2 = 16.0;

if (cfg_.gps_use_rms_error) {
const double rms_m = gps->get_error();  // meters RMS (scalar)
if (!std::isfinite(rms_m) || rms_m <= 0.0) return false;

// Gate out bad GPS
if (rms_m > cfg_.gps_rms_max_m) return false;

const double rms_m_clamped =
std::fmax(cfg_.gps_rms_min_m, std::fmin(rms_m, cfg_.gps_rms_max_m));

const double rms_in = rms_m_clamped * kMetersToInches;
const double R_from_rms_in2 = rms_in * rms_in;

// Conservative: never trust GPS MORE than your baseline unless you want to.
// If you DO want GPS to dominate, change fmax -> fmin (pero no lo recomiendo de entrada).
R_in2 = std::fmax(R_in2, R_from_rms_in2);

// Clamp final
R_in2 = std::fmax(cfg_.gps_r_floor_in2, std::fmin(R_in2, cfg_.gps_r_ceiling_in2));
} else {
if (cfg_.r_gps_x_fixed_in2 > 0.0) R_in2 = cfg_.r_gps_x_fixed_in2;
}

r_x_in2 = R_in2;
r_y_in2 = R_in2;
return true;
}

}  // namespace aon
