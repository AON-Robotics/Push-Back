#pragma once

#include <cstdint>

#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/gps.hpp"
#include "EKF.hpp"


namespace aon {

// ============================================================
// EKFConfig
// ============================================================
// This struct is the full control panel for the EKF + sensors.
// All tunable parameters live here: noise models, geometry,
// coordinate conventions, and safety guards.
//
struct EKFConfig {

  // ---------------------------
  // Process noise (Q matrix)
  // ---------------------------
  // How much the filter believes its own motion model is wrong.
  // Larger values -> trust sensors more
  // Smaller values -> trust odometry/model more
  double q_x     = 0.01;   // Noise on X motion
  double q_y     = 0.01;   // Noise on Y motion
  double q_theta = 0.001;  // Noise on heading (rad^2)

  // ---------------------------
  // Default measurement noise (R)
  // ---------------------------
  // These are variances (σ²), not σ.
  // 16 in² = 4 inches RMS GPS error
  double r_gps_x_default_in2 = 16.0;
  double r_gps_y_default_in2 = 16.0;
  double r_theta_default_rad2 = 0.0076;   // IMU heading noise (rad²)

  // ---------------------------
  // GPS update control
  // ---------------------------
  bool use_gps_update = false;          // Enable GPS corrections
  uint32_t gps_min_period_ms = 200;    // Minimum time between GPS updates (rate limit)

  // If >0, force GPS noise instead of RMS or default
  double r_gps_x_fixed_in2 = 16.0;
  double r_gps_y_fixed_in2 = 16.0;

  // ---------------------------
  // IMU configuration
  // ---------------------------
  int imu_theta_sign = 1;  // Flip yaw direction if IMU is mounted reversed
  int imu_omega_sign = 1;  // Flip gyro direction if needed

  bool use_imu_heading_update = true; // Use IMU heading as EKF correction

  // If > 0, overrides r_theta_default_rad2
  // Lets you tune IMU trust without changing defaults
  double r_theta_override_rad2 = 0.0;

  // ---------------------------
  // Odometry geometry
  // ---------------------------
  // Converts encoder rotation into linear distance
  double track_wheel_radius_in = 0.95;  // Radius of tracking wheels
  double rot_to_wheel_ratio = 1.0;     // Gear ratio between encoder and wheel

  // For holonomic drives:
  // Lateral wheel offset from robot center (inches)
  // Needed to remove fake sideways motion when rotating
  double lateral_wheel_x_offset_in = 3.5;


  // ---------------------------
  // Timing safety
  // ---------------------------
  // EKF is invalid if dt is too small or too large
  // This prevents filter blow-ups from timing glitches
  double dt_min_s = 0.001;
  double dt_max_s = 0.2;


  // ---------------------------
  // GPS RMS gating and scaling
  // ---------------------------
  // Uses gps_get_error() to decide how much to trust GPS
  bool gps_use_rms_error = false;

  // Minimum RMS allowed (prevents R → 0 which would destabilize EKF)
  double gps_rms_min_m = 0.01;    // 1 cm

  // If RMS > this, ignore the GPS measurement
  double gps_rms_max_m = 0.50;    // 0.5 meters

  // Hard limits on GPS variance (inches²)
  double gps_r_floor_in2   = 9.0;    // Minimum σ = 3 inches
  double gps_r_ceiling_in2 = 400.0;  // Maximum σ = 20 inches
};


// ============================================================
// SensorFeeder
// ============================================================
// This class is the bridge between physical sensors and the EKF.
//
// It:
//   - Reads encoders, IMU, GPS
//   - Computes velocities and angles
//   - Calls EKF predict() and update()
//
class SensorFeeder {
 public:
  // rotLat == nullptr  -> Tank drive
  // gps == nullptr     -> No GPS present
  SensorFeeder(pros::Rotation& rotL,
               pros::Rotation& rotR,
               pros::Rotation* rotLat,
               pros::Imu& imu,
               pros::Gps* gps,
               const EKFConfig& cfg);

  // Resets time and encoder baselines
  void reset();

  // Loads Q and R defaults into the EKF
  void applyEkfDefaults(EKF& ekf) const;

  // Runs one full EKF update cycle
  void step(EKF& ekf);


  // ---------------------------
  // Debug / telemetry
  // ---------------------------
  double last_dt_s() const { return last_dt_s_; }
  double last_vx_inps() const { return last_vx_inps_; }
  double last_vy_inps() const { return last_vy_inps_; }
  double last_omega_radps() const { return last_omega_radps_; }
  double last_theta_meas_rad() const { return last_theta_meas_rad_; }
  double last_gps_x_in() const { return last_gps_x_in_; }
  double last_gps_y_in() const { return last_gps_y_in_; }

 private:
  // Angle wrapping helpers
  static double wrapPi(double rad);
  static double degToRad(double deg);

  // Encoder → distance conversion
  double rotDeltaIn(pros::Rotation& r, double& prev_centideg);

  // IMU measurements
  double readThetaRad();
  double readYawRateRadps();

  // GPS position + noise estimation
  bool readGpsXY(double& x_in, double& y_in, double& r_x_in2, double& r_y_in2);

  // Snapshot of tuning parameters
  EKFConfig cfg_;

  // Pointers to actual hardware sensors
  pros::Rotation* rotL;
  pros::Rotation* rotR;
  pros::Rotation* rotLat;  // nullptr => Tank
  pros::Imu* imu;
  pros::Gps* gps;          // nullptr => No GPS

  // Internal timing & encoder history
  bool first_run_ = true;
  uint32_t prev_ms_ = 0;
  uint32_t last_gps_ms_ = 0;
  double prev_rotL_cd_ = 0.0;
  double prev_rotR_cd_ = 0.0;
  double prev_rotLat_cd_ = 0.0;

  // Cached last values for debugging
  double last_dt_s_ = 0.0;
  double last_vx_inps_ = 0.0;
  double last_vy_inps_ = 0.0;
  double last_omega_radps_ = 0.0;
  double last_theta_meas_rad_ = 0.0;
  double last_gps_x_in_ = 0.0;
  double last_gps_y_in_ = 0.0;
};

} // namespace aon
