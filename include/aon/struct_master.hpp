// ============================================================
//  CONTROL PANEL — HPP + S-Curve + TankDrive + EKF pipeline
//
//  Sections:
//    0. MOTOR PORTS + SENSOR PORTS — ports drivetrain, encoders, IMU
//    1. ROBOT GEOMETRY        — wheels, track, gear ratio
//    2. S-CURVE PROFILE       — velocity, acceleration, jerk
//    3. HPP CONTROLLER        — lookahead, kHeading, tolerances
//    4. EKF PROCESS NOISE     — matrix Q
//    5. EKF MEASUREMENT NOISE — IMU noise (R)
//    6. SENSOR FEEDER         — odometry geometry, IMU orientation
//    7. INITIAL STATE         — initial robot pose
// ============================================================

#pragma once

#include "EKF/EKF.hpp"
#include "EKF/sensor_feeder.hpp"
#include "aon/controls/holonomic-pure-pursuit/hpp.hpp"
#include "aon/tank-drive/tank-drive.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include <cstdint>

struct PipelineConfig {

  // ===========================================================
  // MOTOR PORTS + SENSOR PORTS
  //    Negative port = reversed motor
  // ===========================================================

  // Drivetrain — left side
  int8_t lm1 = -1;
  int8_t lm2 = 2;
  int8_t lm3 = 3;
  int8_t lm4 = -4;
 
  // Drivetrain — right side 
  int8_t rm1 = 12;
  int8_t rm2 = -13;
  int8_t rm3 = -18;
  int8_t rm4 = 19;
  
  // Rotation sensors (encoders)
  int  encoder_left_port      = 5;
  bool encoder_left_reversed  = false;
  int  encoder_right_port     = 6;
  bool encoder_right_reversed = true;

  // IMU
  int imu_port = 14;

  // Motor cartridge gearset and brake mode
  // green = 200 RPM max | blue = 600 RPM max | red = 100 RPM max
  okapi::AbstractMotor::gearset motor_gearset = okapi::AbstractMotor::gearset::blue;
  okapi::AbstractMotor::brakeMode motor_brakemode = okapi::AbstractMotor::brakeMode::coast;

  // ===========================================================
  // 1. ROBOT GEOMETRY  (Shared with SCurveProfile + TankDrive)
  // ===========================================================

  double wheel_diam_in        = 3.170;     // Wheel diameter  (in)
  double track_width_in       = 14.60;     // Track width (wheel to wheel) (in)
  double motor_to_drive_ratio = 0.5625;      // RPM_motor × ratio = RPM_rueda (-)

  // ===========================================================
  // 2. S-CURVE MOTION PROFILE
  // ===========================================================

  double max_velocity_rpm  = 480.0;   // Maximum velocity (faster)          (RPM)
  double max_accel_rpmps   = 650.0;   // Acceleration ramp (faster)      (RPM/s)
  double max_decel_rpmps   = 500.0;   // Deceleration ramp (faster)   (RPM/s)
  double jerk_rpmps2       = 5500.0;  // Jerk limit (faster)            (RPM/s²)

  // ===========================================================
  // 3. HPP CONTROLLER
  // ===========================================================

  // Dense waypoint tuning (1 in spacing): tighter lookahead + smoother heading
  // for better path fidelity and less "go-stop" behavior.
  double lookahead_in      = 3.6;    // Maximum lookahead (L_max)  (in)
                                      // With adaptive: ceiling of L = clamp(k_lookahead_s*v, L_min, L_max)
                                      // Set to kH × L_max = v_max for steady-state condition (4.0 × 15.0 = 60)

  double k_lookahead_s     = 0.0;    // Adaptive lookahead gain   (s)
                                      // L = clamp(k_lookahead_s * v_des_ips, lookahead_min_in, lookahead_in)
                                      // Example: v=60 in/s, L=15 in | v=10 in/s, L=5 in
                                      // Set to 0.0 to disable (fixed lookahead = lookahead_in)

  double lookahead_min_in  = 3.6;     // Minimum lookahead (floor)  (in)
                                      // Active when the robot is almost stopped

  double kHeading          = 0.82;    // Heading gain (rad/s / rad)
                                      // Tuned so kHeading × lookahead_in = v_max: 4.0 × 15.0 = 60 
  double omega_max_rps     = 1.35;    // Maximum angular velocity  (rad/s)
  double kappa_pp_gain     = 0.08;    // Gain for PP curvature term (omega_pp)
  double heading_slowdown_gain = 1.80; // Reduce linear speed when heading error is large
  double cross_track_slowdown_gain = 0.40; // Reduce speed when lateral error is large
  double terminal_window_in = 10.0;    // Apply terminal speed cap when remaining distance is below this (in)
  double terminal_kp = 1.3;           // Terminal proportional speed cap gain (1/s)
  double terminal_min_speed_ips = 0.15; // Keep command alive near finish without stalling
  bool   use_endpoint_distance_guard = true; // Keep speed command alive near endpoint if XY error remains
  bool   enforce_monotonic_s = true;         // Prevent s_robot from moving backward due to projection jitter
  double max_s_robot_step_in = 1.00;         // Prevent forward projection jumps that can bypass tight loops
  bool   tank_allow_reverse = true;          // Allow true reverse motion on path segments that require backing up
  bool   tank_turn_in_place_on_large_heading_err = true; // Pivot on large heading error to stabilize 90-deg turn
  double tank_turn_in_place_err_deg = 18.0;   // Turn-in-place trigger threshold (deg)
  double pos_tol_in        = 1.2;      // Position tolerance (in)
  bool require_heading_for_finish = true; // Finish gate also checks heading error
  double heading_tol_deg   = 3.0;      // Heading tolerance at endpoint (deg)
  double hpp_dt_s          = 0.010;   // HPP control period (s) -> 100 Hz

  double k_geo             = 0.00;    // Disable geometric blend; use pure trajectory heading for smoother tank tracking
                                      // 0.5: 50% tangent + 50% geometric bearing, lateral recovery
                                      // Decrease to 0.3 if heading oscillates, increase to 0.7 for faster recovery


  // ===========================================================
  // 4. EKF - PROCESS NOISE (diagonal of the Q matrix)
  //    Larger: more trust in sensors than model
  //    Smaller: more trust in model than sensors
  // ===========================================================

  double q_x     = 0.01;    // Movement noise in X (in²/s)
  double q_y     = 0.01;    // Movement noise in Y (in²/s)
  double q_theta = 0.001;   // Heading noise (rad²/s)

  // ===========================================================
  // 5. EKF - MEASUREMENT NOISE (IMU measurement noise)
  //    Larger: less trust in IMU
  //    Smaller: more trust in IMU
  // ===========================================================

  double r_theta_rad2 = 0.0076;   // Variance heading IMU (rad²)

  // ===========================================================
  // 6. SENSOR FEEDER — Odometry geometry + IMU orientation
  // ===========================================================

  double track_wheel_radius_in = 0.988;  // Tracking wheel radius (in)
  double rot_to_wheel_ratio    = 1.00;    // Encoder to wheel ratio

  // PROS get_rotation() / get_gyro_rate().z are CW-positive.
  // EKF convention is CCW-positive (+θ CCW), Sign must be -1 to invert.
  int    imu_theta_sign = -1;  // -1 corrects PROS CW+ to EKF CCW+
  int    imu_omega_sign = -1;  // -1 corrects PROS gyro CW+ → EKF CCW+

  // ===========================================================
  // 7. INITIAL STATE
  // ===========================================================

  double initial_x_in      = 0.0; // Initial X position  (in)
  double initial_y_in      = 0.0;   // Initial Y position  (in)
  double initial_theta_rad = 0.0;   // Fallback initial heading (rad) (0 = +X)
  double init_heading_deg  = 0; // IMU/EKF heading at boot (deg)

  // ===========================================================
  // 8. GO-TO-POSE SAFETY ENVELOPE (footprint vs field boundaries)
  // ===========================================================
  bool go_to_use_field_clearance = true;
  double field_min_x_in = -120.0;
  double field_max_x_in = 120.0;
  double field_min_y_in = -120.0;
  double field_max_y_in = 120.0;
  // 24x24 in chassis footprint -> circumscribed radius = sqrt(12^2 + 12^2) = 16.97 in
  double robot_footprint_radius_in = 16.97;
  double wall_clearance_in = 1.5;

  // ===========================================================
  // CONVERTERS: produce the sub-configs consumed by each subsystem.
  // ===========================================================

  // Config for SensorFeeder (constructor) + applyEkfDefaults().
  aon::EKFConfig toEKFConfig() const {
    static constexpr double kPi = 3.14159265358979323846;
    aon::EKFConfig c{};
    // Process noise
    c.q_x                    = q_x;
    c.q_y                    = q_y;
    c.q_theta                = q_theta;
    c.r_theta_default_rad2   = r_theta_rad2;
    c.track_wheel_radius_in  = track_wheel_radius_in;
    c.track_width_in         = track_width_in;
    c.rot_to_wheel_ratio     = rot_to_wheel_ratio;
    c.imu_theta_sign         = imu_theta_sign;
    c.imu_omega_sign         = imu_omega_sign;
    c.initial_x_in           = initial_x_in;
    c.initial_y_in           = initial_y_in;
    c.initial_theta_rad      = init_heading_deg * (kPi / 180.0);
    c.init_heading_deg       = init_heading_deg;
    c.use_gps_update         = false;
    c.use_imu_heading_update = true;
    return c;
  }

  // Config for the EKF construction
  // The noise Q/R is overwritten after by applyEkfDefaults()
  EKF::TankConfig toEKFTankConfig() const {
    static constexpr double kPi = 3.14159265358979323846;
    EKF::TankConfig c{};
    c.x0.x_in      = initial_x_in;
    c.x0.y_in      = initial_y_in;
    c.x0.theta_rad = init_heading_deg * (kPi / 180.0);
    return c;
  }

  // Config for the HPP controller + motion profile.
  aon::hpp::Config toHppConfig() const {
    aon::hpp::Config c{};
    c.lookahead_in               = lookahead_in;
    c.k_lookahead_s              = k_lookahead_s;
    c.lookahead_min_in           = lookahead_min_in;
    c.kHeading                   = kHeading;
    c.omega_max_rps              = omega_max_rps;
    c.kappa_pp_gain              = kappa_pp_gain;
    c.heading_slowdown_gain      = heading_slowdown_gain;
    c.cross_track_slowdown_gain  = cross_track_slowdown_gain;
    c.terminal_window_in         = terminal_window_in;
    c.terminal_kp                = terminal_kp;
    c.terminal_min_speed_ips     = terminal_min_speed_ips;
    c.use_endpoint_distance_guard = use_endpoint_distance_guard;
    c.enforce_monotonic_s        = enforce_monotonic_s;
    c.max_s_robot_step_in        = max_s_robot_step_in;
    c.tank_allow_reverse         = tank_allow_reverse;
    c.tank_turn_in_place_on_large_heading_err = tank_turn_in_place_on_large_heading_err;
    c.tank_turn_in_place_err_rad = tank_turn_in_place_err_deg * (3.14159265358979323846 / 180.0);
    c.pos_tol_in                 = pos_tol_in;
    c.require_heading_for_finish = require_heading_for_finish;
    c.heading_tol_rad            = heading_tol_deg * (3.14159265358979323846 / 180.0);
    c.use_traj_heading           = true;
    c.k_geo                      = k_geo;
    c.dt_s                       = hpp_dt_s;
    // SCurveProfile2 wired from robot geometry + profile limits
    c.mp_cfg.max_velocity_rpm        = max_velocity_rpm;
    c.mp_cfg.max_accel_rpmps         = max_accel_rpmps;
    c.mp_cfg.max_decel_rpmps         = max_decel_rpmps;
    c.mp_cfg.jerk_rpmps2             = jerk_rpmps2;
    c.mp_cfg.drive_wheel_diameter_in = wheel_diam_in;
    c.mp_cfg.motor_to_drive_ratio    = motor_to_drive_ratio;
    return c;
  }

  // Config for TankDriveFeeder
  aon::TankDriveConfig toTankDriveConfig() const {
    aon::TankDriveConfig c{};
    c.kin.wheel_diam_in  = wheel_diam_in;
    c.kin.track_width_in = track_width_in;
    c.brake = motor_brakemode;
    c.gear  = motor_gearset;
    return c;
  }
};

inline const PipelineConfig kRobotConfig{};
