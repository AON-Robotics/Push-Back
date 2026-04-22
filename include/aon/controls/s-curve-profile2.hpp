/**
 * @file s-curve-profile2.hpp
 * @brief Jerk-limited S-curve motion profile (distance-to-go driven).
 *
 * Purpose:
 *   Provide a deterministic speed setpoint v(s) for path following.
 *
 * Input:
 *   remainingDist_in  = (s_end - s_robot) in inches  (ARC-LENGTH remaining)
 *   dt_s              = control period in seconds (e.g., 0.02)
 *
 * Output:
 *   target speed in inches/sec (in/s)
 */

 #pragma once

 #include <algorithm>
 #include <cmath>
 
 // ============================================================
 // SCurveProfile2Config (control panel)
 // ============================================================
 // Keep ONLY what matters for tuning + unit conversions.
 struct SCurveProfile2Config {
   // Limits (motor space)
   double max_velocity_rpm = 600.0;   // [rpm]
   double max_accel_rpmps  = 4000.0;  // [rpm/s]
   double max_decel_rpmps  = 3000.0;  // [rpm/s] (magnitude)
   double jerk_rpmps2      = 8000.0;  // [rpm/s^2]
 
   // Geometry / gearing
   // wheel_rpm = motor_rpm * motor_to_drive_ratio
   double drive_wheel_diameter_in = 3.2; // [in]
   double motor_to_drive_ratio    = 1.0;  // [-]
 
   // Timing safety (clamp dt to avoid blowups from jitter)
   double dt_min_s = 0.005;
   double dt_max_s = 0.05;
 };
 
 // ============================================================
 // SCurveProfile2 (stateful)
 // ============================================================
 class SCurveProfile2 {
 public:
   explicit SCurveProfile2(const SCurveProfile2Config& cfg = SCurveProfile2Config())
       : cfg_(cfg) {}
 
   void setConfig(const SCurveProfile2Config& cfg) { cfg_ = cfg; }
   const SCurveProfile2Config& config() const { return cfg_; }
 
   /// @brief Main update (preferred): returns speed command in inches/sec.
   /// @param remainingDist_in Arc-length remaining to goal (s_end - s_robot) [in]
   /// @param dt_s Control period [s] (use fixed 0.02 if your scheduler is fixed-rate)
   double update(double remainingDist_in, double dt_s) {
     const double h = std::clamp(dt_s, cfg_.dt_min_s, cfg_.dt_max_s);
 
     // If done (or bad input), stop
     if (remainingDist_in <= 0.0) {
       vel_ips_ = 0.0;
       accel_ips2_ = 0.0;
       return 0.0;
     }
 
     // ---- unit conversion: RPM -> in/s (linear)
     const double wheel_circ_in = cfg_.drive_wheel_diameter_in * std::acos(-1.0);
     const double ips_per_rpm   = (cfg_.motor_to_drive_ratio * wheel_circ_in) / 60.0;
 
     const double vmax_ips   = std::abs(cfg_.max_velocity_rpm) * ips_per_rpm;
     const double amax_ips2  = std::abs(cfg_.max_accel_rpmps)  * ips_per_rpm;
     const double dmax_ips2  = std::abs(cfg_.max_decel_rpmps)  * ips_per_rpm;
     const double jmax_ips3  = std::abs(cfg_.jerk_rpmps2)      * ips_per_rpm;
 
     // ---- physically correct stopping cap: v^2 <= 2 * a_dec * dist
     const double vcap_stop_ips = std::sqrt(std::max(0.0, 2.0 * dmax_ips2 * remainingDist_in));
     const double vcap_ips      = std::clamp(std::min(vmax_ips, vcap_stop_ips), 0.0, vmax_ips);
 
     // ---- choose accel direction to approach the current cap
     const double vel_error = vcap_ips - vel_ips_;
 
     double accel_target = 0.0;
     if (vel_error > 1e-6)      accel_target = +amax_ips2;
     else if (vel_error < -1e-6) accel_target = -dmax_ips2;
     else                       accel_target = 0.0;
 
     // ---- jerk-limit acceleration toward accel_target
     const double max_delta_a = jmax_ips3 * h;
 
     if (accel_ips2_ < accel_target)      accel_ips2_ = std::min(accel_ips2_ + max_delta_a, accel_target);
     else if (accel_ips2_ > accel_target) accel_ips2_ = std::max(accel_ips2_ - max_delta_a, accel_target);
 
     accel_ips2_ = std::clamp(accel_ips2_, -dmax_ips2, +amax_ips2);
 
     // ---- integrate velocity + enforce cap
     vel_ips_ += accel_ips2_ * h;
     vel_ips_  = std::clamp(vel_ips_, 0.0, vcap_ips);
 
     return vel_ips_;
   }
 
   void reset() {
     vel_ips_ = 0.0;
     accel_ips2_ = 0.0;
   }
 
   double velocityIPS() const { return vel_ips_; }
   double accelIPS2() const { return accel_ips2_; }
 
 private:
   SCurveProfile2Config cfg_{};
   double vel_ips_ = 0.0;       // [in/s]
   double accel_ips2_ = 0.0;    // [in/s^2]
 };
 
