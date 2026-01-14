// src/aon/EKF/EKFDebug.cpp
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "pros/gps.hpp"

#include <cmath>

#include "EKF/EKF.hpp"
#include "EKF/Sensor_Feeder.hpp"

// Call from opcontrol() and/or autonomous loops.
// Requires pros::lcd::initialize() called once in initialize().
void ekf_lcd_debug_update(const aon::SensorFeeder& feeder,
                          const EKF& ekf,
                          const pros::Imu& imu,
                          const pros::Gps* gps) {
  static uint32_t last_print_ms = 0;
  const uint32_t now = pros::millis();
  if (now - last_print_ms < 100) return;  // 10 Hz
  last_print_ms = now;

  // dt_ms
  const int dt_ms = static_cast<int>(feeder.last_dt_s() * 1000.0);

  // vx_inps
  const double vx_inps = feeder.last_vx_inps();

  // ekf pose
  const EKF::State s = ekf.getState();

  // theta_deg (IMU direct)
  const double theta_deg = imu.get_rotation();

  // gpsErr_cm (gps_get_error() returns RMS error in meters)
  double gpsErr_cm = -1.0;
  if (gps) {
    const double rms_m = gps->get_error();
    if (std::isfinite(rms_m) && rms_m > 0.0) {
      gpsErr_cm = rms_m * 100.0;
    }
  }

  pros::lcd::print(0, "dt_ms: %d", dt_ms);
  pros::lcd::print(1, "vx_inps: %.2f", vx_inps);
  pros::lcd::print(2, "ekf.x_in: %.1f", s.x_in);
  pros::lcd::print(3, "ekf.y_in: %.1f", s.y_in);
  pros::lcd::print(4, "theta_deg: %.1f", theta_deg);
  pros::lcd::print(5, "gpsErr_cm: %.1f", gpsErr_cm);
}
