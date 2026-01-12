// src/aon/EKF/EKFDebug.cpp
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pros/imu.hpp"
#include "pros/gps.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.hpp"

#include <cmath>

#include "EKF/EKF.hpp"
#include "EKF/Sensor_Feeder.hpp"

extern EKF ekf;
extern pros::Imu imu;
extern pros::Rotation encoderLeft;
extern pros::Rotation encoderRight;
extern pros::Rotation encoderLateral;

static void DisplayDebugMenu4(const EKF& ekf,
                              const pros::Imu& imu,
                              pros::Rotation& encoderLeft,
                              pros::Rotation& encoderRight,
                              pros::Rotation* encoderLateral) {
  static uint32_t last_draw_ms = 0;
  const uint32_t now = pros::millis();
  if (now - last_draw_ms < 100) return;  // 10 Hz
  last_draw_ms = now;

  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Title
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "EKF ODOMETRY & SENSORS");

  // BACK button
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(10, 10, 90, 40);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 18, "BACK");

  const EKF::State s = ekf.getState();
  const double theta_deg = s.theta_rad * (180.0 / M_PI);
  int y = 60;

  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "EKF X: %.2f in", s.x_in); y += 30;
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "EKF Y: %.2f in", s.y_in); y += 30;
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "EKF Th: %.2f deg", theta_deg); y += 30;

#if GYRO_ENABLED
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "IMU: %.2f deg", imu.get_rotation()); y += 30;
#endif

  const bool has_lateral =
      (encoderLateral != nullptr) && (ekf.mode() == EKF::Mode::HDrive);
  if (has_lateral) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Enc Right: %.2f deg",
                        encoderRight.get_position() / 100.0); y += 30;
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Enc Lat: %.2f deg",
                        encoderLateral->get_position() / 100.0); y += 30;
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Enc Left: %.2f deg",
                        encoderLeft.get_position() / 100.0); y += 30;
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Enc Right: %.2f deg",
                        encoderRight.get_position() / 100.0); y += 30;
  }
}

static void DisplayDebugMenu4() {
  DisplayDebugMenu4(ekf, imu, encoderLeft, encoderRight, &encoderLateral);
}

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
