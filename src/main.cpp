#include "main.hpp"
#include "aon/handler.hpp"
#include "aon/EKF/EKFDebug.hpp"

void initialize() {
  pros::lcd::initialize();
  // aon::init();
  sensorFeeder.applyEkfDefaults(ekf);
  sensorFeeder.reset();

  //   auto st = gps.get_status();
  // if (std::isfinite(st.x) && std::isfinite(st.y)) {
  //   EKF::State s = ekf.getState();
  //   s.x_in = st.x * 39.37007874015748; // meters -> inches
  //   s.y_in = st.y * 39.37007874015748;
  //   s.theta_rad = EKF::normalizeAngle(imu.get_rotation() * M_PI / 180.0);
  //   ekf.setState(s);
  // }
}

void opcontrol() {
  while (true) {
    // aon::poll();
    sensorFeeder.step(ekf);
    DisplayDebugMenu4();
    pros::delay(10);
  }
}
