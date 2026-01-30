#include "main.hpp"
#include "aon/handler.hpp"
#include "aon/EKF/EKFDebug.hpp"

// Added for path follower test
#include "aon/Astar/astar.hpp"
#include "aon/controls/path_following/path_follower.hpp"
#include "aon/controls/pid/pid.hpp"

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
  // --- Path follower test setup (run once) ---
  aon::PID distPID(0.05, 0.0, 0.001, 0.01);  // starting gains (tune later)
  aon::PID turnPID(0.02, 0.0, 0.001, 0.01);

  PathFollower follower(distPID, turnPID);

  // Fake path: from (-24,0) to (24,0) in plane inches (center-origin)
  std::vector<Astar::Waypoint> wps;
  wps.push_back({Astar::from_xy(-24, 0), 0.0});
  wps.push_back({Astar::from_xy( 24, 0), 0.0});
  follower.set_path(wps);

  while (true) {
    // aon::poll();
    sensorFeeder.step(ekf);

    // --- Read pose from EKF ---
    EKF::State s = ekf.getState();

    Pose pose;
    pose.x = s.x_in;                          // inches
    pose.y = s.y_in;                          // inches
    pose.heading_deg = s.theta_rad * 180.0 / M_PI;

    // --- Run follower ---
    DriveCmd cmd = follower.update(pose);

    static int counter = 0;
    if (++counter % 50 == 0) { // 50 * 10ms = 500ms
      printf("x=%.1f y=%.1f hd=%.1f | L=%.2f R=%.2f | i=%d/%d\n",
        pose.x, pose.y, pose.heading_deg,
        cmd.left, cmd.right,
        follower.finished() ? -1 : 0,   // optional placeholder if you don't expose i
        (int)wps.size());
    }


    // --- Print results ---
    pros::lcd::print(0, "L: %.2f  R: %.2f", cmd.left, cmd.right);
    pros::lcd::print(1, "x:%.1f y:%.1f hd:%.1f", pose.x, pose.y, pose.heading_deg);

    DisplayDebugMenu4();
    pros::delay(10);
  }
}


