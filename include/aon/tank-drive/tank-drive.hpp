#pragma once

#include "../../okapi/api.hpp"
#include "../constants.hpp"
#include "../controls/holonomic-pure-pursuit/hpp.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/s-curve-profile2.hpp"
#include "../math/pose.hpp"
#include "../odometry/odometry.hpp"

#include <initializer_list>
#include <cstdint>

namespace aon {

// ============================================================
// TankDriveKinematicsConfig
// ============================================================

struct TankDriveKinematicsConfig {
  double wheel_diam_in  = 4.0;
  double track_width_in = 12.0;
};

// ============================================================
// TankDriveConfig
// ============================================================

struct TankDriveConfig {
  TankDriveKinematicsConfig kin{};

  okapi::AbstractMotor::brakeMode brake =
      okapi::AbstractMotor::brakeMode::brake;

  okapi::AbstractMotor::gearset gear =
      okapi::AbstractMotor::gearset::green;

  okapi::AbstractMotor::encoderUnits enc =
      okapi::AbstractMotor::encoderUnits::degrees;
};

// ============================================================
// TankDrive
// ============================================================

class TankDrive {
 public:
  TankDrive(const std::initializer_list<okapi::Motor> &leftPorts = {0},
            const std::initializer_list<okapi::Motor> &rightPorts = {0});
  virtual ~TankDrive() = default;

  // Modern canonical API
  void bindSCurveProfile2Config(const SCurveProfile2Config &cfg);
  void setKinematics(const TankDriveKinematicsConfig &cfg);
  const TankDriveKinematicsConfig &getKinematics() const { return kin_; }

  virtual void setChassisVelocity(const aon::hpp::Command &cmd);
  void setWheelVelIps(double vLeft_ips, double vRight_ips);
  virtual void stop();

  virtual void setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode);
  virtual void setGearset(okapi::AbstractMotor::gearset gearset);
  virtual void setEncoderUnits(okapi::AbstractMotor::encoderUnits units);

  // ============================================================
  // Legacy Compatibility API
  // ============================================================

  void bindLegacyOdometry(Odometry *odom);

  void motors(double rpm = MAX_RPM, int delay_ms = 0);
  void rotate(double rpm);
  void driveWhileTurning(double forward_rpm, double turn_rpm);
  void drive(double leftX, double leftY, double rightX, double rightY);

  void setSlewRate(double slew_rpmps);
  double getRPM();
  void setMaxVelocity(double rpm);
  double updateProfile(double remainingDist_in, double dt_s);

  void driveProfiled(double dist_in = TILE_WIDTH, bool settle = true);
  void turnProfiled(double angle_deg = 90.0, bool settle = true);

  void move(double dist_in = TILE_WIDTH, bool settle = true);
  void turn(double angle_deg = 90.0, bool settle = true);

  void drivePID(PID pid = PID(0.02, 0, 0),
                double dist_in = TILE_WIDTH,
                const double &max_revs = 100.0);

  void turnPID(PID pid = PID(0.002, 0, 0),
               double angle_deg = 90.0,
               const double &max_revs = 50.0);

  void driveInArc(double radius_in, const double &midSpeed_rpm = 200.0);

  void driveAngleOfArc(const double &radius_in = DRIVE_WIDTH,
                       const double &angle_deg = 90.0,
                       bool settle = true);

  void driveInArcTo(const double &x_m, const double &y_m);
  void turnTo(const double &x_m, const double &y_m);
  void goTo(const double &x_m, const double &y_m);

  /// Legacy stub preserved intentionally.
  void goToPose(const Pose &pose);

 private:
  okapi::MotorGroup leftMotors_;
  okapi::MotorGroup rightMotors_;

  TankDriveKinematicsConfig kin_{};
  const SCurveProfile2Config *mp_cfg_ = nullptr;  // non-owning
  Odometry *legacy_odom_ = nullptr;               // non-owning

  SCurveProfile2Config legacy_linear_cfg_{};
  SCurveProfile2Config legacy_turn_cfg_{};
  SCurveProfile2 legacy_linear_profile_{};
  SCurveProfile2 legacy_turn_profile_{};
  double legacy_slew_rpmps_ = 0.0;

 protected:
  static double clamp(double v, double lo, double hi);
  double ipsToMotorRpm(double v_ips) const;
  double motorRpmToIps(double rpm) const;
  double maxMotorRpmLimit() const;
  void setWheelMotorRpm(double left_rpm, double right_rpm);
  void syncLegacyProfiles();
};

// ============================================================
// TankDriveFeeder
// ============================================================

class TankDriveFeeder {
 public:
  explicit TankDriveFeeder(const TankDriveConfig &cfg) : cfg_(cfg) {}

  void applyDriveDefaults(TankDrive &dt, const aon::hpp::Config &hppCfg) const;

 private:
  TankDriveConfig cfg_;
};

}  // namespace aon