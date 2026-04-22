#include "../../include/aon/tank-drive/tank-drive.hpp"

#include "aon/constants.hpp"
#include "aon/math/misc/misc.hpp"
#include "aon/tools/vector.hpp"
#include "api.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <initializer_list>

namespace aon {

// ==============================
// TankDrive
// ==============================

TankDrive::TankDrive(const std::initializer_list<okapi::Motor> &leftPorts,
                     const std::initializer_list<okapi::Motor> &rightPorts)
    : leftMotors_(leftPorts),
      rightMotors_(rightPorts) {
  syncLegacyProfiles();
}

void TankDrive::bindSCurveProfile2Config(const SCurveProfile2Config &cfg) {
  mp_cfg_ = &cfg;
  syncLegacyProfiles();
}

void TankDrive::setKinematics(const TankDriveKinematicsConfig &cfg) {
  kin_ = cfg;
  syncLegacyProfiles();
}

double TankDrive::clamp(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

double TankDrive::maxMotorRpmLimit() const {
  return (mp_cfg_ != nullptr) ? std::abs(mp_cfg_->max_velocity_rpm)
                              : std::abs(MAX_RPM);
}

double TankDrive::ipsToMotorRpm(double v_ips) const {
  const double D = kin_.wheel_diam_in;
  const double pi = std::acos(-1.0);
  const double circ = pi * D;
  if (circ <= 0.0) return 0.0;

  const double wheel_rpm = (v_ips / circ) * 60.0;

  // wheel_rpm = motor_rpm * motor_to_drive_ratio
  const double ratio = (mp_cfg_ != nullptr) ? mp_cfg_->motor_to_drive_ratio : 1.0;
  if (std::abs(ratio) <= 1e-9) return 0.0;

  return wheel_rpm / ratio;
}

double TankDrive::motorRpmToIps(double rpm) const {
  const double pi = std::acos(-1.0);
  const double circ = pi * kin_.wheel_diam_in;
  if (circ <= 0.0) return 0.0;

  const double ratio = (mp_cfg_ != nullptr) ? mp_cfg_->motor_to_drive_ratio : 1.0;
  if (std::abs(ratio) <= 1e-9) return 0.0;

  const double wheel_rpm = rpm * ratio;
  return wheel_rpm * (circ / 60.0);
}

void TankDrive::setWheelMotorRpm(double left_rpm, double right_rpm) {
  setWheelVelIps(motorRpmToIps(left_rpm), motorRpmToIps(right_rpm));
}

void TankDrive::setWheelVelIps(double vLeft_ips, double vRight_ips) {
  double rpmL = ipsToMotorRpm(vLeft_ips);
  double rpmR = ipsToMotorRpm(vRight_ips);

  const double maxRpm = maxMotorRpmLimit();

  const double maxAbs = std::max(std::abs(rpmL), std::abs(rpmR));
  if (maxAbs > maxRpm && maxAbs > 1e-9) {
    const double scale = maxRpm / maxAbs;
    rpmL *= scale;
    rpmR *= scale;
  }

  rpmL = clamp(rpmL, -maxRpm, +maxRpm);
  rpmR = clamp(rpmR, -maxRpm, +maxRpm);

  leftMotors_.moveVelocity(static_cast<std::int16_t>(std::lround(rpmL)));
  rightMotors_.moveVelocity(static_cast<std::int16_t>(std::lround(rpmR)));
}

void TankDrive::setChassisVelocity(const aon::hpp::Command &cmd) {
  const double halfW = 0.5 * kin_.track_width_in;
  const double vL_ips = cmd.vx_ips - (cmd.omega_rps * halfW);
  const double vR_ips = cmd.vx_ips + (cmd.omega_rps * halfW);
  setWheelVelIps(vL_ips, vR_ips);
}

void TankDrive::stop() {
  leftMotors_.moveVelocity(0);
  rightMotors_.moveVelocity(0);
}

void TankDrive::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode) {
  leftMotors_.setBrakeMode(brakeMode);
  rightMotors_.setBrakeMode(brakeMode);
}

void TankDrive::setGearset(okapi::AbstractMotor::gearset gearset) {
  leftMotors_.setGearing(gearset);
  rightMotors_.setGearing(gearset);
}

void TankDrive::setEncoderUnits(okapi::AbstractMotor::encoderUnits units) {
  leftMotors_.setEncoderUnits(units);
  rightMotors_.setEncoderUnits(units);
  leftMotors_.tarePosition();
  rightMotors_.tarePosition();
}

void TankDrive::bindLegacyOdometry(Odometry *odom) {
  legacy_odom_ = odom;
}

void TankDrive::syncLegacyProfiles() {
  SCurveProfile2Config cfg;
  if (mp_cfg_ != nullptr) {
    cfg = *mp_cfg_;
  } else {
    cfg.max_velocity_rpm = std::abs(MAX_RPM);
    cfg.max_accel_rpmps = std::abs(MAX_ACCEL);
    cfg.max_decel_rpmps = std::abs(MAX_DECEL);
    cfg.jerk_rpmps2 = std::abs(MAX_ACCEL);
    cfg.drive_wheel_diameter_in = kin_.wheel_diam_in;
    cfg.motor_to_drive_ratio = MOTOR_TO_DRIVE_RATIO;
  }

  cfg.drive_wheel_diameter_in = kin_.wheel_diam_in;

  legacy_linear_cfg_ = cfg;
  legacy_turn_cfg_ = cfg;
  legacy_turn_cfg_.max_accel_rpmps *= 3.0;
  legacy_turn_cfg_.max_decel_rpmps *= 0.8;
  legacy_turn_cfg_.jerk_rpmps2 *= 3.0;

  legacy_linear_profile_.setConfig(legacy_linear_cfg_);
  legacy_turn_profile_.setConfig(legacy_turn_cfg_);
}

void TankDrive::motors(double rpm, int delay_ms) {
  setWheelMotorRpm(rpm, rpm);
  if (delay_ms > 0) {
    pros::delay(delay_ms);
    stop();
  }
}

void TankDrive::rotate(double rpm) {
  setWheelMotorRpm(rpm, -rpm);
}

void TankDrive::driveWhileTurning(double forward_rpm, double turn_rpm) {
  setWheelMotorRpm(forward_rpm + turn_rpm, forward_rpm - turn_rpm);
}

void TankDrive::drive(double /*leftX*/, double leftY, double rightX, double /*rightY*/) {
  const double vertical = leftY * MAX_RPM * 0.6;
  const double turn_cmd = rightX * MAX_RPM * 0.4;
  driveWhileTurning(vertical, turn_cmd);
}

void TankDrive::setSlewRate(double slew_rpmps) {
  legacy_slew_rpmps_ = std::max(0.0, slew_rpmps);
}

double TankDrive::getRPM() {
  return (leftMotors_.getActualVelocity() + rightMotors_.getActualVelocity()) * 0.5;
}

void TankDrive::setMaxVelocity(double rpm) {
  syncLegacyProfiles();
  legacy_linear_cfg_.max_velocity_rpm = std::abs(rpm);
  legacy_turn_cfg_.max_velocity_rpm = std::abs(rpm);
  legacy_linear_profile_.setConfig(legacy_linear_cfg_);
  legacy_turn_profile_.setConfig(legacy_turn_cfg_);
}

double TankDrive::updateProfile(double remainingDist_in, double dt_s) {
  syncLegacyProfiles();
  const double ips = legacy_linear_profile_.update(std::max(0.0, remainingDist_in), dt_s);
  return ipsToMotorRpm(ips);
}

void TankDrive::driveProfiled(double dist_in, bool settle) {
  if (legacy_odom_ == nullptr || dist_in == 0.0) return;

  syncLegacyProfiles();
  legacy_linear_profile_.reset();

  const double sign = (dist_in >= 0.0) ? 1.0 : -1.0;
  const double target_dist = std::abs(dist_in);
  const uint32_t timeout_ms =
      pros::millis() + static_cast<uint32_t>((target_dist / 3.0) * 1000.0);

  const Vector start_pos = legacy_odom_->getPosition();
  double last_time_s = static_cast<double>(pros::micros()) / 1e6;

  while (pros::millis() < timeout_ms) {
    const double traveled = (legacy_odom_->getPosition() - start_pos).GetMagnitude();
    if (traveled >= target_dist) break;

    const double remaining = target_dist - traveled;
    const double now_s = static_cast<double>(pros::micros()) / 1e6;
    const double dt_s = std::max(0.001, now_s - last_time_s);
    last_time_s = now_s;

    const double cmd_ips = legacy_linear_profile_.update(remaining, dt_s);
    setWheelVelIps(sign * cmd_ips, sign * cmd_ips);
    pros::delay(20);
  }

  if (settle) stop();
}

void TankDrive::turnProfiled(double angle_deg, bool settle) {
  if (legacy_odom_ == nullptr || angle_deg == 0.0) return;

  syncLegacyProfiles();
  legacy_turn_profile_.reset();

  const double sign = (angle_deg >= 0.0) ? 1.0 : -1.0;
  const double target_angle = std::abs(angle_deg);
  const uint32_t timeout_ms =
      pros::millis() + static_cast<uint32_t>(std::sqrt(target_angle / 2.0) * 1000.0);

  const double circumference_in = kin_.track_width_in * std::acos(-1.0);
  const double start_angle = legacy_odom_->gyroscope.get_rotation();
  double last_time_s = static_cast<double>(pros::micros()) / 1e6;

  while (pros::millis() < timeout_ms) {
    const double traveled_angle =
        std::abs(legacy_odom_->gyroscope.get_rotation() - start_angle);
    if (traveled_angle >= target_angle) break;

    const double remaining_angle = target_angle - traveled_angle;
    const double remaining_arc = circumference_in * (remaining_angle / 360.0);

    const double now_s = static_cast<double>(pros::micros()) / 1e6;
    const double dt_s = std::max(0.001, now_s - last_time_s);
    last_time_s = now_s;

    const double cmd_ips = legacy_turn_profile_.update(remaining_arc, dt_s);
    setWheelVelIps(sign * cmd_ips, -sign * cmd_ips);
    pros::delay(20);
  }

  if (settle) stop();
}

void TankDrive::move(double dist_in, bool settle) {
  driveProfiled(dist_in, settle);
}

void TankDrive::turn(double angle_deg, bool settle) {
  turnProfiled(angle_deg, settle);
}

void TankDrive::drivePID(PID pid, double dist_in, const double &max_revs) {
  if (legacy_odom_ == nullptr || dist_in == 0.0) return;

  const double sign = (dist_in >= 0.0) ? 1.0 : -1.0;
  const double target_dist = std::abs(dist_in);
  pid.Reset();
  const Vector initial_pos = legacy_odom_->getPosition();

  while ((legacy_odom_->getPosition() - initial_pos).GetMagnitude() < target_dist) {
    const double current = (legacy_odom_->getPosition() - initial_pos).GetMagnitude();
    const double output = pid.Output(target_dist, current);
    const double cmd_rpm =
        sign * std::clamp(output * MAX_RPM, -max_revs, max_revs);
    motors(cmd_rpm);
    pros::delay(10);
  }

  stop();
}

void TankDrive::turnPID(PID pid, double angle_deg, const double &max_revs) {
  if (legacy_odom_ == nullptr || angle_deg == 0.0) return;

  double angle = std::abs(angle_deg);
  const double sign = (angle_deg >= 0.0) ? 1.0 : -1.0;
  pid.Reset();

  legacy_odom_->gyroscope.tare();
  const double start_angle = legacy_odom_->getDegrees();

  if (sign < 0.0) angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET;
  if (sign > 0.0) angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET;

  const double start_time = static_cast<double>(pros::micros()) / 1e6;
  const double timeout_s = 3.0 * math::getTimetoTurnDeg(std::abs(angle_deg));

  while (((static_cast<double>(pros::micros()) / 1e6) - start_time) < timeout_s) {
    const double traveled_angle = std::abs(legacy_odom_->getDegrees() - start_angle);
    const double output = pid.Output(angle, traveled_angle);
    const double cmd_rpm =
        sign * std::clamp(output * MAX_RPM, -max_revs, max_revs);
    rotate(cmd_rpm);
    pros::delay(10);
  }

  stop();
}

void TankDrive::driveInArc(double radius_in, const double &midSpeed_rpm) {
  if (radius_in == 0.0) return;

  const bool clockwise = radius_in > 0.0;
  const double radius = std::abs(radius_in);

  const double outer_ratio = (radius + (DRIVE_WIDTH / 2.0)) / radius;
  const double inner_ratio = (radius - (DRIVE_WIDTH / 2.0)) / radius;
  const double outer_speed = midSpeed_rpm * outer_ratio;
  const double inner_speed = midSpeed_rpm * inner_ratio;

  if (clockwise) {
    setWheelMotorRpm(outer_speed, inner_speed);
  } else {
    setWheelMotorRpm(inner_speed, outer_speed);
  }
}

void TankDrive::driveAngleOfArc(const double &radius_in, const double &angle_deg, bool settle) {
  if (legacy_odom_ == nullptr || angle_deg == 0.0) return;

  if (radius_in == 0.0) {
    turn(angle_deg, settle);
    return;
  }

  syncLegacyProfiles();
  legacy_linear_profile_.reset();

  const double sign = (angle_deg >= 0.0) ? 1.0 : -1.0;
  const double distance =
      std::abs((2.0 * radius_in * std::acos(-1.0)) * (angle_deg / 360.0));

  const double right_start = legacy_odom_->encoderRight.get_position();
  const double left_start = legacy_odom_->encoderLeft.get_position();
  double last_time_s = static_cast<double>(pros::micros()) / 1e6;

  while (true) {
    const double right_enc_dist =
        (std::abs(legacy_odom_->encoderRight.get_position() - right_start) / 100.0) *
        std::acos(-1.0) * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;

    const double left_enc_dist =
        (std::abs(legacy_odom_->encoderLeft.get_position() - left_start) / 100.0) *
        std::acos(-1.0) * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;

    const double traveled = 0.5 * (right_enc_dist + left_enc_dist);
    if (traveled >= distance) break;

    const double remaining = distance - traveled;
    const double now_s = static_cast<double>(pros::micros()) / 1e6;
    const double dt_s = std::max(0.001, now_s - last_time_s);
    last_time_s = now_s;

    const double mid_speed_ips = legacy_linear_profile_.update(remaining, dt_s);
    driveInArc(radius_in, sign * ipsToMotorRpm(mid_speed_ips));
    pros::delay(20);
  }

  if (settle) stop();
}

void TankDrive::driveInArcTo(const double &x_m, const double &y_m) {
  if (legacy_odom_ == nullptr) return;

  Vector position = legacy_odom_->getPosition();
  position.SetPosition(math::inchesToMeters(position.GetX()),
                       math::inchesToMeters(position.GetY()));

  double heading = legacy_odom_->getDegrees();
  const Vector target = Vector().SetPosition(x_m, y_m);

  heading = (90.0 - heading);
  if (heading < 0.0) heading += 360.0;
  heading *= std::acos(-1.0) / 180.0;

  const bool tan_defined =
      std::fmod(heading - (std::acos(-1.0) * 0.5), std::acos(-1.0)) != 0.0;

  double m_t = tan_defined ? std::tan(heading) : DBL_MAX;
  double m_s =
      (position.GetX() != x_m) ? (position.GetY() - y_m) / (position.GetX() - x_m) : DBL_MAX;

  m_t = (m_t == 0.0) ? DBL_MIN : m_t;
  m_s = (m_s == 0.0) ? DBL_MIN : m_s;

  Vector midpoint =
      Vector().SetPosition((position.GetX() + x_m) / 2.0,
                           (position.GetY() + y_m) / 2.0);

  const double centerX =
      (midpoint.GetY() - position.GetY() - (position.GetX() / m_t) +
       (midpoint.GetX() / m_s)) /
      ((-1.0 / m_t) + (1.0 / m_s));

  const double centerY =
      ((-1.0 / m_t) * (centerX - position.GetX())) + position.GetY();

  Vector center = Vector().SetPosition(centerX, centerY);

  double radius =
      std::hypot(position.GetX() - center.GetX(), position.GetY() - center.GetY());

  double angle = math::getAngleOfArc(position, target, center);

  const double step = 0.001;
  const Vector projection =
      Vector().SetPosition(position.GetX() + (step * std::cos(heading)),
                           position.GetY() + (step * std::sin(heading)));

  const double projection_angle = math::getAngleInCircle(projection, center);
  const double position_angle = math::getAngleInCircle(position, center);
  const double target_angle = math::getAngleInCircle(target, center);

  const bool clockwise =
      (target_angle < projection_angle && projection_angle < position_angle) ||
      (projection_angle < position_angle && position_angle < target_angle) ||
      (position_angle < target_angle && target_angle < projection_angle);

  if (!clockwise) radius *= -1.0;

  const bool long_way =
      (math::getAngleOfArc(projection, target, center) > angle) ||
      (position_angle < target_angle && target_angle < projection_angle);

  if (long_way) angle = 360.0 - angle;

  driveAngleOfArc(math::metersToInches(radius), angle);
}

void TankDrive::turnTo(const double &x_m, const double &y_m) {
  if (legacy_odom_ == nullptr) return;
  const Vector target = Vector().SetPosition(x_m, y_m);
  const Pose current = legacy_odom_->getPose();
  turn(-math::calculateTurn(target, current));
}

void TankDrive::goTo(const double &x_m, const double &y_m) {
  if (legacy_odom_ == nullptr) return;
  const Vector target = Vector().SetPosition(x_m, y_m);
  const Vector current = legacy_odom_->gpsPosition();
  turn(-math::calculateTurn(target, legacy_odom_->getPose()));
  move(math::findDistance(target, current));
}

void TankDrive::goToPose(const Pose &pose) {
  (void)pose;
}

// ==============================
// TankDriveFeeder
// ==============================

void TankDriveFeeder::applyDriveDefaults(TankDrive &dt, const aon::hpp::Config &hppCfg) const {
  dt.setBrakeMode(cfg_.brake);
  dt.setGearset(cfg_.gear);
  dt.setEncoderUnits(cfg_.enc);
  dt.setKinematics(cfg_.kin);
  dt.bindSCurveProfile2Config(hppCfg.mp_cfg);
}

}  // namespace aon