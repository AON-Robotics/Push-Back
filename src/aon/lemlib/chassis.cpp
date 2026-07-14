#include "aon/lemlib/chassis.hpp"

#include "aon/auton/actions.hpp"
#include "aon/config/robot-config.hpp"
#include "aon/constants.hpp"
#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

#include <cstdio>
#include <cmath>
#include <memory>
#include <vector>

namespace aon::lemlib_integration {
namespace {

struct TrackingSample {
  double left;
  double right;
  double back;
  double rotation;
};

struct CalibrationResult {
  double angle;
  double leftOffset;
  double rightOffset;
  double backOffset;
};

double trackingInches(const pros::Rotation& sensor) {
  const auto& config = aon::config::activeRobotConfig().lemlib;
  const double degrees = sensor.get_position() / 100.0;
  return degrees / 360.0 * M_PI * config.trackingWheelDiameter;
}

TrackingSample trackingSample() {
  const auto& ports = aon::config::activeRobotConfig().lemlib.trackingPorts;
  static pros::Rotation left(ports.left);
  static pros::Rotation right(ports.right);
  static pros::Rotation back(ports.back);
  static pros::Imu imu(ports.imu);
  return {trackingInches(left), trackingInches(right), trackingInches(back),
          imu.get_rotation()};
}

void runFullTurn(lemlib::Chassis& testChassis,
                 lemlib::AngularDirection direction) {
  const float clockwiseHeadings[] = {90.0, 180.0, 270.0, 0.0};
  const float counterclockwiseHeadings[] = {270.0, 180.0, 90.0, 0.0};
  const float* headings =
      direction == lemlib::AngularDirection::CW_CLOCKWISE
          ? clockwiseHeadings
          : counterclockwiseHeadings;

  for (int i = 0; i < 4; ++i) {
    testChassis.turnToHeading(
        headings[i], 2500, {.direction = direction, .maxSpeed = 50}, false);
    pros::delay(150);
  }
}

CalibrationResult reportCalibration(const char* label,
                                    const TrackingSample& start,
                                    const TrackingSample& end) {
  const double radians = (end.rotation - start.rotation) * M_PI / 180.0;
  const double leftDelta = end.left - start.left;
  const double rightDelta = end.right - start.right;
  const double backDelta = end.back - start.back;
  const double leftOffset = radians == 0.0 ? 0.0 : leftDelta / radians;
  const double rightOffset = radians == 0.0 ? 0.0 : rightDelta / radians;
  const double backOffset = radians == 0.0 ? 0.0 : backDelta / radians;

  std::printf(
      "LEMLIB_CAL %s angle=%.3f left_delta=%.3f right_delta=%.3f "
      "back_delta=%.3f left_offset=%.3f right_offset=%.3f "
      "back_offset=%.3f\n",
      label, end.rotation - start.rotation, leftDelta, rightDelta, backDelta,
      leftOffset, rightOffset, backOffset);
  return {end.rotation - start.rotation, leftOffset, rightOffset, backOffset};
}

}  // namespace

lemlib::Chassis& chassis() {
  const auto& config = aon::config::activeRobotConfig().lemlib;
  static const std::vector<std::int8_t> leftPorts(config.motors.left.begin(),
                                                  config.motors.left.end());
  static const std::vector<std::int8_t> rightPorts(config.motors.right.begin(),
                                                   config.motors.right.end());
  static pros::MotorGroup leftMotors(leftPorts);
  static pros::MotorGroup rightMotors(rightPorts);

  static pros::Rotation leftEncoder(config.trackingPorts.left);
  static pros::Rotation rightEncoder(config.trackingPorts.right);
  static pros::Rotation backEncoder(config.trackingPorts.back);
  static const bool encodersConfigured = [&config] {
    leftEncoder.set_reversed(config.trackingPorts.leftReversed);
    rightEncoder.set_reversed(config.trackingPorts.rightReversed);
    backEncoder.set_reversed(config.trackingPorts.backReversed);
    return true;
  }();
  (void)encodersConfigured;
  static pros::Imu imu(config.trackingPorts.imu);

  static lemlib::TrackingWheel leftTrackingWheel(
      &leftEncoder, config.trackingWheelDiameter, config.leftTrackingOffset);
  static lemlib::TrackingWheel rightTrackingWheel(
      &rightEncoder, config.trackingWheelDiameter, config.rightTrackingOffset);
  static lemlib::TrackingWheel backTrackingWheel(
      &backEncoder, config.trackingWheelDiameter, config.backTrackingOffset);

  static lemlib::Drivetrain drivetrain{
      &leftMotors,
      &rightMotors,
      config.trackWidth,
      config.driveWheelDiameter,
      config.drivetrainRpm,
      config.horizontalDrift,
  };

  // LemLib commands the motor groups directly, so these slew limits protect
  // autonomous paths from hard acceleration and braking spikes.
  // LemLib's documented baseline enables controlled lateral movement. These
  // gains are only a starting point; final values require drivetrain testing.
  static lemlib::ControllerSettings lateralController{
      10.0, 0.0, 3.0, 3.0, 1.0, 100.0, 3.0, 500.0,
      config.lateralSlew,
  };
  static lemlib::ControllerSettings angularController{
      2.0, 0.0, 10.0, 0.0, 1.0, 100.0, 3.0, 500.0, config.angularSlew,
  };

  static lemlib::OdomSensors sensors{
      &leftTrackingWheel,
      &rightTrackingWheel,
      &backTrackingWheel,
      nullptr,
      &imu,
  };

  static lemlib::Chassis configuredChassis(
      drivetrain, lateralController, angularController, sensors);
  return configuredChassis;
}

void initializeChassis() {
  lemlib::Chassis& configuredChassis = chassis();
  configuredChassis.calibrate();
  configuredChassis.setPose(0.0, 0.0, 0.0);
}

void startSensorTest() {
  initializeChassis();
  lemlib::Chassis& sensorChassis = chassis();

  pros::lcd::initialize();
  pros::lcd::set_text(0, "LemLib sensor test");
  pros::lcd::set_text(4, "Forward: +Y");
  pros::lcd::set_text(5, "Right: +X");
  pros::lcd::set_text(6, "Clockwise: +heading");

  static std::unique_ptr<pros::Task> displayTask;
  displayTask = std::make_unique<pros::Task>([] {
    lemlib::Chassis& sensorChassis = chassis();
    std::uint32_t lastTerminalUpdate = 0;
    while (true) {
      const lemlib::Pose pose = sensorChassis.getPose();
      pros::lcd::print(1, "X: %7.2f in", pose.x);
      pros::lcd::print(2, "Y: %7.2f in", pose.y);
      pros::lcd::print(3, "H: %7.2f deg", pose.theta);

      if (pros::millis() - lastTerminalUpdate >= 250) {
        std::printf("LEMLIB_POSE x=%.3f y=%.3f heading=%.3f\n",
                    pose.x, pose.y, pose.theta);
        lastTerminalUpdate = pros::millis();
      }
      pros::delay(50);
    }
  });
}

void runTurnTest(double targetHeading) {
  lemlib::Chassis& testChassis = chassis();
  testChassis.setPose(0.0, 0.0, 0.0);
  testChassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(250);

  pros::lcd::set_text(4, "Auton turn running...");
  auton::actions().turnToHeading(
      "LemLib turn test", targetHeading, 2500,
      {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 50});

  const lemlib::Pose finalPose = testChassis.getPose();
  const double error = std::remainder(targetHeading - finalPose.theta, 360.0);
  pros::lcd::print(4, "Target: %7.2f deg", targetHeading);
  pros::lcd::print(5, "Final:  %7.2f deg", finalPose.theta);
  pros::lcd::print(6, "Error:  %+7.2f deg", error);
  std::printf("LEMLIB_TURN target=%.3f final=%.3f error=%.3f\n",
              targetHeading, finalPose.theta, error);
}

void runTrackingCalibrationTest() {
  lemlib::Chassis& testChassis = chassis();
  testChassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  testChassis.setPose(0.0, 0.0, 0.0);
  pros::delay(250);

  pros::lcd::set_text(4, "Cal: clockwise 360");
  const TrackingSample clockwiseStart = trackingSample();
  runFullTurn(testChassis, lemlib::AngularDirection::CW_CLOCKWISE);
  const TrackingSample clockwiseEnd = trackingSample();
  const CalibrationResult clockwise =
      reportCalibration("CW", clockwiseStart, clockwiseEnd);

  pros::delay(1500);
  pros::lcd::set_text(5, "Cal: counter-CW 360");
  const TrackingSample counterStart = trackingSample();
  runFullTurn(testChassis, lemlib::AngularDirection::CCW_COUNTERCLOCKWISE);
  const TrackingSample counterEnd = trackingSample();
  const CalibrationResult counter =
      reportCalibration("CCW", counterStart, counterEnd);
  const double averageLeft = (clockwise.leftOffset + counter.leftOffset) / 2.0;
  const double averageRight =
      (clockwise.rightOffset + counter.rightOffset) / 2.0;
  const double averageBack = (clockwise.backOffset + counter.backOffset) / 2.0;

  pros::lcd::print(4, "CW L:%+.2f R:%+.2f B:%+.2f", clockwise.leftOffset,
                   clockwise.rightOffset, clockwise.backOffset);
  pros::lcd::print(5, "CC L:%+.2f R:%+.2f B:%+.2f", counter.leftOffset,
                   counter.rightOffset, counter.backOffset);
  pros::lcd::print(6, "AV L:%+.2f R:%+.2f B:%+.2f", averageLeft,
                   averageRight, averageBack);
  pros::lcd::print(7, "Angles:%+.1f / %+.1f", clockwise.angle,
                   counter.angle);
}

}  // namespace aon::lemlib_integration
