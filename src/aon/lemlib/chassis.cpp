#include "aon/lemlib/chassis.hpp"

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
  const double degrees = sensor.get_position() / 100.0;
  return degrees / 360.0 * M_PI * TRACKING_WHEEL_DIAMETER;
}

TrackingSample trackingSample() {
  static pros::Rotation left(19);
  static pros::Rotation right(18);
  static pros::Rotation back(5);
  static pros::Imu imu(16);
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
  static pros::MotorGroup leftMotors({11, -12, 13, -14});
  static pros::MotorGroup rightMotors({1, -2, 3, -4});

  static pros::Rotation leftEncoder(19);
  static pros::Rotation rightEncoder(18);
  static pros::Rotation backEncoder(5);
  static const bool rightEncoderConfigured = [] {
    rightEncoder.set_reversed(true);
    return true;
  }();
  (void)rightEncoderConfigured;
  static pros::Imu imu(16);

  static lemlib::TrackingWheel leftTrackingWheel(
      &leftEncoder, TRACKING_WHEEL_DIAMETER,
      -DISTANCE_LEFT_TRACKING_WHEEL_CENTER);
  static lemlib::TrackingWheel rightTrackingWheel(
      &rightEncoder, TRACKING_WHEEL_DIAMETER,
      DISTANCE_RIGHT_TRACKING_WHEEL_CENTER);
  static lemlib::TrackingWheel backTrackingWheel(
      &backEncoder, TRACKING_WHEEL_DIAMETER,
      -DISTANCE_BACK_TRACKING_WHEEL_CENTER);

  static lemlib::Drivetrain drivetrain{
      &leftMotors,
      &rightMotors,
      DRIVE_WIDTH,
      DRIVE_WHEEL_DIAMETER,
      MAX_RPM * MOTOR_TO_DRIVE_RATIO,
      8.0,
  };

  // Untuned placeholders. They must be tuned before enabling LemLib movement.
  static lemlib::ControllerSettings lateralController{
      0.0, 0.0, 0.0, 0.0, 1.0, 100.0, 3.0, 500.0, 0.0,
  };
  static lemlib::ControllerSettings angularController{
      2.0, 0.0, 10.0, 0.0, 1.0, 100.0, 3.0, 500.0, 0.0,
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

void startSensorTest() {
  lemlib::Chassis& sensorChassis = chassis();
  sensorChassis.calibrate();
  sensorChassis.setPose(0.0, 0.0, 0.0);

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
  testChassis.turnToHeading(
      targetHeading, 2500,
      {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 50}, false);

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
