#include "aon/auton/actions.hpp"

#include "aon/lemlib/chassis.hpp"

#include "pros/rtos.hpp"

#include <cstdio>

namespace aon::auton {
namespace {

void logStart(const char* operation, const char* name) {
  std::printf("AUTON_START operation=%s name=%s time=%lu\n", operation, name,
              static_cast<unsigned long>(pros::millis()));
}

void logFinish(const char* operation, const char* name,
               const lemlib::Pose& pose) {
  std::printf(
      "AUTON_FINISH operation=%s name=%s x=%.3f y=%.3f heading=%.3f "
      "time=%lu\n",
      operation, name, pose.x, pose.y, pose.theta,
      static_cast<unsigned long>(pros::millis()));
}

}  // namespace

void Actions::setPose(double x, double y, double heading) {
  lemlib_integration::chassis().setPose(x, y, heading);
}

void Actions::moveToPoint(const char* name, double x, double y, int timeout,
                          lemlib::MoveToPointParams params) {
  auto& robotChassis = lemlib_integration::chassis();
  logStart("moveToPoint", name);
  robotChassis.moveToPoint(x, y, timeout, params, false);
  logFinish("moveToPoint", name, robotChassis.getPose());
}

void Actions::moveToPose(const char* name, double x, double y, double heading,
                         int timeout, lemlib::MoveToPoseParams params) {
  auto& robotChassis = lemlib_integration::chassis();
  logStart("moveToPose", name);
  robotChassis.moveToPose(x, y, heading, timeout, params, false);
  logFinish("moveToPose", name, robotChassis.getPose());
}

void Actions::turnToHeading(const char* name, double heading, int timeout,
                            lemlib::TurnToHeadingParams params) {
  auto& robotChassis = lemlib_integration::chassis();
  logStart("turnToHeading", name);
  robotChassis.turnToHeading(heading, timeout, params, false);
  logFinish("turnToHeading", name, robotChassis.getPose());
}

void Actions::cancelMotion() {
  lemlib_integration::chassis().cancelMotion();
}

void Actions::stop() {
  auto& robotChassis = lemlib_integration::chassis();
  robotChassis.cancelAllMotions();
  robotChassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
}

Actions& actions() {
  static Actions instance;
  return instance;
}

}  // namespace aon::auton
