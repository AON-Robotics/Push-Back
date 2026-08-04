#include "aon/auton/lemlib-routes.hpp"

#include "aon/auton/actions.hpp"
#include "aon/auton/lemlib-validation.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/config/robot-config.hpp"
#include "aon/lemlib/chassis.hpp"

#include "pros/motors.h"

namespace aon::routines {
namespace {

class HardwareValidationMotion final
    : public aon::auton::LemLibValidationMotion {
 public:
  void setPose(const aon::auton::LemLibValidationPose& pose) override {
    auto& routine = aon::auton::actions();
    routine.resetCancellation();
    routine.setPose(pose.x, pose.y, pose.heading);
  }

  bool moveToPoint(const char* name, double x, double y, bool forwards,
                   int maxSpeed, int timeoutMs) override {
    return aon::auton::actions()
        .moveToPoint(name, x, y, timeoutMs,
                     {.forwards = forwards,
                      .maxSpeed = static_cast<float>(maxSpeed)})
        .succeeded;
  }

  bool turnToHeading(const char* name, double heading, int maxSpeed,
                     int timeoutMs) override {
    return aon::auton::actions()
        .turnToHeading(
            name, heading, timeoutMs,
            {.direction = lemlib::AngularDirection::AUTO,
             .maxSpeed = maxSpeed})
        .succeeded;
  }

  aon::auton::LemLibValidationPose pose() const override {
    const auto current = aon::lemlib_integration::chassis().getPose();
    return {current.x, current.y, current.theta};
  }

  void stop() noexcept override {
    aon::auton::actions().stop(pros::E_MOTOR_BRAKE_BRAKE);
  }
};

const char* validationName(aon::auton::LemLibValidationTest test) {
  using aon::auton::LemLibValidationTest;
  switch (test) {
    case LemLibValidationTest::Forward:
      return "TEST LemLib Forward 12in";
    case LemLibValidationTest::Reverse:
      return "TEST LemLib Reverse 12in";
    case LemLibValidationTest::Clockwise90:
      return "TEST LemLib CW 90deg";
    case LemLibValidationTest::Counterclockwise90:
      return "TEST LemLib CCW 90deg";
  }
  return "TEST LemLib Invalid";
}

int runConfiguredValidation(aon::auton::LemLibValidationTest test) {
  const char* const name = validationName(test);
  if (aon::config::activeRobotConfig().identity ==
      aon::config::RobotIdentity::Big) {
    aon::auton::logStep(name, "unsupported big robot");
    aon::auton::actions().stop(pros::E_MOTOR_BRAKE_BRAKE);
    return 0;
  }

  aon::auton::logStep(name, "start");
  HardwareValidationMotion motion;
  const auto result = aon::auton::runLemLibValidation(test, motion);
  aon::auton::logStep(name, result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
}

}  // namespace

int RunLemLibForwardValidation() {
  return runConfiguredValidation(aon::auton::LemLibValidationTest::Forward);
}

int RunLemLibReverseValidation() {
  return runConfiguredValidation(aon::auton::LemLibValidationTest::Reverse);
}

int RunLemLibClockwiseTurnValidation() {
  return runConfiguredValidation(
      aon::auton::LemLibValidationTest::Clockwise90);
}

int RunLemLibCounterclockwiseTurnValidation() {
  return runConfiguredValidation(
      aon::auton::LemLibValidationTest::Counterclockwise90);
}

int RunLemLibCombinedValidation() {
  if (aon::config::activeRobotConfig().identity ==
      aon::config::RobotIdentity::Big) {
    aon::auton::logStep("TEST LemLib Combined", "unsupported big robot");
    aon::auton::actions().stop(pros::E_MOTOR_BRAKE_BRAKE);
    return 0;
  }
  HardwareValidationMotion motion;
  return aon::auton::runAllLemLibValidations(motion) ? 1 : 0;
}

}  // namespace aon::routines
