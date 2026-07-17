#include "aon/auton/routines.hpp"
#include "aon/globals.hpp"
#include "aon/auton/actions.hpp"
#include "aon/auton/mechanism-actions.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"

#include "pros/rtos.hpp"

ASSET(path_jerryio_txt);

namespace aon::routines {
namespace {

bool requiredMotion(const aon::auton::MotionResult& result) {
  if (result.succeeded) return true;
  aon::auton::actions().stop();
  aon::auton::mechanisms::stopAll();
  return false;
}

}  // namespace

int RunLemLibTurnCharacterization(const char* name, double heading) {
  auto& routine = aon::auton::actions();
  routine.setPose(0, 0, 0);
  const auto result = routine.turnToHeading(
      name, heading, 2500,
      {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 50});
  routine.stop();
  return result.succeeded ? 1 : 0;
}

int RunLemLibForwardValidation() {
  auto& routine = aon::auton::actions();

#if USING_BIG_ROBOT
  aon::auton::logStep("LemLib Forward 12in", "unsupported big robot");
  routine.stop();
  return 0;
#else
  aon::auton::logStep("LemLib Forward 12in", "start");
  routine.setPose(0.0, 0.0, 0.0);
  const auto result = routine.moveToPoint(
      "LemLib Forward 12in", 0.0, 12.0, 3000,
      {.forwards = true, .maxSpeed = 35});
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  aon::auton::logStep("LemLib Forward 12in",
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
}

int RunJerryIoPathTest(const char* name) {
  auto& routine = aon::auton::actions();
  // The asset is rotated so heading zero and the initial path tangent both
  // point along +Y. The third asset column is speed, not robot heading.
  routine.setPose(0, 0, 0);
  const auto result =
      routine.followPath(name, path_jerryio_txt, 10, 14000, true);
  routine.stop();
  return result.succeeded ? 1 : 0;
}

int RunStagedLoaderScoreExperiment() {
  auto& routine = aon::auton::actions();

#if USING_BIG_ROBOT
  aon::auton::logStep("Staged Loader", "unsupported big robot");
  routine.stop();
  return 0;
#endif

  aon::auton::logStep("Staged Loader", "start");
  routine.setPose(0, 0, 0);
  aon::auton::mechanisms::finishLoaderCollection();

  // Heading zero points along +Y. Chain most of the loader approach, then
  // slow down for the heading-sensitive final alignment.
  aon::auton::logStep("Staged Loader", "fast loader approach");
  if (!requiredMotion(routine.moveToPoint(
          "staged loader: fast approach", 0.0, 24.0, 1800,
          {.forwards = true,
           .maxSpeed = 70,
           .minSpeed = 35,
           .earlyExitRange = 8})))
    return 0;
  if (!requiredMotion(routine.moveToPoint(
          "staged loader: alignment point", 0.0, 31.0, 1400,
          {.forwards = true, .maxSpeed = 45})))
    return 0;
  if (!requiredMotion(routine.turnToHeading(
          "staged loader: face loader", 86.0, 1200,
          {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 45})))
    return 0;

  aon::auton::logStep("Staged Loader", "collect blocks");
  aon::auton::mechanisms::prepareLoaderCart();
  pros::delay(200);
  aon::auton::mechanisms::beginLoaderCollection();
  if (!requiredMotion(routine.moveToPoint(
          "staged loader: slow contact", 4.0, 31.0, 1000,
          {.forwards = true, .maxSpeed = 30})))
    return 0;
  if (!requiredMotion(
          routine.arcadeFor("staged loader: hold against loader", 25, 0, 250)))
    return 0;
  pros::delay(4000);
  aon::auton::mechanisms::finishLoaderCollection();

  aon::auton::logStep("Staged Loader", "back out");
  if (!requiredMotion(routine.moveToPoint(
          "staged loader: fast retreat", -5.0, 31.0, 1300,
          {.forwards = false,
           .maxSpeed = 55,
           .minSpeed = 30,
           .earlyExitRange = 4})))
    return 0;
  if (!requiredMotion(routine.moveToPoint(
          "staged loader: retreat finish", -9.0, 31.0, 1000,
          {.forwards = false, .maxSpeed = 35})))
    return 0;

  aon::auton::logStep("Staged Loader", "face long goal");
  aon::auton::mechanisms::resetLoaderCart();
  if (!requiredMotion(routine.turnToHeading(
          "staged loader: face long goal", 171.0, 1600,
          {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 45})))
    return 0;

  aon::auton::logStep("Staged Loader", "precise score approach");
#if !USING_BIG_ROBOT
  aon::auton::mechanisms::prepareTopScorer();
#endif
  if (!requiredMotion(routine.moveToPose(
          "staged loader: long goal", -8.0, 25.0, 171.0, 2200,
          {.forwards = true,
           .horizontalDrift = 8,
           .lead = 0.1,
           .maxSpeed = 35})))
    return 0;
  aon::auton::mechanisms::scoreTopBlocks(3000);

  aon::auton::logStep("Staged Loader", "finish");
  routine.stop();
  aon::auton::mechanisms::stopAll();
  return 1;
}

}  // namespace aon::routines
