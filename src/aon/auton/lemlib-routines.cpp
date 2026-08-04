#include "aon/auton/lemlib-routes.hpp"
#include "aon/globals.hpp"
#include "aon/auton/actions.hpp"
#include "aon/auton/figure-eight-validation.hpp"
#include "aon/auton/hybrid-sequence.hpp"
#include "aon/auton/jerryio-path-auton.hpp"
#include "aon/auton/mechanism-actions.hpp"
#include "aon/auton/motion-health.hpp"
#include "aon/auton/red-six-block.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"

#include "pros/rtos.hpp"
#include "pros/misc.hpp"

#include <cstdio>

ASSET(path_jerryio_txt);
ASSET(figure_eight_jerryio_txt);
ASSET(red_six_loader_approach_jerryio_txt);
ASSET(red_six_goal_transfer_jerryio_txt);

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

const char* redSixPhaseName(aon::auton::RedSixPhase phase) {
  using aon::auton::RedSixPhase;
  switch (phase) {
    case RedSixPhase::LoaderPursuit:
      return "loader pursuit";
    case RedSixPhase::LoaderContact:
      return "loader contact";
    case RedSixPhase::CollectSix:
      return "collect six";
    case RedSixPhase::ReverseClearance:
      return "reverse clearance";
    case RedSixPhase::ReverseAlignment:
      return "reverse alignment";
    case RedSixPhase::GoalPursuit:
      return "goal pursuit";
    case RedSixPhase::GoalContact:
      return "goal contact";
    case RedSixPhase::ScoreSix:
      return "score six";
  }
  return "invalid phase";
}

bool redSixMotion(aon::auton::RedSixPhase phase,
                  const aon::auton::MotionResult& result) {
  if (result.succeeded) return true;
  std::printf("AUTON_RED_SIX_FAILURE phase=%s reason=%s\n",
              redSixPhaseName(phase),
              aon::auton::motionFailureName(result.reason));
  return false;
}

int RunLemLibFigureEightValidation() {
  auto& routine = aon::auton::actions();

#if USING_BIG_ROBOT
  aon::auton::logStep(aon::auton::FigureEightValidation::name,
                      "unsupported big robot");
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  return 0;
#else
  using aon::auton::FigureEightValidation;
  aon::auton::logStep(FigureEightValidation::name, "start");
  routine.setPose(FigureEightValidation::startX,
                  FigureEightValidation::startY,
                  FigureEightValidation::startHeading);
  const auto result = routine.followPath(
      FigureEightValidation::name, figure_eight_jerryio_txt,
      FigureEightValidation::lookahead, FigureEightValidation::timeoutMs,
      true);
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  aon::auton::logStep(FigureEightValidation::name,
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
}

int RunJerryIoPathAuton() {
  using aon::auton::JerryIoPathAuton;
  auto& routine = aon::auton::actions();

#if USING_BIG_ROBOT
  aon::auton::logStep(JerryIoPathAuton::name, "unsupported big robot");
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  return 0;
#else
  aon::auton::logStep(JerryIoPathAuton::name, "start");
  routine.setPose(JerryIoPathAuton::startX, JerryIoPathAuton::startY,
                  JerryIoPathAuton::startHeading);
  const auto result = routine.followPath(
      JerryIoPathAuton::name, path_jerryio_txt, JerryIoPathAuton::lookahead,
      JerryIoPathAuton::timeoutMs, true, {},
      aon::auton::OdometryMonitoring::FailClosed);
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  aon::auton::logStep(JerryIoPathAuton::name,
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
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

int RunRedSixBlockHybridAuton(aon::auton::RedSixPhase stopAfter) {
  using aon::auton::RedSixBlock;
  using aon::auton::RedSixCallbacks;
  using aon::auton::RedSixPhase;
  auto& routine = aon::auton::actions();

#if USING_BIG_ROBOT
  (void)stopAfter;
  aon::auton::logStep(RedSixBlock::name, "unsupported big robot");
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  aon::auton::mechanisms::stopAll();
  return 0;
#else
  aon::auton::logStep(RedSixBlock::name, "start");
  // The pose is the tracking center at the repeatable red-side start. Heading
  // zero faces +Y; coordinates are local to this autonomous placement.
  routine.setPose(RedSixBlock::start.x, RedSixBlock::start.y,
                  RedSixBlock::start.heading);
  aon::auton::mechanisms::finishLoaderCollection();
  aon::auton::mechanisms::prepareLoaderCart();

  RedSixCallbacks callbacks;
  callbacks.run = [&](RedSixPhase phase) {
    aon::auton::logStep(RedSixBlock::name, redSixPhaseName(phase));
    switch (phase) {
      case RedSixPhase::LoaderPursuit:
        // Smooth forward travel keeps the loader mechanism leading the route.
        return redSixMotion(
            phase, routine.followPath(
                       "red-six loader pursuit",
                       red_six_loader_approach_jerryio_txt,
                       RedSixBlock::loaderLookahead,
                       RedSixBlock::loaderPathTimeoutMs, true, {},
                       aon::auton::OdometryMonitoring::FailClosed));
      case RedSixPhase::LoaderContact:
        // Approach forward at low speed for repeatable loader alignment.
        return redSixMotion(
            phase, routine.moveToPose(
                       "red-six loader contact", RedSixBlock::loaderContact.x,
                       RedSixBlock::loaderContact.y,
                       RedSixBlock::loaderContact.heading,
                       RedSixBlock::loaderContactTimeoutMs,
                       {.forwards = true, .maxSpeed = 30},
                       aon::auton::OdometryMonitoring::FailClosed));
      case RedSixPhase::CollectSix: {
        aon::auton::mechanisms::beginLoaderCollection();
        const std::uint32_t startedAt = pros::millis();
        // Seat against the loader briefly, then rely on the autonomous HOLD
        // brake mode for the remainder of collection instead of heating the
        // drivetrain against the wall for the full dwell.
        const auto seating = routine.arcadeFor(
            "red-six loader seating hold", 25, 0, 250,
            aon::auton::OdometryMonitoring::FailClosed);
        if (!seating.succeeded) {
          aon::auton::mechanisms::finishLoaderCollection();
          return false;
        }
        bool completed = true;
        while (pros::millis() - startedAt <
               static_cast<std::uint32_t>(RedSixBlock::collectTimeoutMs)) {
          if (routine.isCancellationLatched() ||
              pros::competition::is_disabled()) {
            completed = false;
            break;
          }
          pros::delay(20);
        }
        aon::auton::mechanisms::finishLoaderCollection();
        return completed;
      }
      case RedSixPhase::ReverseClearance:
        aon::auton::mechanisms::resetLoaderCart();
        // Reverse so the robot clears the loader without turning its front
        // mechanism through the wall; early exit carries speed into alignment.
        return redSixMotion(
            phase, routine.moveToPoint(
                       "red-six reverse clearance",
                       RedSixBlock::reverseClearance.x,
                       RedSixBlock::reverseClearance.y,
                       RedSixBlock::reverseClearanceTimeoutMs,
                       {.forwards = false,
                        .maxSpeed = 60,
                        .minSpeed = 25,
                        .earlyExitRange = 2.5},
                       aon::auton::OdometryMonitoring::FailClosed));
      case RedSixPhase::ReverseAlignment:
        // Continue in reverse and finish at the goal path's entry heading.
        return redSixMotion(
            phase, routine.moveToPose(
                       "red-six reverse alignment",
                       RedSixBlock::reverseAlignment.x,
                       RedSixBlock::reverseAlignment.y,
                       RedSixBlock::reverseAlignment.heading,
                       RedSixBlock::reverseAlignmentTimeoutMs,
                       {.forwards = false, .maxSpeed = 40},
                       aon::auton::OdometryMonitoring::FailClosed));
      case RedSixPhase::GoalPursuit:
        // Raise the scorer during safe open travel, before goal contact.
        aon::auton::mechanisms::prepareTopScorer();
        return redSixMotion(
            phase, routine.followPath(
                       "red-six goal pursuit",
                       red_six_goal_transfer_jerryio_txt,
                       RedSixBlock::goalLookahead,
                       RedSixBlock::goalPathTimeoutMs, true, {},
                       aon::auton::OdometryMonitoring::FailClosed));
      case RedSixPhase::GoalContact:
        // Final heading matters here, so use a slow forward pose action.
        return redSixMotion(
            phase, routine.moveToPose(
                       "red-six goal contact", RedSixBlock::goalContact.x,
                       RedSixBlock::goalContact.y,
                       RedSixBlock::goalContact.heading,
                       RedSixBlock::goalContactTimeoutMs,
                       {.forwards = true, .maxSpeed = 30},
                       aon::auton::OdometryMonitoring::FailClosed));
      case RedSixPhase::ScoreSix:
        aon::auton::mechanisms::scoreTopBlocks(
            RedSixBlock::scoreTimeoutMs);
        return !routine.isCancellationLatched() &&
               !pros::competition::is_disabled();
    }
    return false;
  };
  callbacks.stopAll = [&] {
    routine.stop();
    aon::auton::mechanisms::finishLoaderCollection();
    aon::auton::mechanisms::stopAll();
  };

  const auto result = aon::auton::runRedSixSequence(callbacks, stopAfter);
  aon::auton::logStep(RedSixBlock::name,
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
}

int RunRedSixBlockHybridFull() {
  return RunRedSixBlockHybridAuton(aon::auton::RedSixPhase::ScoreSix);
}

}  // namespace aon::routines
