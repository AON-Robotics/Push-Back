#include "aon/auton/lemlib-routes.hpp"
#include "aon/globals.hpp"
#include "aon/auton/actions.hpp"
#include "aon/auton/figure-eight-validation.hpp"
#include "aon/auton/hybrid-sequence.hpp"
#include "aon/auton/jerryio-path-auton.hpp"
#include "aon/auton/jerryio-sequence.hpp"
#include "aon/auton/mechanism-actions.hpp"
#include "aon/auton/motion-health.hpp"
#include "aon/auton/red-six-block.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"
#include "aon/config/robot-config.hpp"
#include "aon/core/hardware.hpp"

#include "pros/rtos.hpp"
#include "pros/misc.hpp"

#include <cstdio>

ASSET(path_jerryio_intake_txt);
ASSET(path_jerryio_outtake_txt);
ASSET(path_jerryio_pistons_txt);
ASSET(figure_eight_jerryio_txt);
ASSET(red_six_loader_approach_jerryio_txt);
ASSET(red_six_goal_transfer_jerryio_txt);

namespace aon::routines {
namespace {

bool requiredMotion(const aon::auton::MotionResult& result,
                    aon::core::Hardware& hardware) {
  if (result.succeeded) return true;
  aon::auton::actions().stop();
  aon::auton::mechanisms::stopAll(hardware);
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

const char* jerryIoPhaseName(aon::auton::JerryIoPhase phase) {
  using aon::auton::JerryIoPhase;
  switch (phase) {
    case JerryIoPhase::FollowToIntake:
      return "follow to intake";
    case JerryIoPhase::Intake:
      return "intake";
    case JerryIoPhase::FollowToOuttake:
      return "follow to outtake";
    case JerryIoPhase::Outtake:
      return "outtake";
    case JerryIoPhase::FollowToPistons:
      return "follow to pistons";
    case JerryIoPhase::PulsePistons:
      return "pulse pistons";
  }
  return "invalid phase";
}

bool waitForCheckpoint(aon::auton::Actions& routine,
                       std::uint32_t durationMs) {
  const std::uint32_t startedAt = pros::millis();
  while (pros::millis() - startedAt < durationMs) {
    if (routine.isCancellationLatched() ||
        pros::competition::is_disabled()) {
      return false;
    }
    pros::delay(20);
  }
  return true;
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
  using aon::auton::JerryIoCallbacks;
  using aon::auton::JerryIoPhase;
  auto& routine = aon::auton::actions();
  auto& hardware = aon::core::hardware();

  if (!aon::config::activeRobotConfig()
           .autonomousAuthorizations.allows(
               aon::config::ExperimentalRoute::JerryIoPath)) {
    aon::auton::logStep(JerryIoPathAuton::name,
                        "locked: physical validation required");
    routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
    return 0;
  }

#if USING_BIG_ROBOT
  aon::auton::logStep(JerryIoPathAuton::name, "unsupported big robot");
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  return 0;
#else
  aon::auton::logStep(JerryIoPathAuton::name, "start");
  routine.setPose(JerryIoPathAuton::startX, JerryIoPathAuton::startY,
                  JerryIoPathAuton::startHeading);
  aon::auton::mechanisms::resetLoaderCart(hardware);

  const auto followAndFace = [&](const char* name, const asset& path,
                                 double heading) {
    const auto followed = routine.followPath(
        name, path, JerryIoPathAuton::lookahead,
        JerryIoPathAuton::timeoutMs, true, {},
        aon::auton::OdometryMonitoring::FailClosed);
    if (!followed.succeeded) return false;
    return routine
        .turnToHeading(name, heading, JerryIoPathAuton::headingTimeoutMs,
                       {.direction = lemlib::AngularDirection::AUTO,
                        .maxSpeed = 45})
        .succeeded;
  };

  JerryIoCallbacks callbacks;
  callbacks.run = [&](JerryIoPhase phase) {
    aon::auton::logStep(JerryIoPathAuton::name, jerryIoPhaseName(phase));
    switch (phase) {
      case JerryIoPhase::FollowToIntake:
        return followAndFace("JerryIO: intake checkpoint",
                             path_jerryio_intake_txt,
                             JerryIoPathAuton::intakeHeading);
      case JerryIoPhase::Intake: {
        hardware.intake.move();
        const bool completed = waitForCheckpoint(
            routine, JerryIoPathAuton::actionDurationMs);
        hardware.intake.stop();
        return completed;
      }
      case JerryIoPhase::FollowToOuttake:
        return followAndFace("JerryIO: outtake checkpoint",
                             path_jerryio_outtake_txt,
                             JerryIoPathAuton::outtakeHeading);
      case JerryIoPhase::Outtake: {
        hardware.intake.move(-INTAKE_VELOCITY);
        const bool completed = waitForCheckpoint(
            routine, JerryIoPathAuton::actionDurationMs);
        hardware.intake.stop();
        return completed;
      }
      case JerryIoPhase::FollowToPistons:
        return followAndFace("JerryIO: piston checkpoint",
                             path_jerryio_pistons_txt,
                             JerryIoPathAuton::pistonsHeading);
      case JerryIoPhase::PulsePistons: {
        aon::auton::mechanisms::prepareLoaderCart(hardware);
        const bool completed = waitForCheckpoint(
            routine, JerryIoPathAuton::actionDurationMs);
        aon::auton::mechanisms::resetLoaderCart(hardware);
        return completed;
      }
    }
    return false;
  };
  callbacks.stopAll = [&] {
    routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
    aon::auton::mechanisms::stopAll(hardware);
  };

  const auto result = aon::auton::runJerryIoSequence(callbacks);
  aon::auton::logStep(JerryIoPathAuton::name,
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
}

int RunStagedLoaderScoreExperiment() {
  auto& routine = aon::auton::actions();
  auto& hardware = aon::core::hardware();
  const auto requireMotion = [&](const aon::auton::MotionResult& result) {
    return requiredMotion(result, hardware);
  };

#if USING_BIG_ROBOT
  aon::auton::logStep("Staged Loader", "unsupported big robot");
  routine.stop();
  return 0;
#endif

  aon::auton::logStep("Staged Loader", "start");
  routine.setPose(0, 0, 0);
  aon::auton::mechanisms::finishLoaderCollection(hardware);

  // Heading zero points along +Y. Chain most of the loader approach, then
  // slow down for the heading-sensitive final alignment.
  aon::auton::logStep("Staged Loader", "fast loader approach");
  if (!requireMotion(routine.moveToPoint(
          "staged loader: fast approach", 0.0, 24.0, 1800,
          {.forwards = true,
           .maxSpeed = 70,
           .minSpeed = 35,
           .earlyExitRange = 8})))
    return 0;
  if (!requireMotion(routine.moveToPoint(
          "staged loader: alignment point", 0.0, 31.0, 1400,
          {.forwards = true, .maxSpeed = 45})))
    return 0;
  if (!requireMotion(routine.turnToHeading(
          "staged loader: face loader", 86.0, 1200,
          {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 45})))
    return 0;

  aon::auton::logStep("Staged Loader", "collect blocks");
  aon::auton::mechanisms::prepareLoaderCart(hardware);
  pros::delay(200);
  aon::auton::mechanisms::beginLoaderCollection(hardware);
  if (!requireMotion(routine.moveToPoint(
          "staged loader: slow contact", 4.0, 31.0, 1000,
          {.forwards = true, .maxSpeed = 30})))
    return 0;
  if (!requireMotion(
          routine.arcadeFor("staged loader: hold against loader", 25, 0, 250)))
    return 0;
  pros::delay(4000);
  aon::auton::mechanisms::finishLoaderCollection(hardware);

  aon::auton::logStep("Staged Loader", "back out");
  if (!requireMotion(routine.moveToPoint(
          "staged loader: fast retreat", -5.0, 31.0, 1300,
          {.forwards = false,
           .maxSpeed = 55,
           .minSpeed = 30,
           .earlyExitRange = 4})))
    return 0;
  if (!requireMotion(routine.moveToPoint(
          "staged loader: retreat finish", -9.0, 31.0, 1000,
          {.forwards = false, .maxSpeed = 35})))
    return 0;

  aon::auton::logStep("Staged Loader", "face long goal");
  aon::auton::mechanisms::resetLoaderCart(hardware);
  if (!requireMotion(routine.turnToHeading(
          "staged loader: face long goal", 171.0, 1600,
          {.direction = lemlib::AngularDirection::AUTO, .maxSpeed = 45})))
    return 0;

  aon::auton::logStep("Staged Loader", "precise score approach");
#if !USING_BIG_ROBOT
  aon::auton::mechanisms::prepareTopScorer(hardware);
#endif
  if (!requireMotion(routine.moveToPose(
          "staged loader: long goal", -8.0, 25.0, 171.0, 2200,
          {.forwards = true,
           .horizontalDrift = 8,
           .lead = 0.1,
           .maxSpeed = 35})))
    return 0;
  aon::auton::mechanisms::scoreTopBlocks(hardware, 3000);

  aon::auton::logStep("Staged Loader", "finish");
  routine.stop();
  aon::auton::mechanisms::stopAll(hardware);
  return 1;
}

int RunRedSixBlockHybridAuton(aon::auton::RedSixPhase stopAfter) {
  using aon::auton::RedSixBlock;
  using aon::auton::RedSixCallbacks;
  using aon::auton::RedSixPhase;
  auto& routine = aon::auton::actions();
  auto& hardware = aon::core::hardware();

#if USING_BIG_ROBOT
  (void)stopAfter;
  aon::auton::logStep(RedSixBlock::name, "unsupported big robot");
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  aon::auton::mechanisms::stopAll(hardware);
  return 0;
#else
  aon::auton::logStep(RedSixBlock::name, "start");
  // The pose is the tracking center at the repeatable red-side start. Heading
  // zero faces +Y; coordinates are local to this autonomous placement.
  routine.setPose(RedSixBlock::start.x, RedSixBlock::start.y,
                  RedSixBlock::start.heading);
  aon::auton::mechanisms::finishLoaderCollection(hardware);
  aon::auton::mechanisms::prepareLoaderCart(hardware);

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
        aon::auton::mechanisms::beginLoaderCollection(hardware);
        const std::uint32_t startedAt = pros::millis();
        // Seat against the loader briefly, then rely on the autonomous HOLD
        // brake mode for the remainder of collection instead of heating the
        // drivetrain against the wall for the full dwell.
        const auto seating = routine.arcadeFor(
            "red-six loader seating hold", 25, 0, 250,
            aon::auton::OdometryMonitoring::FailClosed);
        if (!seating.succeeded) {
          aon::auton::mechanisms::finishLoaderCollection(hardware);
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
        aon::auton::mechanisms::finishLoaderCollection(hardware);
        return completed;
      }
      case RedSixPhase::ReverseClearance:
        aon::auton::mechanisms::resetLoaderCart(hardware);
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
        aon::auton::mechanisms::prepareTopScorer(hardware);
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
            hardware, RedSixBlock::scoreTimeoutMs);
        return !routine.isCancellationLatched() &&
               !pros::competition::is_disabled();
    }
    return false;
  };
  callbacks.stopAll = [&] {
    routine.stop();
    aon::auton::mechanisms::finishLoaderCollection(hardware);
    aon::auton::mechanisms::stopAll(hardware);
  };

  const auto result = aon::auton::runRedSixSequence(callbacks, stopAfter);
  aon::auton::logStep(RedSixBlock::name,
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
}

int RunRedSixBlockHybridFull() {
  auto& routine = aon::auton::actions();
  if (!aon::config::activeRobotConfig()
           .autonomousAuthorizations.allows(
               aon::config::ExperimentalRoute::RedSixBlock)) {
    aon::auton::logStep(aon::auton::RedSixBlock::name,
                        "locked: physical validation required");
    routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
    return 0;
  }
  return RunRedSixBlockHybridAuton(aon::auton::RedSixPhase::ScoreSix);
}

}  // namespace aon::routines
