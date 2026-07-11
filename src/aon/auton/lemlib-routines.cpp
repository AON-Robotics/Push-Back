#include "aon/auton/routines.hpp"
#include "aon/globals.hpp"
#include "aon/auton/actions.hpp"
#include "aon/auton/step-logger.hpp"
#include "aon/constants.hpp"

#include "pros/rtos.hpp"

ASSET(path_jerryio_txt);

namespace aon::routines {

int RunLemLibTurnCharacterization(const char* name, double heading) {
  auto& routine = aon::auton::actions();
  routine.setPose(0, 0, 0);
  routine.turnToHeading(name, heading, 2500,
                        {.direction = lemlib::AngularDirection::AUTO,
                         .maxSpeed = 50});
  routine.stop();
  return 1;
}

int RunJerryIoPathTest(const char* name) {
  auto& routine = aon::auton::actions();
  routine.setPose(0, 0, 20);
  routine.followPath(name, path_jerryio_txt, 10, 8000, true);
  routine.stop();
  return 1;
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
  intake.stopScan();

  // Heading zero points along +Y. Chain most of the loader approach, then
  // slow down for the heading-sensitive final alignment.
  aon::auton::logStep("Staged Loader", "fast loader approach");
  routine.moveToPoint("staged loader: fast approach", 0.0, 24.0, 1800,
                      {.forwards = true,
                       .maxSpeed = 70,
                       .minSpeed = 35,
                       .earlyExitRange = 8});
  routine.moveToPoint("staged loader: alignment point", 0.0, 31.0, 1400,
                      {.forwards = true, .maxSpeed = 45});
  routine.turnToHeading("staged loader: face loader", 86.0, 1200,
                        {.direction = lemlib::AngularDirection::AUTO,
                         .maxSpeed = 45});

  aon::auton::logStep("Staged Loader", "collect blocks");
  intake.dropCart();
  pros::delay(200);
  intake.activateScan();
  routine.moveToPoint("staged loader: slow contact", 4.0, 31.0, 1000,
                      {.forwards = true, .maxSpeed = 30});
  routine.arcadeFor("staged loader: hold against loader", 25, 0, 250);
  pros::delay(4000);
  intake.stopScan();

  aon::auton::logStep("Staged Loader", "back out");
  routine.moveToPoint("staged loader: fast retreat", -5.0, 31.0, 1300,
                      {.forwards = false,
                       .maxSpeed = 55,
                       .minSpeed = 30,
                       .earlyExitRange = 4});
  routine.moveToPoint("staged loader: retreat finish", -9.0, 31.0, 1000,
                      {.forwards = false, .maxSpeed = 35});

  aon::auton::logStep("Staged Loader", "face long goal");
  intake.raiseCart();
  routine.turnToHeading("staged loader: face long goal", 171.0, 1600,
                        {.direction = lemlib::AngularDirection::AUTO,
                         .maxSpeed = 45});

  aon::auton::logStep("Staged Loader", "precise score approach");
#if !USING_BIG_ROBOT
  intake.raiseScorer();
#endif
  routine.moveToPose("staged loader: long goal", -8.0, 25.0, 171.0, 2200,
                     {.forwards = true,
                      .horizontalDrift = 8,
                      .lead = 0.1,
                      .maxSpeed = 35});
  intake.score(Intake::TOP, 3000);

  aon::auton::logStep("Staged Loader", "finish");
  routine.stop();
  intake.stop();
  return 1;
}

}  // namespace aon::routines
