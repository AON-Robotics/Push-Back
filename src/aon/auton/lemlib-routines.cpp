#include "aon/auton/routines.hpp"
#include "aon/globals.hpp"
#include "aon/auton/actions.hpp"
#include "aon/constants.hpp"

#include "pros/rtos.hpp"

#include <cstdio>

ASSET(path_jerryio_txt);

namespace aon::routines {
namespace {

void logExperimentalStep(const char* step) {
  std::printf("LEMLIB_LOADER_SCORE_PARK step=%s time=%lu\n", step,
              static_cast<unsigned long>(pros::millis()));
}

}  // namespace

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

int RunLoaderScoreParkExperiment() {
  auto& routine = aon::auton::actions();

  logExperimentalStep("start");
  routine.setPose(0, 0, 0);
  intake.stopScan();

  logExperimentalStep("loader align");
  routine.moveToPoint("loader score park: align to loader", 24.0, 0.0, 2600,
                      {.forwards = true, .maxSpeed = 55});

  logExperimentalStep("cart drop and intake");
  intake.dropCart();
  pros::delay(150);
  intake.activateScan();

  logExperimentalStep("loader touch");
  routine.moveToPoint("loader score park: touch loader", 32.0, 0.0, 1800,
                      {.forwards = true, .maxSpeed = 35});
  pros::delay(500);
  intake.stopScan();

  logExperimentalStep("back out");
  routine.moveToPoint("loader score park: back out", 14.0, 0.0, 2200,
                      {.forwards = false, .maxSpeed = 45});

  logExperimentalStep("face score");
  intake.raiseCart();
  routine.turnToHeading("loader score park: face score", 170.0, 2200,
                        {.direction = lemlib::AngularDirection::AUTO,
                         .maxSpeed = 45});

  logExperimentalStep("score");
#if !USING_BIG_ROBOT
  intake.raiseScorer();
#endif
  routine.moveToPoint("loader score park: score approach", 22.0, 4.0, 1800,
                      {.forwards = true, .maxSpeed = 35});
  intake.score(Intake::TOP, 1800);

  logExperimentalStep("park setup");
  routine.moveToPoint("loader score park: park setup", 6.0, 0.0, 2200,
                      {.forwards = false, .maxSpeed = 45});
  routine.turnToHeading("loader score park: face park", -90.0, 1800,
                        {.direction = lemlib::AngularDirection::AUTO,
                         .maxSpeed = 45});

  logExperimentalStep("finish");
  routine.stop();
  intake.stop();
  return 1;
}

}  // namespace aon::routines
