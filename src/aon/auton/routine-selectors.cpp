#include "aon/auton/routines.hpp"
#include "aon/auton/lemlib-routes.hpp"
#include "aon/auton/native-routes.hpp"
#include "aon/auton/jerryio-path-auton.hpp"
#include "aon/auton/red-six-block.hpp"
#include "aon/auton/status.hpp"
#include "aon/auton/fallback-status.hpp"

#include "aon/constants.hpp"
#include "aon/drivetrain/legacy-motion.hpp"
#include "aon/globals.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/auton/actions.hpp"
#include "aon/shadow/service.hpp"

#include <cstdio>

namespace aon::routines {

namespace {

bool prepareNativeLocalization() {
  const core::TaskStartResult result = legacy_motion::prepare();
  if (result == core::TaskStartResult::Started ||
      result == core::TaskStartResult::AlreadyRunning) {
    return true;
  }
  std::fprintf(stderr, "Native localization task failed to start\n");
  drivetrain.stop();
  intake.stop();
  return false;
}

int runNativeRoutine(const char* name, void (*routine)()) {
  if (!aon::lemlib_integration::localizationReady()) return 0;
  aon::auton::startRoutine(name);
  if (!prepareNativeLocalization()) {
    aon::auton::finishRoutine(false);
    return 0;
  }
  routine();
  drivetrain.stop();
  intake.stop();
  aon::auton::finishRoutine(true);
  return 1;
}

int runRoutine(const char* name, int (*routine)()) {
  if (!aon::lemlib_integration::localizationReady()) return 0;
  aon::auton::actions().resetCancellation();
  aon::auton::startRoutine(name);
  aon::auton::lockFallbackSelection();
  const int result = routine();
  drivetrain.stop();
  intake.stop();
  aon::auton::finishRoutine(result != 0);
  return result;
}

}  // namespace

int RedRoutine1() {
#if USING_BIG_ROBOT
  return runNativeRoutine("Kevin Loader", bigBotStayThere);
#else
  return runNativeRoutine("Kevin Loader", smallBotRoutine);
#endif
}

int RedRoutine2() {
#if USING_BIG_ROBOT
  return runNativeRoutine("Kevin Park", bigBotPark);
#else
  return runNativeRoutine("Kevin Park", smallBotPark);
#endif
}

int RedRoutine3() {
  return runRoutine(aon::auton::RedSixBlock::name,
                    RunRedSixBlockHybridFull);
}

int BlueRoutine1() {
#if USING_BIG_ROBOT
  return runNativeRoutine("Kevin Loader", bigBotStayThere);
#else
  return runNativeRoutine("Kevin Loader", smallBotRoutine);
#endif
}

int BlueRoutine2() {
#if USING_BIG_ROBOT
  return runNativeRoutine("Kevin Park", bigBotPark);
#else
  return runNativeRoutine("Kevin Park", smallBotPark);
#endif
}

int BlueRoutine3() {
  return runRoutine(aon::auton::JerryIoPathAuton::name,
                    RunJerryIoPathAuton);
}

int SkillsRoutine1() {
  if (!aon::lemlib_integration::localizationReady()) return 0;
  aon::auton::startRoutine("Skills AUT1");
  if (!prepareNativeLocalization()) {
    aon::auton::finishRoutine(false);
    return 0;
  }
#if USING_BIG_ROBOT
  BigBotSkillsRoutine();
#else
  smallBotRoutine();
  smallBotPark();
#endif
  drivetrain.stop();
  intake.stop();
  aon::auton::finishRoutine(true);
  return 1;
}

int SkillsRoutine2() {
  return runRoutine("TEST LemLib Forward 12in",
                    RunLemLibForwardValidation);
}

int SkillsRoutine3() {
  return runRoutine("SHADOW PLAYBACK", RunShadowPlayback);
}

int RunShadowPlayback() {
  return aon::shadow::service().runArmedPlayback() ==
                 aon::shadow::ResultCode::Ok
             ? 1
             : 0;
}

}  // namespace aon::routines
