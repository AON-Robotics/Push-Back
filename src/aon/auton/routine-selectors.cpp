#include "aon/auton/routines.hpp"
#include "aon/auton/figure-eight-validation.hpp"
#include "aon/auton/red-six-block.hpp"
#include "aon/auton/status.hpp"
#include "aon/auton/fallback-status.hpp"

#include "aon/constants.hpp"
#include "aon/drivetrain/legacy-motion.hpp"
#include "aon/globals.hpp"
#include "aon/auton/actions.hpp"
#include "aon/shadow/service.hpp"

namespace aon::routines {

namespace {

int runNativeRoutine(const char* name, void (*routine)()) {
  aon::auton::startRoutine(name);
  aon::legacy_motion::prepare();
  routine();
  drivetrain.stop();
  intake.stop();
  aon::auton::finishRoutine(true);
  return 1;
}

int runRoutine(const char* name, int (*routine)()) {
  aon::auton::actions().resetCancellation();
  aon::auton::startRoutine(name);
  aon::auton::lockFallbackSelection();
  const int result = routine();
  drivetrain.stop();
  intake.stop();
  aon::auton::finishRoutine(result != 0);
  return result;
}

int runDrivetrainTest(const char* name, int (*routine)()) {
  aon::auton::startRoutine(name);
  aon::legacy_motion::prepare();
  const int result = routine();
  drivetrain.stop();
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
  return runRoutine(aon::auton::FigureEightValidation::name,
                    RunLemLibFigureEightValidation);
}

int SkillsRoutine1() {
  aon::auton::startRoutine("Skills AUT1");
  aon::legacy_motion::prepare();
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
  return runDrivetrainTest("TEST Drive 6in", RunNativeForwardReverseTest);
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
