#include "aon/competition/autonomous-routines.hpp"

#include "aon/constants.hpp"
#include "aon/globals.hpp"

namespace aon::routines {

#if USING_BIG_ROBOT
void bigBotStayThere();
void bigBotPark();
void BigBotSkillsRoutine();
#else
void smallBotRoutine();
void smallBotPark();
#endif

int LemLibTurnRoutine(const char* name, double heading);
int LemLibKevinInspiredRoutine();

namespace {

int runNativeRoutine(void (*routine)()) {
  routine();
  drivetrain.stop();
  intake.stop();
  return 1;
}

}  // namespace

int RedRoutine1() {
#if USING_BIG_ROBOT
  return runNativeRoutine(bigBotStayThere);
#else
  return runNativeRoutine(smallBotRoutine);
#endif
}

int RedRoutine2() {
#if USING_BIG_ROBOT
  return runNativeRoutine(bigBotPark);
#else
  return runNativeRoutine(smallBotPark);
#endif
}

int RedRoutine3() {
  return LemLibKevinInspiredRoutine();
}

int BlueRoutine1() {
#if USING_BIG_ROBOT
  return runNativeRoutine(bigBotStayThere);
#else
  return runNativeRoutine(smallBotRoutine);
#endif
}

int BlueRoutine2() {
#if USING_BIG_ROBOT
  return runNativeRoutine(bigBotPark);
#else
  return runNativeRoutine(smallBotPark);
#endif
}

int BlueRoutine3() {
  return LemLibKevinInspiredRoutine();
}

int SkillsRoutine1() {
#if USING_BIG_ROBOT
  BigBotSkillsRoutine();
#else
  smallBotRoutine();
  smallBotPark();
#endif
  drivetrain.stop();
  intake.stop();
  return 1;
}

int SkillsRoutine2() {
  return LemLibTurnRoutine("Skills 2 LemLib turn", -90);
}

int SkillsRoutine3() {
  return LemLibTurnRoutine("Skills 3 LemLib turn", 180);
}

}  // namespace aon::routines
