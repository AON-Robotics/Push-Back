#pragma once

#include "aon/auton/hybrid-sequence.hpp"

namespace aon::routines {

/**
 * @brief Competition autonomous slots exposed to the GUI and PROS callbacks.
 */
int RedRoutine1();
int RedRoutine2();
int RedRoutine3();
int BlueRoutine1();
int BlueRoutine2();
int BlueRoutine3();
int SkillsRoutine1();
int SkillsRoutine2();
int SkillsRoutine3();

// Native PROS route bodies kept for proven Kevin-style autonomous behavior.
void safeBigBotRoutine();
void bigBotCurves();
void bigBotContinuity();
void bigBotStayThere();
void bigBotLongGoalThenPark();
void bigBotPark();
void BigBotSkillsRoutine();
void smallBotRoutine();
void blackBeard();
void jackSparrow();
void smallBotRoutineWorlds();
void smallBotCurves();
void smallBotPark();
void smallbotjorgeg();

// LemLib-backed tests and experimental routes.
int RunLemLibTurnCharacterization(const char* name, double heading);
int RunLemLibForwardValidation();
int RunLemLibFigureEightValidation();
int RunJerryIoPathTest(const char* name);
int RunStagedLoaderScoreExperiment();
/**
 * @brief Runs the red-side six-block loader-to-long-goal autonomous.
 *
 * Starting configuration:
 * - Tracking center at the documented local red starting origin.
 * - Robot front facing +Y at heading zero.
 * - Loader cart and scorer clear for initialization.
 *
 * @param stopAfter Inclusive phase used by progressive physical gates.
 */
int RunRedSixBlockHybridAuton(
    aon::auton::RedSixPhase stopAfter = aon::auton::RedSixPhase::ScoreSix);
/** @brief Runs the complete red six-block route for selector registration. */
int RunRedSixBlockHybridFull();
int RunShadowPlayback();

// Low-speed native drivetrain checks. These never command mechanisms.
int RunNativeForwardReverseTest();
int RunNativeTurnTest();

}  // namespace aon::routines
