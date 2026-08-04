#pragma once

namespace aon::routines {

/**
 * @brief Native autonomous route bodies selected for the big robot.
 *
 * @warning These synchronous routines directly command process-lifetime robot
 * hardware. They require completed hardware initialization, are not
 * thread-safe, and must only run in the matching robot configuration.
 */
void safeBigBotRoutine();
/** @copydoc safeBigBotRoutine() */
void bigBotCurves();
/** @copydoc safeBigBotRoutine() */
void bigBotContinuity();
/** @copydoc safeBigBotRoutine() */
void bigBotStayThere();
/** @copydoc safeBigBotRoutine() */
void bigBotLongGoalThenPark();
/** @copydoc safeBigBotRoutine() */
void bigBotPark();
/** @copydoc safeBigBotRoutine() */
void BigBotSkillsRoutine();

/**
 * @brief Native autonomous route bodies selected for the small robot.
 *
 * @warning These synchronous routines have the same initialization,
 * thread-safety, and robot-configuration requirements as the big-robot routes.
 */
void smallBotRoutine();
/** @copydoc smallBotRoutine() */
void blackBeard();
/** @copydoc smallBotRoutine() */
void jackSparrow();
/** @copydoc smallBotRoutine() */
void smallBotRoutineWorlds();
/** @copydoc smallBotRoutine() */
void smallBotCurves();
/** @copydoc smallBotRoutine() */
void smallBotPark();
/** @copydoc smallBotRoutine() */
void smallbotjorgeg();

}  // namespace aon::routines
