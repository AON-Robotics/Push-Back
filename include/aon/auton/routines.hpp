#pragma once

namespace aon::routines {

/**
 * @name Competition autonomous slots
 *
 * Each slot synchronously commands initialized robot hardware through its
 * selected route, stops the drivetrain and intake before returning, and
 * reports execution through the shared autonomous status service.
 *
 * @return Nonzero when the selected route completes successfully; zero when a
 * route with recoverable failure reporting does not complete.
 * @warning These entry points are not thread-safe and must only be invoked by
 * the competition or isolated debug autonomous task.
 * @{
 */
/** @brief Runs the first red-alliance competition slot. */
int RedRoutine1();
/** @brief Runs the second red-alliance competition slot. */
int RedRoutine2();
/** @brief Runs the third red-alliance competition slot. */
int RedRoutine3();
/** @brief Runs the first blue-alliance competition slot. */
int BlueRoutine1();
/** @brief Runs the second blue-alliance competition slot. */
int BlueRoutine2();
/** @brief Runs the third blue-alliance competition slot. */
int BlueRoutine3();
/** @brief Runs the first skills/debug slot. */
int SkillsRoutine1();
/** @brief Runs the second skills/debug slot. */
int SkillsRoutine2();
/** @brief Runs the third skills/debug slot. */
int SkillsRoutine3();
/** @} */

}  // namespace aon::routines
