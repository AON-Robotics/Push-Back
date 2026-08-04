#pragma once

#include "aon/constants.hpp"

/**
 * @brief Legacy native test entry points for drivetrain, odometry, and sensors.
 *
 * @warning These functions directly command process-lifetime global hardware.
 * They are not thread-safe, do not perform ownership arbitration, and must run
 * only in an isolated test slot with the robot secured or in a clear field.
 * Several tests loop indefinitely and require an external competition-task or
 * controller stop. Units and ranges remain those of the underlying legacy API.
 */
namespace aon::tests {

/** @brief Drives an octagonal GPS validation route. */
void gpsOctagon();

/**
 * @brief Runs the distance-sensor speed experiment indefinitely.
 * @param rpm Motor velocity in revolutions per minute.
 */
void distanceSensorSpeed(double rpm = MAX_RPM);

/** @brief Exercises legacy odometry moves and turns. */
void odom();

/** @brief Exercises concurrent intake scanning and drivetrain output. */
void concurrency();

/** @brief Repeatedly aligns the robot to a red vision target. */
void alignment();

/** @brief Continuously reports raw and filtered vision distance estimates. */
void visionSensorDistance();

/** @brief Continuously compares gyroscope extended Kalman filter settings. */
void gyroWithEKF();

/** @brief Runs the potentiometer-adjustable single native motion test. */
int adjustable();

/** @brief Selects one of three native arc tests from the potentiometer. */
int multiple();

/** @brief Exercises repeated clockwise and counterclockwise turns. */
void turns();

/** @brief Exercises a four-sided move-and-turn sequence. */
void square();

/** @brief Exercises chained legacy arc and straight motions. */
void continuity();

/** @brief Enables the intake color-sorting scan test. */
void colorSorting();

/** @brief Exercises point-to-point pure-pursuit poses. */
void purePursuitPoint();

/** @brief Exercises a short pure-pursuit pose sequence. */
void purePursuitSimpleFollow();

/** @brief Exercises the extended S-shaped pure-pursuit path. */
void purePursuitPath();

#if !USING_BIG_ROBOT
/** @brief Exercises the small robot's X-drive pose sequence. */
void xDriveRoutine();
#endif

}  // namespace aon::tests

namespace aon::routines {

/**
 * @brief Moves six inches forward and backward at reduced native-drive speed.
 * @return One after the drivetrain is stopped and its velocity limit restored.
 * @warning Commands drivetrain hardware synchronously; mechanisms are never
 * commanded. Run only with the robot secured or in a clear field.
 */
int RunNativeForwardReverseTest();

/**
 * @brief Turns 45 degrees clockwise and counterclockwise at reduced speed.
 * @return One after the drivetrain is stopped and its velocity limit restored.
 * @warning Commands drivetrain hardware synchronously; mechanisms are never
 * commanded. Run only with the robot secured or in a clear field.
 */
int RunNativeTurnTest();

}  // namespace aon::routines
