#pragma once

#include <cstdint>
#include <functional>

#include "aon/auton/fallback-status.hpp"
#include "lemlib/api.hpp"

namespace aon::auton {

/** Controls whether configured encoder fallback may be used after odometry fails. */
enum class OdometryMonitoring { Configured, FailClosed };

/** Result of one synchronous autonomous motion request. */
struct MotionResult {
  bool succeeded;
  MotionMode mode;
  bool fallbackUsed;
  MotionFailureReason reason;
};

/**
 * @brief Synchronous autonomous operations backed by the LemLib chassis.
 *
 * Coordinates are inches in the active field frame and headings are degrees.
 * Timeouts and durations are milliseconds. Motion methods may command motors,
 * block until completion, update fallback status, and stop early when
 * cancellation or odometry monitoring fails. The process-owned instance is
 * valid for program lifetime and is not safe for concurrent motion commands.
 */
class Actions {
 public:
  /** Resets LemLib odometry to an absolute pose in inches and degrees. */
  void setPose(double x, double y, double heading);

  /** Moves to a field point, returning the final motion/fallback status. */
  MotionResult moveToPoint(const char* name, double x, double y, int timeout,
                           lemlib::MoveToPointParams params = {},
                           OdometryMonitoring monitoring =
                               OdometryMonitoring::Configured);
  /** Moves to a field pose, returning the final motion/fallback status. */
  MotionResult moveToPose(const char* name, double x, double y, double heading,
                          int timeout, lemlib::MoveToPoseParams params = {},
                          OdometryMonitoring monitoring =
                              OdometryMonitoring::Configured);
  /** Turns to an absolute heading in degrees. */
  MotionResult turnToHeading(const char* name, double heading, int timeout,
                             lemlib::TurnToHeadingParams params = {});
  /** Applies normalized arcade commands in [-127, 127] for durationMs. */
  MotionResult arcadeFor(const char* name, int throttle, int turn,
                         std::uint32_t durationMs,
                         OdometryMonitoring monitoring =
                             OdometryMonitoring::Configured);
  /** Follows a LemLib path asset; onPoll runs from the motion polling loop. */
  MotionResult followPath(const char* name, const asset& path, float lookahead,
                          int timeout, bool forwards = true,
                          const std::function<void()>& onPoll = {},
                          OdometryMonitoring monitoring =
                              OdometryMonitoring::Configured);

  /** Latches cancellation and requests any active LemLib motion to stop. */
  void cancelMotion();
  /** Stops any idle output and rearms the drivetrain for a new routine. */
  bool prepareMotion();
  /** @brief Rearms drivetrain actions at the start of a new autonomous run. */
  void resetCancellation();
  /** @brief Reports whether emergency cancellation remains latched. */
  bool isCancellationLatched() const;
  /** Stops the drivetrain using the requested PROS brake mode. */
  void stop(pros::motor_brake_mode_e brakeMode = pros::E_MOTOR_BRAKE_HOLD);
};

/** Returns the non-owning, process-lifetime autonomous action service. */
Actions& actions();

}  // namespace aon::auton
