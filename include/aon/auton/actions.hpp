#pragma once

#include <cstdint>
#include <functional>

#include "aon/auton/fallback-status.hpp"
#include "lemlib/api.hpp"

namespace aon::auton {

enum class OdometryMonitoring { Configured, FailClosed };

struct MotionResult {
  bool succeeded;
  MotionMode mode;
  bool fallbackUsed;
  MotionFailureReason reason;
};

/// Named autonomous operations backed by the project's existing LemLib chassis.
///
/// Keep route code behind this interface so logging, cancellation, and safety
/// behavior can be changed without rewriting every routine.
class Actions {
 public:
  void setPose(double x, double y, double heading);

  MotionResult moveToPoint(const char* name, double x, double y, int timeout,
                           lemlib::MoveToPointParams params = {});
  MotionResult moveToPose(const char* name, double x, double y, double heading,
                          int timeout, lemlib::MoveToPoseParams params = {});
  MotionResult turnToHeading(const char* name, double heading, int timeout,
                             lemlib::TurnToHeadingParams params = {});
  MotionResult arcadeFor(const char* name, int throttle, int turn,
                         std::uint32_t durationMs);
  MotionResult followPath(const char* name, const asset& path, float lookahead,
                          int timeout, bool forwards = true,
                          const std::function<void()>& onPoll = {},
                          OdometryMonitoring monitoring =
                              OdometryMonitoring::Configured);

  void cancelMotion();
  /** Stops any idle output and rearms the drivetrain for a new routine. */
  bool prepareMotion();
  /** @brief Rearms drivetrain actions at the start of a new autonomous run. */
  void resetCancellation();
  void stop(pros::motor_brake_mode_e brakeMode = pros::E_MOTOR_BRAKE_HOLD);
};

Actions& actions();

}  // namespace aon::auton
