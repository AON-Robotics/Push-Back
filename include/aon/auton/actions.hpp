#pragma once

#include "lemlib/api.hpp"

namespace aon::auton {

/// Named autonomous operations backed by the project's existing LemLib chassis.
///
/// Keep route code behind this interface so logging, cancellation, and safety
/// behavior can be changed without rewriting every routine.
class Actions {
 public:
  void setPose(double x, double y, double heading);

  void moveToPoint(const char* name, double x, double y, int timeout,
                   lemlib::MoveToPointParams params = {});
  void moveToPose(const char* name, double x, double y, double heading,
                  int timeout, lemlib::MoveToPoseParams params = {});
  void turnToHeading(const char* name, double heading, int timeout,
                     lemlib::TurnToHeadingParams params = {});

  void cancelMotion();
  void stop();
};

Actions& actions();

}  // namespace aon::auton
