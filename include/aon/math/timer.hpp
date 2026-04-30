#pragma once

#include "../../api.h"

namespace aon {

/// @brief Non-blocking timer class to aid in control loops with timeouts
/// @note Uses \b milliseconds
class Timer {
 private:
  uint32_t duration;
  uint32_t startTime;

 public:
  Timer(uint32_t duration = 0)
      : duration(duration), startTime(pros::millis()) {}

  /// @brief Begins a timer of the set `duration`
  /// @param duration The amount of time in \b milliseconds to count down
  void start(uint32_t duration) {
    this->duration = duration;
    this->reset();
  }

  /// @brief Determines whether or not the timer has completed, i.e. if the set time has passed
  /// @return `true` if the set time has passed, `false` otherwise
  bool isCompleted() const { return this->timeRemaining() == 0; }

  /// @brief Sets the start time to now
  void reset() { startTime = pros::millis(); }

  /// @brief Calculates the remaing time of the timer
  /// @return The time in \b milliseconds remaining until the timer is completed
  uint32_t timeRemaining() const {
    uint32_t elapsed = pros::millis() - startTime;
    return (elapsed < duration) ? (duration - elapsed) : 0;
  }

  /// @brief Determines if the timer has started
  /// @return `true` if the timers `duration` is positive, `false` otherwise
  bool hasStarted() const { return this->duration > 0; }
};

}  // namespace aon
