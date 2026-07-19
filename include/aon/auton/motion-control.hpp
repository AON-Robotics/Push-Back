#pragma once

#include "aon/auton/motion-state.hpp"
#include "pros/rtos.hpp"

#include <utility>

namespace aon::auton {

/**
 * @brief Serializes autonomous drivetrain actions and latches cancellation.
 *
 * Callbacks run while the internal lock is held so an emergency stop cannot
 * complete between a cancellation check and a drivetrain command.
 */
class MotionControl {
 public:
  /**
   * @brief Claims the drivetrain for one autonomous action.
   * @return true when ownership was acquired; false when another action owns it.
   */
  bool tryBegin() {
    mutex_.take();
    const bool acquired = state_.tryBegin();
    mutex_.give();
    return acquired;
  }

  /** @brief Releases drivetrain ownership after the active action stops. */
  void finish() {
    mutex_.take();
    state_.finish();
    mutex_.give();
  }

  /** @brief Rearms motion when a new autonomous routine is dispatched. */
  void resetCancellation() {
    mutex_.take();
    state_.resetCancellation();
    mutex_.give();
  }

  /**
   * @brief Runs a drivetrain command only while the action remains active.
   * @param callback Command that must not race with emergency cancellation.
   * @return false when cancellation has already been requested.
   */
  template <typename Callback>
  bool runIfActive(Callback&& callback) {
    mutex_.take();
    const bool active = state_.isActive();
    if (active) std::forward<Callback>(callback)();
    mutex_.give();
    return active;
  }

  /**
   * @brief Latches cancellation and runs its stop command atomically.
   * @param callback Stop command that must precede all later drive commands.
   */
  template <typename Callback>
  void cancelAndRun(Callback&& callback) {
    mutex_.take();
    state_.cancel();
    std::forward<Callback>(callback)();
    mutex_.give();
  }

  /**
   * @brief Reports whether the current drivetrain owner was cancelled.
   * @return true until the cancelled owner releases the drivetrain.
   */
  bool isCancelled() const {
    mutex_.take();
    const bool cancelled = state_.isCancelled();
    mutex_.give();
    return cancelled;
  }

 private:
  mutable pros::Mutex mutex_;
  MotionState state_;
};

}  // namespace aon::auton
