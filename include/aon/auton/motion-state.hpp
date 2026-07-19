#pragma once

namespace aon::auton {

/**
 * @brief Tracks autonomous drivetrain ownership without platform dependencies.
 *
 * Cancellation remains latched after the active action exits. A routine must
 * explicitly reset it before claiming the drivetrain again.
 */
class MotionState {
 public:
  /** @brief Claims an idle drivetrain for one action. */
  bool tryBegin() {
    if (state_ != State::Idle) return false;
    state_ = State::Active;
    return true;
  }

  /** @brief Releases a normally completed action without clearing cancellation. */
  void finish() {
    if (state_ == State::Active) state_ = State::Idle;
    if (state_ == State::CancelledActive) state_ = State::CancelledIdle;
  }

  /** @brief Latches emergency cancellation, including when currently idle. */
  void cancel() {
    if (state_ == State::Active) {
      state_ = State::CancelledActive;
    } else if (state_ == State::Idle) {
      state_ = State::CancelledIdle;
    }
  }

  /** @brief Rearms motion after the caller starts a new autonomous dispatch. */
  void resetCancellation() {
    if (state_ == State::CancelledIdle) state_ = State::Idle;
  }

  /** @brief Returns true only while an uncancelled action owns the drivetrain. */
  bool isActive() const { return state_ == State::Active; }

  /** @brief Returns true while emergency cancellation remains latched. */
  bool isCancelled() const {
    return state_ == State::CancelledActive ||
           state_ == State::CancelledIdle;
  }

 private:
  enum class State { Idle, Active, CancelledActive, CancelledIdle };
  State state_ = State::Idle;
};

}  // namespace aon::auton
