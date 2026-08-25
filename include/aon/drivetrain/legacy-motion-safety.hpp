#pragma once

namespace aon::legacy_motion {

/**
 * @brief Decides whether a blocking legacy motion loop may keep commanding
 * the drivetrain.
 *
 * Keeping the field-disable input explicit makes the safety rule testable
 * without robot hardware. A disabled robot must never resume a motor command
 * merely because its target or timeout has not yet been reached.
 */
[[nodiscard]] constexpr bool shouldContinue(bool targetReached,
                                            bool timedOut,
                                            bool disabled) noexcept {
  return !targetReached && !timedOut && !disabled;
}

}  // namespace aon::legacy_motion
