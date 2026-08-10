#pragma once

#include <cstdint>

namespace aon::core {

/** Outcome of an idempotent RTOS task-start request. */
enum class TaskStartResult {
  Disabled,
  Started,
  AlreadyRunning,
  Failed,
};

/** Numeric task states mirrored from PROS `task_state_e_t`. */
enum class TaskState : std::uint32_t {
  Running = 0U,
  Ready,
  Blocked,
  Suspended,
  Deleted,
  Invalid,
};

/** Returns true for PROS states that identify a schedulable task. */
[[nodiscard]] constexpr bool isLiveTaskState(
    std::uint32_t state) noexcept {
  return state <= static_cast<std::uint32_t>(TaskState::Suspended);
}

}  // namespace aon::core
