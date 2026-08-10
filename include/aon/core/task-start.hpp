#pragma once

#include <cstdint>

namespace aon::core {

enum class TaskStartResult {
  Disabled,
  Started,
  AlreadyRunning,
  Failed,
};

enum class TaskState : std::uint32_t {
  Running = 0U,
  Ready,
  Blocked,
  Suspended,
  Deleted,
  Invalid,
};

[[nodiscard]] constexpr bool isLiveTaskState(
    std::uint32_t state) noexcept {
  return state <= static_cast<std::uint32_t>(TaskState::Suspended);
}

}  // namespace aon::core
