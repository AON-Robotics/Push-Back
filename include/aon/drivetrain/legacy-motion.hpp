#pragma once

#include "aon/core/task-start.hpp"

namespace aon::legacy_motion {

/**
 * @brief Starts the legacy odometry task when a native motion routine needs it.
 *
 * Repeated calls are safe. Once started, the task remains active until the
 * program restarts, so LemLib validation should run from a fresh boot.
 * @return `Started`, `AlreadyRunning`, or `Failed`; never starts implicitly
 * when fused localization is selected but unavailable.
 */
[[nodiscard]] core::TaskStartResult prepare();

}  // namespace aon::legacy_motion
