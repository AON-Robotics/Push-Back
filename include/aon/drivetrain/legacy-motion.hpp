#pragma once

namespace aon::legacy_motion {

/**
 * @brief Starts the legacy odometry task when a native motion routine needs it.
 *
 * Repeated calls are safe. Once started, the task remains active until the
 * program restarts, so LemLib validation should run from a fresh boot.
 */
void prepare();

}  // namespace aon::legacy_motion
