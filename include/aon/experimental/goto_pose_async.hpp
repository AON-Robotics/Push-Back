#pragma once

#include <cstdint>

#include "aon/autonomy/goto_pose.hpp"

namespace aon::experimental {

inline void startGoToPoseWorker() {}

inline void enqueueGoToPose(double, double, double) {}

inline void enqueueGoToPoseDeg(double, double, double) {}

inline bool goToPoseBusy() { return false; }

inline bool goToPoseIdle() { return true; }

inline bool goToPoseSimpleDeg(double x_in,
                              double y_in,
                              double heading_deg,
                              std::uint32_t timeout_ms = 8000) {
  aon::autonomy::GoToPoseOptions opts{};
  opts.timeout_ms = timeout_ms;

  const aon::autonomy::GoToPoseStatus status =
      aon::autonomy::goToPoseDeg(x_in, y_in, heading_deg, opts);
  return status == aon::autonomy::GoToPoseStatus::reached;
}

}  // namespace aon::experimental
