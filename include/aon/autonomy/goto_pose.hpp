#pragma once

#ifndef AON_AUTONOMY_GOTO_POSE_HPP_
#define AON_AUTONOMY_GOTO_POSE_HPP_

#include <cstdint>

#include "aon/autonomy/trajectory_builder.hpp"

namespace aon::autonomy {

enum class GoToPoseStatus : std::uint8_t {
  reached = 0,
  timed_out = 1,
  no_path = 2,
};

struct GoToPoseOptions {
  std::uint32_t timeout_ms = 10000;
  TrajectoryBuildConfig build_cfg{};
};

GoToPoseStatus goToPose(double x_in,
                        double y_in,
                        double heading_rad,
                        const GoToPoseOptions &opts = {});

GoToPoseStatus goToPoseDeg(double x_in,
                           double y_in,
                           double heading_deg,
                           const GoToPoseOptions &opts = {});

void setGoToPoseCsvDebugEnabled(bool enable);
bool isGoToPoseCsvDebugEnabled();

}  // namespace aon::autonomy

#endif  // AON_AUTONOMY_GOTO_POSE_HPP_
