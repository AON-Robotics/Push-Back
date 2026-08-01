#pragma once

#include "aon/shadow/player.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace aon::shadow {

constexpr std::size_t kRuntimePathCapacity = 64U * 1024U;

struct RuntimePath {
  std::array<std::uint8_t, kRuntimePathCapacity> bytes{};
  std::size_t used = 0;
};

ResultCode serializeRuntimePath(const ProcessedRoute& route,
                                const RouteSegment& segment,
                                RuntimePath& output);
ResultCode validateRuntimePaths(const ProcessedRoute& route,
                                RuntimePath& scratch);
ResultCode playOnRobot(const DecodedRecording& recording,
                       const PlaybackPolicy& policy);
ResultCode prepareRobotPlayback();
void cancelRobotPlayback();

}  // namespace aon::shadow
