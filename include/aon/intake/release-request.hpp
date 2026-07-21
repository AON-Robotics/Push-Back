#pragma once

#include <cstdint>

namespace aon::intake_sync {

constexpr std::uint32_t kReleaseActiveMask = 1U;
constexpr std::uint32_t kReleaseGenerationMask = 0x7fffffffU;

constexpr std::uint32_t nextReleaseRequest(std::uint32_t current,
                                           bool active) {
  const std::uint32_t generation =
      ((current >> 1U) + 1U) & kReleaseGenerationMask;
  return (generation << 1U) | (active ? kReleaseActiveMask : 0U);
}

constexpr bool releaseRequestActive(std::uint32_t request) {
  return (request & kReleaseActiveMask) != 0U;
}

}  // namespace aon::intake_sync
