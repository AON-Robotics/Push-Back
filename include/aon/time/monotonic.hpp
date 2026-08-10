#pragma once

#include <cstdint>
#include <limits>

namespace aon::time {

/// True when a nonzero interval is unambiguous for signed modular ordering.
[[nodiscard]] constexpr bool validInterval(
    std::uint32_t duration) noexcept {
  return duration > 0U &&
         duration < static_cast<std::uint32_t>(
                        std::numeric_limits<std::int32_t>::max());
}

/// @brief Returns the signed modular distance from earlier to later.
/// @details Comparisons are unambiguous when timestamps differ by less than
/// half of the uint32_t range, which is over 24 days for millisecond clocks.
[[nodiscard]] constexpr std::int32_t delta(
    std::uint32_t later, std::uint32_t earlier) noexcept {
  return static_cast<std::int32_t>(later - earlier);
}

/// @brief Reports whether candidate occurs strictly after reference.
[[nodiscard]] constexpr bool strictlyAfter(
    std::uint32_t candidate, std::uint32_t reference) noexcept {
  return delta(candidate, reference) > 0;
}

/// @brief Returns elapsed ticks using unsigned modular arithmetic.
[[nodiscard]] constexpr std::uint32_t elapsed(
    std::uint32_t now, std::uint32_t since) noexcept {
  return now - since;
}

/// @brief Reports whether at least duration ticks have elapsed.
[[nodiscard]] constexpr bool elapsedAtLeast(
    std::uint32_t now, std::uint32_t since,
    std::uint32_t duration) noexcept {
  return elapsed(now, since) >= duration;
}

/// @brief Reports whether now is strictly before a modular deadline.
[[nodiscard]] constexpr bool beforeDeadline(
    std::uint32_t now, std::uint32_t deadline) noexcept {
  return delta(deadline, now) > 0;
}

}  // namespace aon::time
