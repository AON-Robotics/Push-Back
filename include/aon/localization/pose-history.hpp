#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "aon/odometry/pose-estimator.hpp"

namespace aon::localization {

struct TimedPose {
  EstimatorPose pose{};
  std::uint32_t timestampMs = 0;
};

enum class PoseHistoryPushResult : std::uint8_t {
  Accepted,
  Invalid,
  OutOfOrder,
};

enum class PoseHistorySampleResult : std::uint8_t {
  Exact,
  Interpolated,
  Empty,
  TooOld,
  Future,
};

class PoseHistory {
 public:
  static constexpr std::size_t kCapacity = 32;

  [[nodiscard]] PoseHistoryPushResult push(TimedPose sample) noexcept;
  [[nodiscard]] PoseHistorySampleResult sampleAt(
      std::uint32_t timestampMs, TimedPose& sample) const noexcept;
  void clear() noexcept;
  [[nodiscard]] std::size_t size() const noexcept;

 private:
  [[nodiscard]] const TimedPose& logicalAt(std::size_t index) const noexcept;

  std::array<TimedPose, kCapacity> samples_{};
  std::size_t size_ = 0;
  std::size_t next_ = 0;
};

}  // namespace aon::localization
