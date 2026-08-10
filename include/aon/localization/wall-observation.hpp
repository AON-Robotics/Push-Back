#pragma once

#include <cstdint>

#include "aon/lidar/scan-processor.hpp"
#include "aon/odometry/ekf.hpp"

namespace aon::localization {

enum class WallCorrectionResult : std::uint8_t {
  Accepted,
  Invalid,
  Future,
  Stale,
  Duplicate,
  RejectedByEstimator,
};

class WallObservationAdapter {
 public:
  explicit WallObservationAdapter(std::uint32_t maximumAgeMs) noexcept;

  [[nodiscard]] WallCorrectionResult apply(
      Ekf& estimator, const lidar::WallObservation& observation,
      double maximumNis, std::uint32_t nowMs) noexcept;
  void reset() noexcept;

 private:
  std::uint32_t maximumAgeMs_;
  bool hasTimestamp_ = false;
  std::uint32_t lastTimestampMs_ = 0;
};

}  // namespace aon::localization
