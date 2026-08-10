#include "aon/localization/wall-observation.hpp"

#include <cmath>

namespace aon::localization {

WallObservationAdapter::WallObservationAdapter(
    std::uint32_t maximumAgeMs) noexcept
    : maximumAgeMs_(maximumAgeMs) {}

WallCorrectionResult WallObservationAdapter::apply(
    Ekf& estimator, const lidar::WallObservation& observation,
    double maximumNis, std::uint32_t nowMs) noexcept {
  if (!std::isfinite(observation.positionInches) ||
      !std::isfinite(observation.variance) || observation.variance <= 0.0 ||
      !std::isfinite(observation.meanResidualInches) ||
      observation.support == 0 || std::isnan(maximumNis) || maximumNis < 0.0) {
    return WallCorrectionResult::Invalid;
  }
  const std::int32_t age =
      static_cast<std::int32_t>(nowMs - observation.captureTimestampMs);
  if (age < 0) return WallCorrectionResult::Future;
  if (static_cast<std::uint32_t>(age) > maximumAgeMs_) {
    return WallCorrectionResult::Stale;
  }
  if (hasTimestamp_ &&
      static_cast<std::int32_t>(observation.captureTimestampMs -
                                lastTimestampMs_) <= 0) {
    return WallCorrectionResult::Duplicate;
  }

  hasTimestamp_ = true;
  lastTimestampMs_ = observation.captureTimestampMs;
  const PositionAxis axis = observation.axis == lidar::WallAxis::X
                                ? PositionAxis::X
                                : PositionAxis::Y;
  if (!estimator.updateAxisPosition(axis, observation.positionInches,
                                    observation.variance, maximumNis)) {
    return WallCorrectionResult::RejectedByEstimator;
  }
  return WallCorrectionResult::Accepted;
}

void WallObservationAdapter::reset() noexcept {
  hasTimestamp_ = false;
  lastTimestampMs_ = 0;
}

}  // namespace aon::localization
