#pragma once

#include <cmath>

#include "aon/odometry/sensor-measurements.hpp"

namespace aon::localization {

struct TrackingGeometry {
  double leftOffsetInches;
  double rightOffsetInches;
  double backOffsetInches;
};

using WheelDeltas = WheelDistances;

struct LocalMotion {
  double rightInches;
  double forwardInches;
  double headingRadians;
  bool lateralValid;
};

struct EstimatorPose {
  double xInches;
  double yInches;
  double headingRadians;
};

/** Returns true only when every pose component is finite. */
[[nodiscard]] inline bool isFinite(EstimatorPose pose) noexcept {
  return std::isfinite(pose.xInches) && std::isfinite(pose.yInches) &&
         std::isfinite(pose.headingRadians);
}

LocalMotion localMotion(WheelDeltas wheels,
                        TrackingGeometry geometry) noexcept;
EstimatorPose propagatePose(EstimatorPose pose,
                            LocalMotion motion) noexcept;

}  // namespace aon::localization
