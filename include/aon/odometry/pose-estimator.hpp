#pragma once

#include <cmath>

#include "aon/odometry/sensor-measurements.hpp"

namespace aon::localization {

struct TrackingGeometry {
  double leftOffsetInches{0.0};
  double rightOffsetInches{0.0};
  double backOffsetInches{0.0};
};

using WheelDeltas = WheelDistances;

struct LocalMotion {
  double rightInches{0.0};
  double forwardInches{0.0};
  double headingRadians{0.0};
  bool lateralValid{false};
};

struct EstimatorPose {
  double xInches{0.0};
  double yInches{0.0};
  double headingRadians{0.0};
};

/** @brief Returns true only when every pose component is finite. */
[[nodiscard]] inline bool isFinite(EstimatorPose pose) noexcept {
  return std::isfinite(pose.xInches) && std::isfinite(pose.yInches) &&
         std::isfinite(pose.headingRadians);
}

/** @brief Converts incremental wheel travel in inches to robot-local motion. */
LocalMotion localMotion(WheelDeltas wheels,
                        TrackingGeometry geometry) noexcept;

/** @brief Applies robot-local inches/radians to a field-frame pose. */
EstimatorPose propagatePose(EstimatorPose pose,
                            LocalMotion motion) noexcept;

}  // namespace aon::localization
