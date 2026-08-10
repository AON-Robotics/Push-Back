#pragma once

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

LocalMotion localMotion(WheelDeltas wheels,
                        TrackingGeometry geometry) noexcept;
EstimatorPose propagatePose(EstimatorPose pose,
                            LocalMotion motion) noexcept;

}  // namespace aon::localization
