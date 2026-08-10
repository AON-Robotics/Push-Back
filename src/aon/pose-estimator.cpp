#include "aon/odometry/pose-estimator.hpp"

#include <cmath>
#include <limits>

namespace aon::localization {
namespace {

constexpr double kMinimumTrackerSeparation = 1e-9;

LocalMotion invalidMotion() noexcept {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  return {invalid, invalid, invalid, false};
}

EstimatorPose invalidPose() noexcept {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  return {invalid, invalid, invalid};
}

bool finiteGeometry(TrackingGeometry geometry) noexcept {
  return std::isfinite(geometry.leftOffsetInches) &&
         std::isfinite(geometry.rightOffsetInches) &&
         std::isfinite(geometry.backOffsetInches);
}

}  // namespace

LocalMotion localMotion(WheelDeltas wheels,
                        TrackingGeometry geometry) noexcept {
  const double separation =
      geometry.rightOffsetInches - geometry.leftOffsetInches;
  if (!wheels.leftValid || !wheels.rightValid ||
      !std::isfinite(wheels.leftInches) ||
      !std::isfinite(wheels.rightInches) || !finiteGeometry(geometry) ||
      std::abs(separation) < kMinimumTrackerSeparation) {
    return invalidMotion();
  }

  const double heading =
      (wheels.leftInches - wheels.rightInches) / separation;
  const double forward =
      ((wheels.leftInches + geometry.leftOffsetInches * heading) +
       (wheels.rightInches + geometry.rightOffsetInches * heading)) /
      2.0;

  // A missing lateral tracker must not fabricate sideways travel. Forward and
  // angular odometry remain useful, while diagnostics retain the degraded flag.
  const bool lateralValid =
      wheels.backValid && std::isfinite(wheels.backInches);
  const double right = lateralValid
                           ? wheels.backInches -
                                 geometry.backOffsetInches * heading
                           : 0.0;
  return {right, forward, heading, lateralValid};
}

EstimatorPose propagatePose(EstimatorPose pose,
                            LocalMotion motion) noexcept {
  if (!std::isfinite(pose.xInches) || !std::isfinite(pose.yInches) ||
      !std::isfinite(pose.headingRadians) ||
      !std::isfinite(motion.rightInches) ||
      !std::isfinite(motion.forwardInches) ||
      !std::isfinite(motion.headingRadians)) {
    return invalidPose();
  }

  const double halfHeading = motion.headingRadians / 2.0;
  const double fieldHeading = pose.headingRadians + halfHeading;
  const double scale = sinc(halfHeading);
  const double cosine = std::cos(fieldHeading);
  const double sine = std::sin(fieldHeading);

  return {
      pose.xInches +
          scale * (motion.rightInches * cosine +
                   motion.forwardInches * sine),
      pose.yInches +
          scale * (-motion.rightInches * sine +
                   motion.forwardInches * cosine),
      wrapRadians(pose.headingRadians + motion.headingRadians),
  };
}

}  // namespace aon::localization
