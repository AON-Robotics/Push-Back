#include "aon/auton/motion-health.hpp"

#include <cmath>

namespace aon::auton {

MotionHealthMonitor::MotionHealthMonitor(HealthThresholds thresholds)
    : thresholds_(thresholds) {}

void MotionHealthMonitor::reset(const MotionSample& sample) {
  previous_ = sample;
  trusted_ = sample;
  trackingBaseline_ = sample;
  invalidCount_ = 0;
  impossibleJumpCount_ = 0;
  leftTrackingBaselineAt_ = sample.timeMs;
  rightTrackingBaselineAt_ = sample.timeMs;
  initialized_ = true;
}

MotionFailureReason MotionHealthMonitor::observe(const MotionSample& sample,
                                                 MotionIntent intent) {
  if (!initialized_) reset(sample);

  const bool finitePose =
      sample.poseValid && std::isfinite(sample.poseX) &&
      std::isfinite(sample.poseY) && std::isfinite(sample.poseHeading);
  const bool devicesValid =
      sample.leftMotorValid && sample.rightMotorValid &&
      sample.leftTrackingValid && sample.rightTrackingValid &&
      sample.backTrackingValid && sample.imuValid;

  if (!finitePose || !devicesValid) {
    ++invalidCount_;
    previous_ = sample;
    if (invalidCount_ >= thresholds_.invalidConfirmationSamples) {
      return finitePose ? MotionFailureReason::DeviceInvalid
                        : MotionFailureReason::NonFinitePose;
    }
    return MotionFailureReason::None;
  }

  invalidCount_ = 0;

  const double poseJump =
      std::hypot(sample.poseX - previous_.poseX,
                 sample.poseY - previous_.poseY);
  const double headingJump = std::abs(
      std::remainder(sample.poseHeading - previous_.poseHeading, 360.0));
  const bool impossibleJump =
      poseJump > thresholds_.maxPoseJumpInchesPerSample ||
      headingJump > thresholds_.maxHeadingJumpDegreesPerSample;

  if (impossibleJump) {
    ++impossibleJumpCount_;
    previous_ = sample;
    if (impossibleJumpCount_ >=
        thresholds_.impossibleJumpConfirmationSamples) {
      return MotionFailureReason::ImpossiblePoseJump;
    }
    return MotionFailureReason::None;
  }

  impossibleJumpCount_ = 0;

  if (intent == MotionIntent::Idle) {
    trackingBaseline_ = sample;
    leftTrackingBaselineAt_ = sample.timeMs;
    rightTrackingBaselineAt_ = sample.timeMs;
  } else {
    const bool leftTrackingMoved =
        std::abs(sample.leftTrackingInches -
                 trackingBaseline_.leftTrackingInches) >=
        thresholds_.trackingMovementEpsilonInches;
    if (leftTrackingMoved) {
      trackingBaseline_.leftTrackingInches = sample.leftTrackingInches;
      trackingBaseline_.leftMotorDegrees = sample.leftMotorDegrees;
      leftTrackingBaselineAt_ = sample.timeMs;
    }

    const bool rightTrackingMoved =
        std::abs(sample.rightTrackingInches -
                 trackingBaseline_.rightTrackingInches) >=
        thresholds_.trackingMovementEpsilonInches;
    if (rightTrackingMoved) {
      trackingBaseline_.rightTrackingInches = sample.rightTrackingInches;
      trackingBaseline_.rightMotorDegrees = sample.rightMotorDegrees;
      rightTrackingBaselineAt_ = sample.timeMs;
    }

    const bool leftFrozen =
        sample.timeMs - leftTrackingBaselineAt_ >=
            thresholds_.frozenTrackingDwellMs &&
        std::abs(sample.leftMotorDegrees -
                 trackingBaseline_.leftMotorDegrees) >=
            thresholds_.frozenMotorDeltaDegrees;
    const bool rightFrozen =
        sample.timeMs - rightTrackingBaselineAt_ >=
            thresholds_.frozenTrackingDwellMs &&
        std::abs(sample.rightMotorDegrees -
                 trackingBaseline_.rightMotorDegrees) >=
            thresholds_.frozenMotorDeltaDegrees;
    if (leftFrozen || rightFrozen) {
      return MotionFailureReason::FrozenTracking;
    }
  }

  previous_ = sample;
  trusted_ = sample;
  return MotionFailureReason::None;
}

const MotionSample& MotionHealthMonitor::lastTrustedSample() const {
  return trusted_;
}

const char* motionFailureName(MotionFailureReason reason) {
  switch (reason) {
    case MotionFailureReason::None:
      return "none";
    case MotionFailureReason::DeviceInvalid:
      return "device-invalid";
    case MotionFailureReason::NonFinitePose:
      return "non-finite-pose";
    case MotionFailureReason::ImpossiblePoseJump:
      return "impossible-pose-jump";
    case MotionFailureReason::FrozenTracking:
      return "frozen-tracking";
    case MotionFailureReason::Timeout:
      return "timeout";
    case MotionFailureReason::Cancelled:
      return "cancelled";
    case MotionFailureReason::Unsupported:
      return "unsupported";
    case MotionFailureReason::RetryFailed:
      return "retry-failed";
  }
  return "unknown";
}

}  // namespace aon::auton
