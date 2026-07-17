#pragma once

#include <cstdint>

namespace aon::auton {

enum class MotionIntent { Idle, Linear, Turn };

enum class MotionFailureReason {
  None,
  DeviceInvalid,
  NonFinitePose,
  ImpossiblePoseJump,
  FrozenTracking,
  Timeout,
  Cancelled,
  Unsupported,
  RetryFailed,
};

struct MotionSample {
  std::uint32_t timeMs = 0;
  double poseX = 0.0;
  double poseY = 0.0;
  double poseHeading = 0.0;
  double leftMotorDegrees = 0.0;
  double rightMotorDegrees = 0.0;
  double leftTrackingInches = 0.0;
  double rightTrackingInches = 0.0;
  double backTrackingInches = 0.0;
  double imuDegrees = 0.0;
  bool poseValid = false;
  bool leftMotorValid = false;
  bool rightMotorValid = false;
  bool leftTrackingValid = false;
  bool rightTrackingValid = false;
  bool backTrackingValid = false;
  bool imuValid = false;
};

struct HealthThresholds {
  std::uint32_t invalidConfirmationSamples = 3;
  std::uint32_t impossibleJumpConfirmationSamples = 2;
  std::uint32_t frozenTrackingDwellMs = 300;
  double frozenMotorDeltaDegrees = 15.0;
  double trackingMovementEpsilonInches = 0.02;
  double maxPoseJumpInchesPerSample = 8.0;
  double maxHeadingJumpDegreesPerSample = 45.0;
};

class MotionHealthMonitor {
 public:
  explicit MotionHealthMonitor(HealthThresholds thresholds);

  void reset(const MotionSample& sample);
  MotionFailureReason observe(const MotionSample& sample, MotionIntent intent);
  const MotionSample& lastTrustedSample() const;

 private:
  HealthThresholds thresholds_;
  MotionSample previous_{};
  MotionSample trusted_{};
  MotionSample trackingBaseline_{};
  std::uint32_t invalidCount_ = 0;
  std::uint32_t impossibleJumpCount_ = 0;
  std::uint32_t leftTrackingBaselineAt_ = 0;
  std::uint32_t rightTrackingBaselineAt_ = 0;
  bool initialized_ = false;
};

const char* motionFailureName(MotionFailureReason reason);

}  // namespace aon::auton
