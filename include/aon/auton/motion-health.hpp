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
  Busy,
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

/**
 * @brief Decides whether automatic sensor-fault monitoring may run.
 * @param trackingMode true only while LemLib tracking owns the motion.
 * @param automaticFallbackAuthorized physical-test authorization flag.
 * @return true only when both prerequisites are satisfied.
 */
bool shouldMonitorAutomatically(bool trackingMode,
                                bool automaticFallbackAuthorized);

/**
 * @brief Prevents forced encoder mode before its physical validation gate.
 * @param forced true when the operator requests forced encoder mode.
 * @param forcedEncoderTestingAuthorized separate bench-test authorization.
 * @return true when the requested selection is allowed.
 */
bool forcedEncoderSelectionAllowed(bool forced,
                                   bool forcedEncoderTestingAuthorized);

/**
 * @brief Validates the independent feedback required by timed drive commands.
 * @param leftMotorValid validity of the left drive encoder sample.
 * @param rightMotorValid validity of the right drive encoder sample.
 * @return true only when both drivetrain sides reported valid feedback.
 */
bool driveFeedbackValid(bool leftMotorValid, bool rightMotorValid);

}  // namespace aon::auton
