#include "aon/auton/motion-health.hpp"

#include <cstdint>
#include <cstdlib>
#include <iostream>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

using aon::auton::MotionFailureReason;
using aon::auton::MotionHealthMonitor;
using aon::auton::MotionIntent;
using aon::auton::MotionSample;

namespace {

MotionSample healthy(std::uint32_t timeMs) {
  MotionSample sample{};
  sample.timeMs = timeMs;
  sample.poseValid = true;
  sample.leftMotorValid = true;
  sample.rightMotorValid = true;
  sample.leftTrackingValid = true;
  sample.rightTrackingValid = true;
  sample.backTrackingValid = true;
  sample.imuValid = true;
  return sample;
}

void testInvalidSamplesRequireConfirmation() {
  MotionHealthMonitor monitor({});
  auto sample = healthy(0);
  monitor.reset(sample);

  sample = healthy(20);
  sample.leftTrackingValid = false;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);

  sample = healthy(40);
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);

  for (std::uint32_t timeMs : {60U, 80U}) {
    sample = healthy(timeMs);
    sample.leftTrackingValid = false;
    CHECK(monitor.observe(sample, MotionIntent::Linear) ==
          MotionFailureReason::None);
  }

  sample = healthy(100);
  sample.leftTrackingValid = false;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::DeviceInvalid);
}

void testFrozenTrackingRequiresMotorMovementAndDwell() {
  MotionHealthMonitor monitor({});
  monitor.reset(healthy(0));

  for (std::uint32_t timeMs = 20; timeMs < 300; timeMs += 20) {
    auto sample = healthy(timeMs);
    sample.leftMotorDegrees = static_cast<double>(timeMs) / 10.0;
    sample.rightMotorDegrees = static_cast<double>(timeMs) / 10.0;
    CHECK(monitor.observe(sample, MotionIntent::Linear) ==
          MotionFailureReason::None);
  }

  auto sample = healthy(300);
  sample.leftMotorDegrees = 30.0;
  sample.rightMotorDegrees = 30.0;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::FrozenTracking);
}

void testBlockedDriveDoesNotLookLikeFrozenTracking() {
  MotionHealthMonitor monitor({});
  monitor.reset(healthy(0));

  for (std::uint32_t timeMs = 20; timeMs <= 500; timeMs += 20) {
    CHECK(monitor.observe(healthy(timeMs), MotionIntent::Linear) ==
          MotionFailureReason::None);
  }
}

void testImpossiblePoseJumpRequiresConfirmation() {
  MotionHealthMonitor monitor({});
  monitor.reset(healthy(0));

  auto sample = healthy(20);
  sample.poseX = 9.0;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);

  sample = healthy(40);
  sample.poseX = 18.0;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::ImpossiblePoseJump);
}

}  // namespace

int main() {
  testInvalidSamplesRequireConfirmation();
  testFrozenTrackingRequiresMotorMovementAndDwell();
  testBlockedDriveDoesNotLookLikeFrozenTracking();
  testImpossiblePoseJumpRequiresConfirmation();
  std::cout << "motion fallback tests passed\n";
  return 0;
}
