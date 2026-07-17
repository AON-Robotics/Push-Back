#pragma once

#include "aon/auton/fallback-geometry.hpp"
#include "aon/auton/motion-health.hpp"

#include <atomic>
#include <cstdint>

namespace aon::auton {

struct EncoderMotionRequest {
  const char* name;
  FallbackGeometry geometry;
  int requestedMaxOutput;
  std::uint32_t timeoutMs;
  bool imuAllowed;
};

struct EncoderMotionResult {
  bool succeeded;
  MotionFailureReason reason;
  double distanceErrorInches;
  double headingErrorDegrees;
};

class EncoderMotionController {
 public:
  EncoderMotionResult execute(const EncoderMotionRequest& request);
  void cancel();

 private:
  EncoderMotionResult driveDistance(double distanceInches, int maximumOutput,
                                    std::uint32_t startedAt,
                                    std::uint32_t timeoutMs);
  EncoderMotionResult turn(double degrees, int maximumOutput,
                           bool imuAllowed, std::uint32_t startedAt,
                           std::uint32_t timeoutMs);

  std::atomic_bool cancelled_{false};
};

}  // namespace aon::auton
