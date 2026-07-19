#pragma once

#include "aon/auton/fallback-geometry.hpp"
#include "aon/auton/motion-control.hpp"
#include "aon/auton/motion-health.hpp"

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
  /**
   * @brief Executes one relative encoder motion under shared drive ownership.
   * @param request Relative motion, output cap, and timeout to execute.
   * @param control Ownership and cancellation state for the active action.
   * @return Final controller status and remaining error.
   */
  EncoderMotionResult execute(const EncoderMotionRequest& request,
                              MotionControl& control);

 private:
  EncoderMotionResult driveDistance(double distanceInches, int maximumOutput,
                                    std::uint32_t startedAt,
                                    std::uint32_t timeoutMs,
                                    MotionControl& control);
  EncoderMotionResult turn(double degrees, int maximumOutput,
                           bool imuAllowed, std::uint32_t startedAt,
                           std::uint32_t timeoutMs, MotionControl& control);
};

}  // namespace aon::auton
