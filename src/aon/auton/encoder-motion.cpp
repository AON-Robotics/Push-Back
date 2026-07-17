#include "aon/auton/encoder-motion.hpp"

#include "aon/config/robot-config.hpp"
#include "aon/lemlib/drive-io.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace aon::auton {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDistanceFinishBand = 0.5;
constexpr double kHeadingFinishBand = 2.0;
constexpr std::uint32_t kLoopDelayMs = 20;

int controlledOutput(double error, double kP, int minimumOutput,
                     int maximumOutput) {
  if (maximumOutput <= 0 || error == 0.0) return 0;
  int output = static_cast<int>(std::round(error * kP));
  output = std::clamp(output, -maximumOutput, maximumOutput);
  if (output > 0 && output < minimumOutput) output = minimumOutput;
  if (output < 0 && output > -minimumOutput) output = -minimumOutput;
  return std::clamp(output, -maximumOutput, maximumOutput);
}

double motorDegreesToInches(double degrees) {
  const auto& config = aon::config::activeRobotConfig().lemlib;
  return degrees / 360.0 * kPi * config.driveWheelDiameter *
         config.fallback.wheelRevolutionsPerMotorRevolution;
}

bool timedOut(std::uint32_t startedAt, std::uint32_t timeoutMs) {
  return pros::millis() - startedAt >= timeoutMs;
}

EncoderMotionResult failure(MotionFailureReason reason, double distanceError,
                            double headingError) {
  aon::lemlib_integration::stopDrive();
  return {false, reason, distanceError, headingError};
}

}  // namespace

EncoderMotionResult EncoderMotionController::driveDistance(
    double distanceInches, int maximumOutput, std::uint32_t startedAt,
    std::uint32_t timeoutMs) {
  using namespace aon::lemlib_integration;
  const auto& fallback = aon::config::activeRobotConfig().lemlib.fallback;
  const DriveSensorSample start = sampleDriveSensors();
  if (!start.leftMotorValid || !start.rightMotorValid) {
    return failure(MotionFailureReason::RetryFailed, distanceInches, 0.0);
  }

  std::uint32_t settledAt = 0;
  double error = distanceInches;
  while (true) {
    if (cancelled_) return failure(MotionFailureReason::Cancelled, error, 0.0);
    if (timedOut(startedAt, timeoutMs)) {
      return failure(MotionFailureReason::Timeout, error, 0.0);
    }

    const DriveSensorSample sample = sampleDriveSensors();
    if (!sample.leftMotorValid || !sample.rightMotorValid) {
      return failure(MotionFailureReason::RetryFailed, error, 0.0);
    }
    const double averageDegrees =
        ((sample.leftMotorDegrees - start.leftMotorDegrees) +
         (sample.rightMotorDegrees - start.rightMotorDegrees)) /
        2.0;
    error = distanceInches - motorDegreesToInches(averageDegrees);

    if (std::abs(error) <= kDistanceFinishBand) {
      if (settledAt == 0) settledAt = pros::millis();
      stopDrive();
      if (pros::millis() - settledAt >= fallback.settleMs) {
        return {true, MotionFailureReason::None, error, 0.0};
      }
    } else {
      settledAt = 0;
      const int output = controlledOutput(
          error, fallback.distanceKp, fallback.minimumOutput, maximumOutput);
      commandTank(output, output);
    }
    pros::delay(kLoopDelayMs);
  }
}

EncoderMotionResult EncoderMotionController::turn(
    double degrees, int maximumOutput, bool imuAllowed,
    std::uint32_t startedAt, std::uint32_t timeoutMs) {
  using namespace aon::lemlib_integration;
  const auto& config = aon::config::activeRobotConfig().lemlib;
  const auto& fallback = config.fallback;
  const DriveSensorSample start = sampleDriveSensors();
  if (!start.leftMotorValid || !start.rightMotorValid) {
    return failure(MotionFailureReason::RetryFailed, 0.0, degrees);
  }

  bool usingImu = imuAllowed && start.imuValid;
  double lastHeadingDelta = 0.0;
  double encoderHeadingBase = 0.0;
  double leftEncoderBase = start.leftMotorDegrees;
  double rightEncoderBase = start.rightMotorDegrees;
  std::uint32_t settledAt = 0;
  double error = degrees;

  while (true) {
    if (cancelled_) return failure(MotionFailureReason::Cancelled, 0.0, error);
    if (timedOut(startedAt, timeoutMs)) {
      return failure(MotionFailureReason::Timeout, 0.0, error);
    }

    const DriveSensorSample sample = sampleDriveSensors();
    if (!sample.leftMotorValid || !sample.rightMotorValid) {
      return failure(MotionFailureReason::RetryFailed, 0.0, error);
    }

    double headingDelta = 0.0;
    if (usingImu && sample.imuValid) {
      headingDelta = std::remainder(sample.imuDegrees - start.imuDegrees, 360.0);
      lastHeadingDelta = headingDelta;
    } else {
      if (usingImu) {
        usingImu = false;
        encoderHeadingBase = lastHeadingDelta;
        leftEncoderBase = sample.leftMotorDegrees;
        rightEncoderBase = sample.rightMotorDegrees;
      }
      const double leftDistance = motorDegreesToInches(
          sample.leftMotorDegrees - leftEncoderBase);
      const double rightDistance = motorDegreesToInches(
          sample.rightMotorDegrees - rightEncoderBase);
      headingDelta = encoderHeadingBase +
                     (rightDistance - leftDistance) / config.trackWidth *
                         180.0 / kPi;
    }

    error = std::remainder(degrees - headingDelta, 360.0);
    if (std::abs(error) <= kHeadingFinishBand) {
      if (settledAt == 0) settledAt = pros::millis();
      stopDrive();
      if (pros::millis() - settledAt >= fallback.settleMs) {
        return {true, MotionFailureReason::None, 0.0, error};
      }
    } else {
      settledAt = 0;
      const int output = controlledOutput(
          error, fallback.turnKp, fallback.minimumOutput, maximumOutput);
      commandTank(-output, output);
    }
    pros::delay(kLoopDelayMs);
  }
}

EncoderMotionResult EncoderMotionController::execute(
    const EncoderMotionRequest& request) {
  cancelled_ = false;
  const auto& fallback =
      aon::config::activeRobotConfig().lemlib.fallback;
  const int maximumOutput = std::abs(cappedFallbackOutput(
      request.requestedMaxOutput, fallback.maximumOutputPercent));
  const std::uint32_t startedAt = pros::millis();

  if (std::abs(request.geometry.turnDegrees) > kHeadingFinishBand) {
    const auto result = turn(request.geometry.turnDegrees, maximumOutput,
                             request.imuAllowed, startedAt, request.timeoutMs);
    if (!result.succeeded) return result;
  }
  if (std::abs(request.geometry.distanceInches) > kDistanceFinishBand) {
    const auto result = driveDistance(request.geometry.distanceInches,
                                      maximumOutput, startedAt,
                                      request.timeoutMs);
    if (!result.succeeded) return result;
  }
  if (std::abs(request.geometry.finalTurnDegrees) > kHeadingFinishBand) {
    const auto result = turn(request.geometry.finalTurnDegrees, maximumOutput,
                             request.imuAllowed, startedAt, request.timeoutMs);
    if (!result.succeeded) return result;
  }
  aon::lemlib_integration::stopDrive();
  return {true, MotionFailureReason::None, 0.0, 0.0};
}

void EncoderMotionController::cancel() {
  cancelled_ = true;
  aon::lemlib_integration::stopDrive();
}

}  // namespace aon::auton
