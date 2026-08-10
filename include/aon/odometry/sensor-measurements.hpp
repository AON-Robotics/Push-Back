#pragma once

#include <cmath>
#include <cstdint>

namespace aon::localization {

inline constexpr double kPi = 3.14159265358979323846;

struct WheelDistances {
  double leftInches;
  double rightInches;
  double backInches;
  bool leftValid;
  bool rightValid;
  bool backValid;
};

struct ImuMeasurement {
  double headingRadians;
  bool valid;
};

struct GpsMeasurement {
  double xInches;
  double yInches;
  double headingRadians;
  double positionErrorInches;
  bool positionValid;
  bool headingValid;
  bool fresh;
  std::uint32_t timestampMs;
};

struct GpsValidationConfig {
  double minimumXInches;
  double maximumXInches;
  double minimumYInches;
  double maximumYInches;
  double maximumReportedErrorInches;
  double maximumJumpInches;
  double maximumHeadingJumpRadians;
  std::uint32_t minimumSamplePeriodMs;
  double maximumPositionNis;
  double maximumHeadingNis;
};

enum class GpsRejectionReason {
  None,
  NotFresh,
  InvalidPosition,
  NonFinite,
  StaleTimestamp,
  ExcessiveReportedError,
  OutsideFieldBounds,
  PositionJump,
  HeadingJump,
};

struct GpsGateResult {
  bool positionAccepted;
  bool headingAccepted;
  GpsRejectionReason reason;
};

inline double radians(double degreesValue) noexcept {
  return degreesValue * kPi / 180.0;
}

inline double degrees(double radiansValue) noexcept {
  return radiansValue * 180.0 / kPi;
}

inline double wrapRadians(double angle) noexcept {
  return std::remainder(angle, 2.0 * kPi);
}

inline double shortestAngleDelta(double from, double to) noexcept {
  return wrapRadians(to - from);
}

inline double sinc(double value) noexcept {
  // sin(z) / z loses useful precision near zero, where wheel increments are
  // most often sampled. The series keeps stationary updates well behaved.
  if (std::abs(value) < 1e-4) {
    const double squared = value * value;
    return 1.0 - squared / 6.0 + squared * squared / 120.0;
  }
  return std::sin(value) / value;
}

class GpsGate {
 public:
  explicit GpsGate(GpsValidationConfig config) noexcept : config_(config) {}

  void reset() noexcept {
    hasPosition_ = false;
    hasHeading_ = false;
    lastTimestampMs_ = 0U;
    lastXInches_ = 0.0;
    lastYInches_ = 0.0;
    lastHeadingRadians_ = 0.0;
  }

  GpsGateResult evaluate(GpsMeasurement measurement) noexcept {
    if (!measurement.fresh) {
      return {false, false, GpsRejectionReason::NotFresh};
    }
    if (!measurement.positionValid) {
      return {false, false, GpsRejectionReason::InvalidPosition};
    }
    if (!std::isfinite(measurement.xInches) ||
        !std::isfinite(measurement.yInches) ||
        !std::isfinite(measurement.positionErrorInches) ||
        (measurement.headingValid &&
         !std::isfinite(measurement.headingRadians))) {
      return {false, false, GpsRejectionReason::NonFinite};
    }

    if (hasPosition_) {
      const auto elapsed = static_cast<std::int32_t>(
          measurement.timestampMs - lastTimestampMs_);
      if (elapsed <= 0 ||
          static_cast<std::uint32_t>(elapsed) <
              config_.minimumSamplePeriodMs) {
        return {false, false, GpsRejectionReason::StaleTimestamp};
      }
    }
    if (measurement.positionErrorInches < 0.0 ||
        measurement.positionErrorInches >
            config_.maximumReportedErrorInches) {
      return {false, false,
              GpsRejectionReason::ExcessiveReportedError};
    }
    if (measurement.xInches < config_.minimumXInches ||
        measurement.xInches > config_.maximumXInches ||
        measurement.yInches < config_.minimumYInches ||
        measurement.yInches > config_.maximumYInches) {
      return {false, false, GpsRejectionReason::OutsideFieldBounds};
    }
    if (hasPosition_ &&
        std::hypot(measurement.xInches - lastXInches_,
                   measurement.yInches - lastYInches_) >
            config_.maximumJumpInches) {
      return {false, false, GpsRejectionReason::PositionJump};
    }

    lastXInches_ = measurement.xInches;
    lastYInches_ = measurement.yInches;
    lastTimestampMs_ = measurement.timestampMs;
    hasPosition_ = true;

    if (!measurement.headingValid) {
      return {true, false, GpsRejectionReason::None};
    }
    if (hasHeading_ &&
        std::abs(shortestAngleDelta(lastHeadingRadians_,
                                    measurement.headingRadians)) >
            config_.maximumHeadingJumpRadians) {
      return {true, false, GpsRejectionReason::HeadingJump};
    }

    lastHeadingRadians_ = wrapRadians(measurement.headingRadians);
    hasHeading_ = true;
    return {true, true, GpsRejectionReason::None};
  }

 private:
  GpsValidationConfig config_;
  bool hasPosition_{false};
  bool hasHeading_{false};
  std::uint32_t lastTimestampMs_{0U};
  double lastXInches_{0.0};
  double lastYInches_{0.0};
  double lastHeadingRadians_{0.0};
};

}  // namespace aon::localization
