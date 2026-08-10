#pragma once

#include <cmath>
#include <cstdint>

namespace aon::localization {

inline constexpr double kPi = 3.14159265358979323846;

struct WheelDistances {
  double leftInches{0.0};
  double rightInches{0.0};
  double backInches{0.0};
  bool leftValid{false};
  bool rightValid{false};
  bool backValid{false};
};

inline WheelDistances consumeWheelDistances(
    WheelDistances current, WheelDistances& baselines) noexcept {
  const WheelDistances deltas{
      current.leftInches - baselines.leftInches,
      current.rightInches - baselines.rightInches,
      current.backInches - baselines.backInches,
      current.leftValid && baselines.leftValid,
      current.rightValid && baselines.rightValid,
      current.backValid && baselines.backValid,
  };
  // Invalid sensors also invalidate their baseline. The first recovered sample
  // establishes a new baseline instead of accumulating motion during the gap.
  baselines = current;
  return deltas;
}

struct ImuMeasurement {
  double headingRadians{0.0};
  bool valid{false};
};

struct GpsMeasurement {
  double xInches{0.0};
  double yInches{0.0};
  double headingRadians{0.0};
  double positionErrorInches{0.0};
  bool positionValid{false};
  bool headingValid{false};
  bool fresh{false};
  std::uint32_t timestampMs{0U};
};

struct GpsValidationConfig {
  double minimumXInches{0.0};
  double maximumXInches{0.0};
  double minimumYInches{0.0};
  double maximumYInches{0.0};
  double maximumReportedErrorInches{0.0};
  double maximumJumpInches{0.0};
  double maximumHeadingJumpRadians{0.0};
  std::uint32_t minimumSamplePeriodMs{0U};
  double maximumPositionNis{0.0};
  double maximumHeadingNis{0.0};
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
  InnovationRejected,
};

struct GpsGateResult {
  bool positionAccepted{false};
  bool headingAccepted{false};
  GpsRejectionReason reason{GpsRejectionReason::None};
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

  /** @brief Validates without advancing the accepted-sample baseline. */
  [[nodiscard]] GpsGateResult evaluate(
      GpsMeasurement measurement) const noexcept {
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

    if (!measurement.headingValid) {
      return {true, false, GpsRejectionReason::None};
    }
    if (hasHeading_ &&
        std::abs(shortestAngleDelta(lastHeadingRadians_,
                                    measurement.headingRadians)) >
            config_.maximumHeadingJumpRadians) {
      return {true, false, GpsRejectionReason::HeadingJump};
    }

    return {true, true, GpsRejectionReason::None};
  }

  /** @brief Advances only components accepted by the downstream filter. */
  void commit(GpsMeasurement measurement, bool positionAccepted,
              bool headingAccepted) noexcept {
    // Physical plausibility is only advanced after the EKF accepts the same
    // observation; a statistical outlier must not poison the next jump check.
    if (positionAccepted) {
      lastXInches_ = measurement.xInches;
      lastYInches_ = measurement.yInches;
      lastTimestampMs_ = measurement.timestampMs;
      hasPosition_ = true;
    }
    if (headingAccepted) {
      lastHeadingRadians_ = wrapRadians(measurement.headingRadians);
      hasHeading_ = true;
    }
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

class GpsFreshnessTracker {
 public:
  /** @brief Clears the last observed GPS signature. */
  void reset() noexcept { hasSample_ = false; }

  /** @brief Marks exact duplicate polls stale and timestamps changed samples. */
  bool observe(GpsMeasurement& sample, std::uint32_t observationMs) noexcept {
    const bool changed =
        !hasSample_ || sample.xInches != last_.xInches ||
        sample.yInches != last_.yInches ||
        sample.headingRadians != last_.headingRadians ||
        sample.positionErrorInches != last_.positionErrorInches ||
        sample.positionValid != last_.positionValid ||
        sample.headingValid != last_.headingValid;
    sample.fresh = changed;
    sample.timestampMs = changed ? observationMs : last_.timestampMs;
    if (changed) {
      last_ = sample;
      hasSample_ = true;
    }
    return changed;
  }

 private:
  bool hasSample_{false};
  GpsMeasurement last_{};
};

}  // namespace aon::localization
