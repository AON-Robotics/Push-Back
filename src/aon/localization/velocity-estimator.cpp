#include "aon/localization/velocity-estimator.hpp"

#include <cmath>

namespace aon::localization {
namespace {

bool validConfig(const VelocityEstimatorConfig& config) noexcept {
  return std::isfinite(config.smoothingAlpha) &&
         config.smoothingAlpha > 0.0 && config.smoothingAlpha <= 1.0 &&
         std::isfinite(config.minimumDtSeconds) &&
         std::isfinite(config.maximumDtSeconds) &&
         config.minimumDtSeconds > 0.0 &&
         config.maximumDtSeconds >= config.minimumDtSeconds;
}

double filtered(double previous, double sample, double alpha) noexcept {
  return previous + alpha * (sample - previous);
}

}  // namespace

VelocityEstimator::VelocityEstimator(VelocityEstimatorConfig config) noexcept
    : config_(config) {}

VelocityUpdateResult VelocityEstimator::update(
    LocalMotion motion, double startingHeadingRadians,
    double dtSeconds) noexcept {
  if (!validConfig(config_)) return VelocityUpdateResult::InvalidConfig;
  if (!std::isfinite(dtSeconds) ||
      dtSeconds < config_.minimumDtSeconds ||
      dtSeconds > config_.maximumDtSeconds) {
    return VelocityUpdateResult::InvalidTiming;
  }
  if (!std::isfinite(motion.rightInches) ||
      !std::isfinite(motion.forwardInches) ||
      !std::isfinite(motion.headingRadians) ||
      !std::isfinite(startingHeadingRadians)) {
    return VelocityUpdateResult::InvalidMotion;
  }

  const double midpointHeading =
      startingHeadingRadians + motion.headingRadians * 0.5;
  const double right = motion.lateralValid ? motion.rightInches : 0.0;
  const double fieldX = right * std::cos(midpointHeading) +
                        motion.forwardInches * std::sin(midpointHeading);
  const double fieldY = -right * std::sin(midpointHeading) +
                        motion.forwardInches * std::cos(midpointHeading);
  const Velocity2D sample{fieldX / dtSeconds, fieldY / dtSeconds,
                          motion.headingRadians / dtSeconds};
  if (!initialized_) {
    velocity_ = sample;
    initialized_ = true;
  } else {
    velocity_.xInchesPerSecond =
        filtered(velocity_.xInchesPerSecond, sample.xInchesPerSecond,
                 config_.smoothingAlpha);
    velocity_.yInchesPerSecond =
        filtered(velocity_.yInchesPerSecond, sample.yInchesPerSecond,
                 config_.smoothingAlpha);
    velocity_.angularRadiansPerSecond =
        filtered(velocity_.angularRadiansPerSecond,
                 sample.angularRadiansPerSecond, config_.smoothingAlpha);
  }
  return VelocityUpdateResult::Accepted;
}

Velocity2D VelocityEstimator::velocity() const noexcept { return velocity_; }

void VelocityEstimator::reset() noexcept {
  velocity_ = {};
  initialized_ = false;
}

}  // namespace aon::localization
