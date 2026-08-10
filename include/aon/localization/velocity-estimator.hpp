#pragma once

#include <cstdint>

#include "aon/odometry/pose-estimator.hpp"

namespace aon::localization {

struct Velocity2D {
  double xInchesPerSecond = 0.0;
  double yInchesPerSecond = 0.0;
  double angularRadiansPerSecond = 0.0;
};

struct VelocityEstimatorConfig {
  double smoothingAlpha = 0.35;
  double minimumDtSeconds = 0.001;
  double maximumDtSeconds = 0.2;
};

enum class VelocityUpdateResult : std::uint8_t {
  Accepted,
  InvalidConfig,
  InvalidTiming,
  InvalidMotion,
};

class VelocityEstimator {
 public:
  explicit VelocityEstimator(VelocityEstimatorConfig config) noexcept;

  [[nodiscard]] VelocityUpdateResult update(
      LocalMotion motion, double startingHeadingRadians,
      double dtSeconds) noexcept;
  [[nodiscard]] Velocity2D velocity() const noexcept;
  void reset() noexcept;

 private:
  VelocityEstimatorConfig config_;
  Velocity2D velocity_{};
  bool initialized_ = false;
};

}  // namespace aon::localization
