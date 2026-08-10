#pragma once

#include <array>

#include "aon/odometry/pose-estimator.hpp"

namespace aon::localization {

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct EkfConfig {
  double initialPositionVariance;
  double initialHeadingVariance;
  double stationaryPositionVariance;
  double stationaryHeadingVariance;
  double positionVariancePerInch;
  double headingVariancePerRadian;
  double imuHeadingVariance;
  double gpsPositionVariance;
  double gpsHeadingVariance;
  double singularityTolerance;
};

struct CovarianceDiagonal {
  double xVariance;
  double yVariance;
  double headingVariance;
};

class Ekf {
 public:
  explicit Ekf(EkfConfig config) noexcept;

  void reset(EstimatorPose pose) noexcept;
  bool predict(LocalMotion motion) noexcept;

  EstimatorPose pose() const noexcept;
  Matrix3 covariance() const noexcept;
  CovarianceDiagonal covarianceDiagonal() const noexcept;

 private:
  EkfConfig config_;
  EstimatorPose state_{};
  Matrix3 covariance_{};
};

}  // namespace aon::localization
