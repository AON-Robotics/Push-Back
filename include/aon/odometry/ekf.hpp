#pragma once

#include <array>

#include "aon/odometry/pose-estimator.hpp"

namespace aon::localization {

using Matrix3 = std::array<std::array<double, 3>, 3>;

struct EkfConfig {
  double initialPositionVariance{0.0};
  double initialHeadingVariance{0.0};
  double stationaryPositionVariance{0.0};
  double stationaryHeadingVariance{0.0};
  double positionVariancePerInch{0.0};
  double headingVariancePerRadian{0.0};
  double imuHeadingVariance{0.0};
  double gpsPositionVariance{0.0};
  double gpsHeadingVariance{0.0};
  double singularityTolerance{0.0};
};

struct CovarianceDiagonal {
  double xVariance{0.0};
  double yVariance{0.0};
  double headingVariance{0.0};
};

enum class PositionAxis { X, Y };

class Ekf {
 public:
  explicit Ekf(EkfConfig config) noexcept;

  void reset(EstimatorPose pose) noexcept;
  bool predict(LocalMotion motion) noexcept;
  bool updateImuHeading(double headingRadians) noexcept;
  bool updateGpsPosition(double xInches, double yInches,
                         double maximumNis) noexcept;
  bool updateGpsHeading(double headingRadians, double maximumNis,
                        bool enabled) noexcept;
  bool updateAxisPosition(PositionAxis axis, double positionInches,
                          double measurementVariance,
                          double maximumNis) noexcept;

  EstimatorPose pose() const noexcept;
  Matrix3 covariance() const noexcept;
  CovarianceDiagonal covarianceDiagonal() const noexcept;

 private:
  EkfConfig config_;
  EstimatorPose state_{};
  Matrix3 covariance_{};

  bool updateHeading(double headingRadians, double measurementVariance,
                     double maximumNis) noexcept;
};

}  // namespace aon::localization
