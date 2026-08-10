#pragma once

#include <cstdint>

#include "aon/odometry/ekf.hpp"
#include "aon/odometry/sensor-measurements.hpp"

namespace aon::localization {

struct LocalizationDiagnostics {
  EstimatorPose rawPose{};
  EstimatorPose fusedPose{};
  WheelDistances wheelDistances{};
  ImuMeasurement imu{};
  GpsMeasurement gps{};
  bool gpsPositionAccepted{false};
  bool gpsHeadingAccepted{false};
  GpsRejectionReason gpsRejectionReason{GpsRejectionReason::None};
  CovarianceDiagonal covariance{};
  double dtSeconds{0.0};
  std::uint32_t executionMicroseconds{0U};
  std::uint32_t deadlineMisses{0U};
  std::uint32_t wheelSensorErrors{0U};
  std::uint32_t imuSensorErrors{0U};
  std::uint32_t gpsSensorErrors{0U};
  std::uint32_t numericalRejections{0U};
  std::uint32_t resetCount{0U};
};

}  // namespace aon::localization
