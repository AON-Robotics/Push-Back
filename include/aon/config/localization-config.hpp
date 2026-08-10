#pragma once

#include <cstdint>

#include "aon/odometry/ekf.hpp"
#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"

namespace aon::config {

struct GpsHardwareConfig {
  bool enabled;
  std::int8_t port;
  double xOffsetMeters;
  double yOffsetMeters;
  double headingOffsetDegrees;
  bool headingUpdateEnabled;
  localization::GpsValidationConfig validation;
};

struct LocalizationConfig {
  localization::TrackingGeometry geometry;
  double trackingWheelDiameterInches;
  std::uint32_t loopPeriodMs;
  localization::EkfConfig ekf;
  GpsHardwareConfig gps;
  bool fusedLemLibAuthorized;
  bool fusedNavigationAuthorized;
};

}  // namespace aon::config
