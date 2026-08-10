#pragma once

#include <cstdint>

#include "aon/odometry/ekf.hpp"
#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"

namespace aon::config {

/** @brief GPS installation, validation, and gated heading fusion values. */
struct GpsHardwareConfig {
  bool enabled{false};
  std::int8_t port{0};
  double xOffsetMeters{0.0};
  double yOffsetMeters{0.0};
  double headingOffsetDegrees{0.0};
  bool headingUpdateEnabled{false};
  localization::GpsValidationConfig validation{};
};

/** @brief Fixed localization values and physical authorization gates. */
struct LocalizationConfig {
  localization::TrackingGeometry geometry{};
  double trackingWheelDiameterInches{0.0};
  std::uint32_t loopPeriodMs{0U};
  localization::EkfConfig ekf{};
  GpsHardwareConfig gps{};
  bool fusedLemLibAuthorized{false};
  bool fusedNavigationAuthorized{false};
};

}  // namespace aon::config
