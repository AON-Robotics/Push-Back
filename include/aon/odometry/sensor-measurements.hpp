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

}  // namespace aon::localization
