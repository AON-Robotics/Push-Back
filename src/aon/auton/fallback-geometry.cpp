#include "aon/auton/fallback-geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::auton {
namespace {

constexpr double kPi = 3.14159265358979323846;

double targetBearing(TrustedPose start, double targetX, double targetY) {
  const double deltaX = targetX - start.x;
  const double deltaY = targetY - start.y;
  if (deltaX == 0.0 && deltaY == 0.0) return start.heading;
  return std::atan2(deltaX, deltaY) * 180.0 / kPi;
}

}  // namespace

double headingFallback(double currentHeading, double targetHeading) {
  return std::remainder(targetHeading - currentHeading, 360.0);
}

FallbackGeometry pointFallback(TrustedPose start, double targetX,
                               double targetY) {
  const double bearing = targetBearing(start, targetX, targetY);
  return {
      headingFallback(start.heading, bearing),
      std::hypot(targetX - start.x, targetY - start.y),
      0.0,
  };
}

FallbackGeometry poseFallback(TrustedPose start, double targetX,
                              double targetY, double targetHeading) {
  const double bearing = targetBearing(start, targetX, targetY);
  return {
      headingFallback(start.heading, bearing),
      std::hypot(targetX - start.x, targetY - start.y),
      headingFallback(bearing, targetHeading),
  };
}

double motorDegreesForDistance(
    double distanceInches, double driveWheelDiameter,
    double wheelRevolutionsPerMotorRevolution) {
  return distanceInches /
         (kPi * driveWheelDiameter * wheelRevolutionsPerMotorRevolution) *
         360.0;
}

int cappedFallbackOutput(int requested, int configuredPercent) {
  const int percent = std::clamp(configuredPercent, 0, 100);
  const long long scaled =
      static_cast<long long>(requested) * static_cast<long long>(percent) / 100;
  return static_cast<int>(std::clamp(
      scaled, static_cast<long long>(std::numeric_limits<int>::min()),
      static_cast<long long>(std::numeric_limits<int>::max())));
}

std::uint32_t fallbackBudget(std::uint32_t startedAt, std::uint32_t now,
                             std::uint32_t originalTimeout,
                             std::uint32_t transitionAllowance) {
  const std::uint32_t elapsed = now - startedAt;
  const std::uint32_t remaining =
      elapsed >= originalTimeout ? 0 : originalTimeout - elapsed;
  const std::uint64_t budget = static_cast<std::uint64_t>(remaining) +
                               transitionAllowance;
  return static_cast<std::uint32_t>(std::min<std::uint64_t>(
      budget, std::numeric_limits<std::uint32_t>::max()));
}

}  // namespace aon::auton
