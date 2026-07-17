#include "aon/auton/fallback-geometry.hpp"

#include <cmath>

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

}  // namespace aon::auton
