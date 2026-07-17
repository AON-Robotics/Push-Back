#pragma once

#include <cstdint>

namespace aon::auton {

struct TrustedPose {
  double x;
  double y;
  double heading;
};

struct FallbackGeometry {
  double turnDegrees;
  double distanceInches;
  double finalTurnDegrees;
};

FallbackGeometry pointFallback(TrustedPose start, double targetX,
                               double targetY);
FallbackGeometry poseFallback(TrustedPose start, double targetX,
                              double targetY, double targetHeading);
FallbackGeometry withTravelDirection(FallbackGeometry geometry,
                                     bool forwards);
double headingFallback(double currentHeading, double targetHeading);
double motorDegreesForDistance(
    double distanceInches, double driveWheelDiameter,
    double wheelRevolutionsPerMotorRevolution);
int cappedFallbackOutput(int requested, int configuredPercent);
std::uint32_t fallbackBudget(std::uint32_t startedAt, std::uint32_t now,
                             std::uint32_t originalTimeout,
                             std::uint32_t transitionAllowance);

}  // namespace aon::auton
