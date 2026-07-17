#pragma once

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
double headingFallback(double currentHeading, double targetHeading);
double motorDegreesForDistance(
    double distanceInches, double driveWheelDiameter,
    double wheelRevolutionsPerMotorRevolution);

}  // namespace aon::auton
