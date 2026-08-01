#pragma once

#include "aon/auton/red-six-block-generated.hpp"

namespace aon::auton {

struct RedSixPose {
  double x;
  double y;
  double heading;
};

/**
 * @brief Coordinate and timing contract for the red six-block routine.
 *
 * The local origin is the robot tracking center at its repeatable red-side
 * starting placement. Heading zero faces forward along the field's +Y axis;
 * +X points to the robot's right and positive headings turn clockwise.
 */
struct RedSixBlock {
  static constexpr const char* name = "RED 6-BLOCK HYBRID";
  static constexpr RedSixPose start{0.0, 0.0, 0.0};
  static constexpr RedSixPose loaderStage{0.0, 24.0, 30.0};
  static constexpr RedSixPose loaderContact{4.0, 31.0, 86.0};
  static constexpr RedSixPose reverseClearance{-5.0, 31.0, 90.0};
  static constexpr RedSixPose reverseAlignment{-9.0, 31.0, 171.0};
  static constexpr RedSixPose goalStage{
      RedSixBlockGenerated::goalStageX,
      RedSixBlockGenerated::goalStageY,
      RedSixBlockGenerated::goalStageHeading};
  static constexpr RedSixPose goalContact{-8.0, 25.0, 171.0};

  static constexpr float loaderLookahead = 7.0F;
  static constexpr float goalLookahead = 3.0F;
  static constexpr int loaderPathTimeoutMs = 2000;
  static constexpr int loaderContactTimeoutMs = 900;
  static constexpr int collectTimeoutMs = 4000;
  static constexpr int reverseClearanceTimeoutMs = 800;
  static constexpr int reverseAlignmentTimeoutMs = 1000;
  static constexpr int goalPathTimeoutMs = 800;
  static constexpr int goalContactTimeoutMs = 800;
  static constexpr int scoreTimeoutMs = 2700;
  static constexpr int autonomousLimitMs = 15000;
  static constexpr int requiredMarginMs = 1000;
};

}  // namespace aon::auton
