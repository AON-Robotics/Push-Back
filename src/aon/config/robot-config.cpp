#include "aon/config/robot-config.hpp"

#include "aon/constants.hpp"

namespace aon::config {

const RobotConfig& activeRobotConfig() {
#if USING_BIG_ROBOT
  static const RobotConfig config{
      RobotIdentity::Big,
      {
          {{{12, -13, -18, 19}}, {{-1, 2, 3, -4}}},
          {5, -6, 7, 14, false, false, false},
          DRIVE_WHEEL_DIAMETER,
          TRACKING_WHEEL_DIAMETER,
          DRIVE_WIDTH,
          MAX_RPM * MOTOR_TO_DRIVE_RATIO,
          8.0,
          5.0,
          8.0,
          -DISTANCE_LEFT_TRACKING_WHEEL_CENTER,
          DISTANCE_RIGHT_TRACKING_WHEEL_CENTER,
          -DISTANCE_BACK_TRACKING_WHEEL_CENTER,
      },
  };
#else
  static const RobotConfig config{
      RobotIdentity::Small,
      {
          {{{11, -12, 13, -14}}, {{1, -2, 3, -4}}},
          {19, 18, 5, 16, false, true, false},
          DRIVE_WHEEL_DIAMETER,
          TRACKING_WHEEL_DIAMETER,
          DRIVE_WIDTH,
          MAX_RPM * MOTOR_TO_DRIVE_RATIO,
          8.0,
          5.0,
          8.0,
          -DISTANCE_LEFT_TRACKING_WHEEL_CENTER,
          DISTANCE_RIGHT_TRACKING_WHEEL_CENTER,
          -DISTANCE_BACK_TRACKING_WHEEL_CENTER,
      },
  };
#endif
  return config;
}

}  // namespace aon::config
