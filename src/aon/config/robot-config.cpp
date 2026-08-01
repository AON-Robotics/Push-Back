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
          {
              MOTOR_TO_DRIVE_RATIO,
              4.0,
              1.2,
              18,
              60,
              100,
              250,
              {3, 2, 300, 15.0, 0.02, 8.0, 45.0},
              false,
              false,
          },
      },
      false,  // shadowPlaybackAuthorized
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
          {
              MOTOR_TO_DRIVE_RATIO,
              4.0,
              1.2,
              18,
              60,
              100,
              250,
              {3, 2, 300, 15.0, 0.02, 8.0, 45.0},
              false,
              false,
          },
      },
      true,  // shadowPlaybackAuthorized
  };
#endif
  return config;
}

}  // namespace aon::config
