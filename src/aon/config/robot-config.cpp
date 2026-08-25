#include "aon/config/robot-config.hpp"

#include "aon/constants.hpp"

namespace aon::config {
namespace {

constexpr AuthorizationSnapshot smallRobotLocalizationTestAuthorizations()
    noexcept {
  auto authorizations = baselineAuthorizations(RobotIdentity::Small);
  // This authorizes the wheel-and-IMU EKF and its pose consumers for a
  // supervised test. GPS stays disabled because port and mounting geometry
  // have not been supplied for this robot.
  authorizations.fusedLemLib = true;
  authorizations.fusedNavigation = true;
  return authorizations;
}

const LocalizationConfig& localizationConfig(
    const AuthorizationSnapshot& authorizations) {
  // These geometry values preserve the existing LemLib configuration; they
  // remain nominal until the signed full-turn measurements are recorded.
  static const LocalizationConfig config{
      {-DISTANCE_LEFT_TRACKING_WHEEL_CENTER,
       DISTANCE_RIGHT_TRACKING_WHEEL_CENTER,
       -DISTANCE_BACK_TRACKING_WHEEL_CENTER},
      TRACKING_WHEEL_DIAMETER,
      10U,
      {
          4.0,
          aon::localization::radians(5.0) *
              aon::localization::radians(5.0),
          1e-6,
          1e-8,
          0.01,
          0.01,
          aon::localization::radians(2.0) *
              aon::localization::radians(2.0),
          4.0,
          aon::localization::radians(8.0) *
              aon::localization::radians(8.0),
          1e-12,
      },
      {
          authorizations.gpsHardware,
          0,
          0.0,
          0.0,
          0.0,
          authorizations.gpsHeadingFusion,
          {-72.0, 72.0, -72.0, 72.0, 6.0, 18.0,
           aon::localization::radians(45.0), 50U, 9.21, 6.63},
      },
      authorizations.fusedLemLib,
      authorizations.fusedNavigation,
  };
  return config;
}

}  // namespace

const RobotConfig& activeRobotConfig() {
#if USING_BIG_ROBOT
  constexpr const RobotHardwareMap& hardwareMap = bigRobotHardwareMap;
  constexpr auto authorizations =
      baselineAuthorizations(RobotIdentity::Big);
  const LocalizationConfig& localization =
      localizationConfig(authorizations);
  static const RobotConfig config{
      RobotIdentity::Big,
      {
          hardwareMap.drive,
          hardwareMap.lemlibTracking,
          DRIVE_WHEEL_DIAMETER,
          TRACKING_WHEEL_DIAMETER,
          DRIVE_WIDTH,
          MAX_RPM * MOTOR_TO_DRIVE_RATIO,
          8.0,
          5.0,
          8.0,
          static_cast<float>(localization.geometry.leftOffsetInches),
          static_cast<float>(localization.geometry.rightOffsetInches),
          static_cast<float>(localization.geometry.backOffsetInches),
          {
              MOTOR_TO_DRIVE_RATIO,
              4.0,
              1.2,
              18,
              60,
              100,
              250,
              {3, 2, 300, 15.0, 0.02, 8.0, 45.0},
              authorizations.automaticEncoderFallback,
              authorizations.forcedEncoderTesting,
          },
      },
      localization,
      authorizations.shadowPlayback,
      {authorizations.redSixBlock, authorizations.jerryIoPath},
  };
#else
  constexpr const RobotHardwareMap& hardwareMap = smallRobotHardwareMap;
  constexpr auto authorizations =
      smallRobotLocalizationTestAuthorizations();
  const LocalizationConfig& localization =
      localizationConfig(authorizations);
  static const RobotConfig config{
      RobotIdentity::Small,
      {
          hardwareMap.drive,
          hardwareMap.lemlibTracking,
          DRIVE_WHEEL_DIAMETER,
          TRACKING_WHEEL_DIAMETER,
          DRIVE_WIDTH,
          MAX_RPM * MOTOR_TO_DRIVE_RATIO,
          8.0,
          5.0,
          8.0,
          static_cast<float>(localization.geometry.leftOffsetInches),
          static_cast<float>(localization.geometry.rightOffsetInches),
          static_cast<float>(localization.geometry.backOffsetInches),
          {
              MOTOR_TO_DRIVE_RATIO,
              4.0,
              1.2,
              18,
              60,
              100,
              250,
              {3, 2, 300, 15.0, 0.02, 8.0, 45.0},
              authorizations.automaticEncoderFallback,
              authorizations.forcedEncoderTesting,
          },
      },
      localization,
      true,  // Supervised small-robot Shadow playback physical test only.
      {true, false},  // Supervised Red Six Block physical test only.
  };
#endif
  return config;
}

}  // namespace aon::config
