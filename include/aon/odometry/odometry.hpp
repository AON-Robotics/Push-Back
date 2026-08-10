#pragma once

#include <cstdint>
#include <optional>

#include "aon/config/robot-config.hpp"
#include "aon/math/pose.hpp"
#include "aon/odometry/diagnostics.hpp"
#include "aon/odometry/ekf.hpp"
#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"
#include "aon/localization/velocity-estimator.hpp"
#include "aon/tools/vector.hpp"
#include "api.h"

namespace aon {

class Odometry {
 public:
  Odometry(const config::LocalizationConfig& config, std::int8_t leftPort,
           std::int8_t rightPort, std::int8_t backPort,
           std::int8_t imuPort);
  Odometry(const Odometry&) = delete;
  Odometry& operator=(const Odometry&) = delete;

  Pose getPose();
  Pose rawOdometryPose();
  double getX();
  double getY();
  double getDegrees();
  Vector getPosition();
  localization::LocalizationDiagnostics getDiagnostics();

  void resetPose(double x, double y, double thetaDegrees);
  void update();

  // IMU calibration is a boot operation. Runtime pose resets only change the
  // field offset and therefore never impose a multi-second autonomous pause.
  bool calibrateImu(std::uint32_t timeoutMs = 3000U);
  void runLocalizationLoop();
  void runLocalizationLoop(void (*publisher)(const Pose&));

  // Legacy motion code still needs direct cumulative sensor travel. These
  // references do not transfer ownership and will also serve the LemLib adapter.
  pros::Rotation& leftTrackingSensor() noexcept;
  pros::Rotation& rightTrackingSensor() noexcept;
  pros::Rotation& backTrackingSensor() noexcept;
  pros::Imu& imuSensor() noexcept;

 private:
  struct PublishedSnapshot {
    Pose rawPose{};
    Pose fusedPose{};
    localization::LocalizationDiagnostics diagnostics{};
  };

  config::LocalizationConfig config_;
  double distancePerCentidegree_;
  pros::Rotation encoderLeft_;
  pros::Rotation encoderRight_;
  pros::Rotation encoderBack_;
  pros::Imu imu_;
  std::optional<pros::Gps> gps_;

  localization::EstimatorPose rawPose_{};
  localization::Ekf ekf_;
  localization::GpsGate gpsGate_;
  localization::VelocityEstimator velocityEstimator_{{0.35, 0.001, 0.2}};
  localization::WheelDistances wheelBaselines_{};
  double imuFieldOffsetRadians_{0.0};
  bool imuFieldOffsetValid_{false};
  std::uint32_t lastUpdateMs_{0U};
  std::uint32_t generation_{0U};
  PublishedSnapshot published_{};
  pros::Mutex snapshotMutex_;

  void recordDeadlineMiss();
};

}  // namespace aon
