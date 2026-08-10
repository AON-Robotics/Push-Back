#pragma once

#include <atomic>
#include <cstdint>
#include <optional>

#include "aon/config/localization-config.hpp"
#include "aon/math/pose.hpp"
#include "aon/odometry/diagnostics.hpp"
#include "aon/odometry/ekf.hpp"
#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"
#include "aon/localization/velocity-estimator.hpp"
#include "aon/tools/vector.hpp"
#include "api.h"

namespace aon {

/** @brief Owns localization sensors and publishes coherent raw/fused poses. */
class Odometry {
 public:
  Odometry(const config::LocalizationConfig& config, std::int8_t leftPort,
           std::int8_t rightPort, std::int8_t backPort,
           std::int8_t imuPort);
  Odometry(const Odometry&) = delete;
  Odometry& operator=(const Odometry&) = delete;
  Odometry(Odometry&&) = delete;
  Odometry& operator=(Odometry&&) = delete;

  /** @brief Returns fused inches/degrees, or all-NaN on lock timeout. */
  Pose getPose();
  /** @brief Returns raw inches/degrees, or all-NaN on lock timeout. */
  Pose rawOdometryPose();
  double getX();
  double getY();
  double getDegrees();
  Vector getPosition();
  /** @brief Returns diagnostics including two-millisecond lock timeouts. */
  localization::LocalizationDiagnostics getDiagnostics();

  /** @brief Resets inches/degrees, returning false without a partial reset. */
  bool resetPose(double x, double y, double thetaDegrees);
  /** @brief Samples sensors and attempts one bounded estimator update. */
  void update();

  /** @brief Performs boot-only IMU calibration with a millisecond timeout. */
  bool calibrateImu(std::uint32_t timeoutMs = 3000U);
  /** @brief Runs deterministic updates using the configured millisecond period. */
  void runLocalizationLoop();
  /** @brief Runs updates and publishes fused poses without holding state locks. */
  void runLocalizationLoop(void (*publisher)(const Pose&));

  /** @brief Borrows the owned left tracker; ownership is not transferred. */
  pros::Rotation& leftTrackingSensor() noexcept;
  /** @brief Borrows the owned right tracker; ownership is not transferred. */
  pros::Rotation& rightTrackingSensor() noexcept;
  /** @brief Borrows the owned lateral tracker; ownership is not transferred. */
  pros::Rotation& backTrackingSensor() noexcept;
  /** @brief Borrows the owned IMU; ownership is not transferred. */
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
  localization::GpsFreshnessTracker gpsFreshness_;
  localization::VelocityEstimator velocityEstimator_{{0.35, 0.001, 0.2}};
  localization::WheelDistances wheelBaselines_{};
  double imuFieldOffsetRadians_{0.0};
  bool imuFieldOffsetValid_{false};
  std::uint32_t lastUpdateMs_{0U};
  std::uint32_t generation_{0U};
  PublishedSnapshot published_{};
  pros::Mutex publicationMutex_;
  pros::Mutex snapshotMutex_;
  std::atomic<std::uint32_t> publicationLockTimeouts_{0U};
  std::atomic<std::uint32_t> snapshotLockTimeouts_{0U};

  void recordDeadlineMiss();
  void publishCurrent(void (*publisher)(const Pose&));
};

}  // namespace aon
