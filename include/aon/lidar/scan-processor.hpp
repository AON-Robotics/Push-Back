#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "aon/field/field-map.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/odometry/pose-estimator.hpp"

namespace aon::lidar {

struct PolarSample {
  double angleRadians = 0.0;
  double rangeInches = 0.0;
  bool valid = false;
};

struct LidarScan {
  static constexpr std::size_t kMaximumSamples = 720;
  std::array<PolarSample, kMaximumSamples> samples{};
  std::size_t sampleCount = 0;
  std::uint32_t captureTimestampMs = 0;
};

struct LidarMount {
  double xRightInches = 0.0;
  double yForwardInches = 0.0;
  double yawRadians = 0.0;
};

struct ScanContext {
  LidarMount mount;
  localization::EstimatorPose predictedPose;
};

enum class WallAxis : std::uint8_t { X, Y };

struct WallObservation {
  WallAxis axis = WallAxis::X;
  double positionInches = 0.0;
  double variance = 0.0;
  double meanResidualInches = 0.0;
  std::size_t support = 0;
  std::uint32_t captureTimestampMs = 0;
};

struct ScanProcessorConfig {
  double minimumRangeInches = 0.0;
  double maximumRangeInches = 0.0;
  double wallAssociationDistanceInches = 0.0;
  std::size_t minimumWallSupport = 0;
  double clusterDistanceInches = 0.0;
  std::size_t minimumClusterPoints = 0;
  double baseWallVariance = 0.0;
};

enum class ScanStatus : std::uint8_t {
  Accepted,
  InvalidConfig,
  InvalidContext,
  InvalidScan,
};

struct ScanResult {
  static constexpr std::size_t kMaximumWallObservations = 2;
  ScanStatus status = ScanStatus::InvalidScan;
  std::array<WallObservation, kMaximumWallObservations> wallObservations{};
  std::size_t wallObservationCount = 0;
  std::array<navigation::ObstacleDetection,
             navigation::DynamicObstacleMap::kMaximumObstacles>
      obstacles{};
  std::size_t obstacleCount = 0;
  std::size_t rejectedSampleCount = 0;
};

class ScanProcessor {
 public:
  explicit ScanProcessor(ScanProcessorConfig config) noexcept;

  [[nodiscard]] ScanResult process(const LidarScan& scan,
                                   const ScanContext& context,
                                   const field::FieldMap& field) const noexcept;

 private:
  [[nodiscard]] bool validConfig() const noexcept;
  ScanProcessorConfig config_;
};

}  // namespace aon::lidar
