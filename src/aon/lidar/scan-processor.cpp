#include "aon/lidar/scan-processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::lidar {
namespace {

struct WallAccumulator {
  WallAxis axis = WallAxis::X;
  double wallCoordinate = 0.0;
  double residualSum = 0.0;
  double residualSquaredSum = 0.0;
  std::size_t support = 0;
};

struct ClusterAccumulator {
  field::Point2 previous{};
  double sumX = 0.0;
  double sumY = 0.0;
  double minimumX = 0.0;
  double maximumX = 0.0;
  double minimumY = 0.0;
  double maximumY = 0.0;
  std::size_t count = 0;
};

field::Point2 rotate(field::Point2 point, double clockwiseRadians) noexcept {
  const double cosine = std::cos(clockwiseRadians);
  const double sine = std::sin(clockwiseRadians);
  return {point.xInches * cosine + point.yInches * sine,
          -point.xInches * sine + point.yInches * cosine};
}

bool finitePose(const localization::EstimatorPose& pose) noexcept {
  return std::isfinite(pose.xInches) && std::isfinite(pose.yInches) &&
         std::isfinite(pose.headingRadians);
}

void startCluster(ClusterAccumulator& cluster, field::Point2 point) noexcept {
  cluster = {point, point.xInches, point.yInches, point.xInches,
             point.xInches, point.yInches, point.yInches, 1};
}

void addToCluster(ClusterAccumulator& cluster, field::Point2 point) noexcept {
  cluster.previous = point;
  cluster.sumX += point.xInches;
  cluster.sumY += point.yInches;
  cluster.minimumX = std::min(cluster.minimumX, point.xInches);
  cluster.maximumX = std::max(cluster.maximumX, point.xInches);
  cluster.minimumY = std::min(cluster.minimumY, point.yInches);
  cluster.maximumY = std::max(cluster.maximumY, point.yInches);
  ++cluster.count;
}

void finishCluster(ClusterAccumulator& cluster,
                   const ScanProcessorConfig& config,
                   std::uint32_t timestampMs, ScanResult& result) noexcept {
  if (cluster.count >= config.minimumClusterPoints &&
      result.obstacleCount < result.obstacles.size()) {
    const double inverseCount = 1.0 / static_cast<double>(cluster.count);
    const field::Point2 center{cluster.sumX * inverseCount,
                              cluster.sumY * inverseCount};
    const double radius = std::max(
        0.5,
        0.5 * std::hypot(cluster.maximumX - cluster.minimumX,
                         cluster.maximumY - cluster.minimumY));
    result.obstacles[result.obstacleCount++] = {
        navigation::ObstacleShape::Circle, center, radius, 0.0, 0.0, 0.0,
        std::min(1.0, static_cast<double>(cluster.count) /
                          static_cast<double>(config.minimumClusterPoints * 2)),
        timestampMs};
  }
  cluster = {};
}

}  // namespace

ScanProcessor::ScanProcessor(ScanProcessorConfig config) noexcept
    : config_(config) {}

bool ScanProcessor::validConfig() const noexcept {
  return std::isfinite(config_.minimumRangeInches) &&
         config_.minimumRangeInches >= 0.0 &&
         std::isfinite(config_.maximumRangeInches) &&
         config_.maximumRangeInches > config_.minimumRangeInches &&
         std::isfinite(config_.wallAssociationDistanceInches) &&
         config_.wallAssociationDistanceInches > 0.0 &&
         config_.minimumWallSupport > 0 &&
         std::isfinite(config_.clusterDistanceInches) &&
         config_.clusterDistanceInches > 0.0 &&
         config_.minimumClusterPoints > 0 &&
         std::isfinite(config_.baseWallVariance) &&
         config_.baseWallVariance > 0.0;
}

ScanResult ScanProcessor::process(const LidarScan& scan,
                                  const ScanContext& context,
                                  const field::FieldMap& field) const noexcept {
  ScanResult result;
  if (!validConfig()) {
    result.status = ScanStatus::InvalidConfig;
    return result;
  }
  if (!std::isfinite(context.mount.xRightInches) ||
      !std::isfinite(context.mount.yForwardInches) ||
      !std::isfinite(context.mount.yawRadians) ||
      !finitePose(context.predictedPose) ||
      field.validate() != field::FieldMapIssue::None) {
    result.status = ScanStatus::InvalidContext;
    return result;
  }
  if (scan.sampleCount > scan.samples.size()) return result;

  const field::Bounds bounds = field.bounds();
  std::array<WallAccumulator, 4> walls{{
      {WallAxis::X, bounds.minimumXInches, 0.0, 0.0, 0},
      {WallAxis::X, bounds.maximumXInches, 0.0, 0.0, 0},
      {WallAxis::Y, bounds.minimumYInches, 0.0, 0.0, 0},
      {WallAxis::Y, bounds.maximumYInches, 0.0, 0.0, 0},
  }};
  ClusterAccumulator cluster;

  for (std::size_t index = 0; index < scan.sampleCount; ++index) {
    const PolarSample sample = scan.samples[index];
    if (!sample.valid || !std::isfinite(sample.angleRadians) ||
        !std::isfinite(sample.rangeInches) ||
        sample.rangeInches < config_.minimumRangeInches ||
        sample.rangeInches > config_.maximumRangeInches) {
      ++result.rejectedSampleCount;
      finishCluster(cluster, config_, scan.captureTimestampMs, result);
      continue;
    }

    field::Point2 lidarPoint{sample.rangeInches * std::sin(sample.angleRadians),
                             sample.rangeInches * std::cos(sample.angleRadians)};
    field::Point2 robotPoint = rotate(lidarPoint, context.mount.yawRadians);
    robotPoint.xInches += context.mount.xRightInches;
    robotPoint.yInches += context.mount.yForwardInches;
    field::Point2 fieldPoint =
        rotate(robotPoint, context.predictedPose.headingRadians);
    fieldPoint.xInches += context.predictedPose.xInches;
    fieldPoint.yInches += context.predictedPose.yInches;

    double distances[4] = {
        std::abs(fieldPoint.xInches - bounds.minimumXInches),
        std::abs(fieldPoint.xInches - bounds.maximumXInches),
        std::abs(fieldPoint.yInches - bounds.minimumYInches),
        std::abs(fieldPoint.yInches - bounds.maximumYInches),
    };
    const double wallTolerance = config_.wallAssociationDistanceInches;
    if (fieldPoint.yInches < bounds.minimumYInches - wallTolerance ||
        fieldPoint.yInches > bounds.maximumYInches + wallTolerance) {
      distances[0] = std::numeric_limits<double>::infinity();
      distances[1] = std::numeric_limits<double>::infinity();
    }
    if (fieldPoint.xInches < bounds.minimumXInches - wallTolerance ||
        fieldPoint.xInches > bounds.maximumXInches + wallTolerance) {
      distances[2] = std::numeric_limits<double>::infinity();
      distances[3] = std::numeric_limits<double>::infinity();
    }
    std::size_t closest = 0;
    for (std::size_t wall = 1; wall < walls.size(); ++wall) {
      if (distances[wall] < distances[closest]) closest = wall;
    }
    if (distances[closest] <= config_.wallAssociationDistanceInches) {
      const double coordinate = walls[closest].axis == WallAxis::X
                                    ? fieldPoint.xInches
                                    : fieldPoint.yInches;
      const double residual = coordinate - walls[closest].wallCoordinate;
      walls[closest].residualSum += residual;
      walls[closest].residualSquaredSum += residual * residual;
      ++walls[closest].support;
      finishCluster(cluster, config_, scan.captureTimestampMs, result);
      continue;
    }

    if (!field.contains(fieldPoint)) {
      ++result.rejectedSampleCount;
      finishCluster(cluster, config_, scan.captureTimestampMs, result);
      continue;
    }

    if (cluster.count == 0) {
      startCluster(cluster, fieldPoint);
    } else if (std::hypot(fieldPoint.xInches - cluster.previous.xInches,
                          fieldPoint.yInches - cluster.previous.yInches) <=
               config_.clusterDistanceInches) {
      addToCluster(cluster, fieldPoint);
    } else {
      finishCluster(cluster, config_, scan.captureTimestampMs, result);
      startCluster(cluster, fieldPoint);
    }
  }
  finishCluster(cluster, config_, scan.captureTimestampMs, result);

  for (WallAxis axis : {WallAxis::X, WallAxis::Y}) {
    const WallAccumulator* best = nullptr;
    for (const WallAccumulator& wall : walls) {
      if (wall.axis == axis && wall.support >= config_.minimumWallSupport &&
          (best == nullptr || wall.support > best->support)) {
        best = &wall;
      }
    }
    if (best == nullptr ||
        result.wallObservationCount == result.wallObservations.size()) {
      continue;
    }
    const double inverseSupport = 1.0 / static_cast<double>(best->support);
    const double mean = best->residualSum * inverseSupport;
    const double residualVariance = std::max(
        0.0, best->residualSquaredSum * inverseSupport - mean * mean);
    const double predictedAxis = axis == WallAxis::X
                                     ? context.predictedPose.xInches
                                     : context.predictedPose.yInches;
    result.wallObservations[result.wallObservationCount++] = {
        axis, predictedAxis - mean,
        config_.baseWallVariance + residualVariance * inverseSupport,
        mean, best->support, scan.captureTimestampMs};
  }

  result.status = ScanStatus::Accepted;
  return result;
}

}  // namespace aon::lidar
