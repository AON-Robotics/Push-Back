#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/time/monotonic.hpp"

#include <cmath>
#include <limits>

namespace aon::navigation {
namespace {

double distanceSquared(field::Point2 first, field::Point2 second) noexcept {
  const double dx = first.xInches - second.xInches;
  const double dy = first.yInches - second.yInches;
  return dx * dx + dy * dy;
}

}  // namespace

DynamicObstacleMap::DynamicObstacleMap(DynamicObstacleConfig config) noexcept
    : config_(config) {}

bool DynamicObstacleMap::valid(
    const ObstacleDetection& detection) const noexcept {
  if (!std::isfinite(config_.associationDistanceInches) ||
      config_.associationDistanceInches < 0.0 || config_.lifetimeMs == 0 ||
      !std::isfinite(config_.velocitySmoothing) ||
      config_.velocitySmoothing < 0.0 || config_.velocitySmoothing > 1.0) {
    return false;
  }
  if (!field::isFinite(detection.center) ||
      !std::isfinite(detection.radiusInches) ||
      !std::isfinite(detection.halfWidthInches) ||
      !std::isfinite(detection.halfHeightInches) ||
      !std::isfinite(detection.headingRadians) ||
      !std::isfinite(detection.confidence) || detection.confidence < 0.0 ||
      detection.confidence > 1.0) {
    return false;
  }
  if (detection.shape == ObstacleShape::Circle) {
    return detection.radiusInches > 0.0;
  }
  return detection.halfWidthInches > 0.0 &&
         detection.halfHeightInches > 0.0;
}

std::size_t DynamicObstacleMap::associationFor(
    const ObstacleDetection& detection) const noexcept {
  std::size_t best = size_;
  double bestDistance = config_.associationDistanceInches *
                        config_.associationDistanceInches;
  for (std::size_t index = 0; index < size_; ++index) {
    if (obstacles_[index].shape != detection.shape) {
      continue;
    }
    const double candidate =
        distanceSquared(obstacles_[index].center, detection.center);
    if (candidate <= bestDistance) {
      best = index;
      bestDistance = candidate;
    }
  }
  return best;
}

std::size_t DynamicObstacleMap::replacementIndex() const noexcept {
  std::size_t replacement = 0;
  for (std::size_t index = 1; index < size_; ++index) {
    const auto& candidate = obstacles_[index];
    const auto& current = obstacles_[replacement];
    if (candidate.confidence < current.confidence ||
        (candidate.confidence == current.confidence &&
         time::strictlyAfter(current.lastObservedMs,
                             candidate.lastObservedMs))) {
      replacement = index;
    }
  }
  return replacement;
}

ObstacleUpdateResult DynamicObstacleMap::update(
    const ObstacleDetection& detection) noexcept {
  if (!valid(detection)) return ObstacleUpdateResult::Invalid;

  const std::size_t associated = associationFor(detection);
  if (associated < size_) {
    DynamicObstacle& obstacle = obstacles_[associated];
    if (!time::strictlyAfter(detection.timestampMs,
                             obstacle.lastObservedMs)) {
      return ObstacleUpdateResult::OutOfOrder;
    }
    const double elapsedSeconds =
        static_cast<double>(time::elapsed(detection.timestampMs,
                                          obstacle.lastObservedMs)) /
        1000.0;
    const double measuredX =
        (detection.center.xInches - obstacle.center.xInches) / elapsedSeconds;
    const double measuredY =
        (detection.center.yInches - obstacle.center.yInches) / elapsedSeconds;
    const double blend = config_.velocitySmoothing;
    obstacle.velocityXInchesPerSecond =
        (1.0 - blend) * obstacle.velocityXInchesPerSecond + blend * measuredX;
    obstacle.velocityYInchesPerSecond =
        (1.0 - blend) * obstacle.velocityYInchesPerSecond + blend * measuredY;
    obstacle.center = detection.center;
    obstacle.radiusInches = detection.radiusInches;
    obstacle.halfWidthInches = detection.halfWidthInches;
    obstacle.halfHeightInches = detection.halfHeightInches;
    obstacle.headingRadians = detection.headingRadians;
    obstacle.confidence = detection.confidence;
    obstacle.lastObservedMs = detection.timestampMs;
    obstacle.expiresAtMs = detection.timestampMs + config_.lifetimeMs;
    return ObstacleUpdateResult::Updated;
  }

  std::size_t target = size_;
  ObstacleUpdateResult result = ObstacleUpdateResult::Inserted;
  if (size_ == obstacles_.size()) {
    target = replacementIndex();
    if (detection.confidence <= obstacles_[target].confidence) {
      return ObstacleUpdateResult::CapacityRejected;
    }
    result = ObstacleUpdateResult::Replaced;
  } else {
    ++size_;
  }
  obstacles_[target] = {
      nextTrackId_++, detection.shape, detection.center,
      detection.radiusInches, detection.halfWidthInches,
      detection.halfHeightInches, detection.headingRadians,
      0.0, 0.0, detection.confidence, detection.timestampMs,
      detection.timestampMs + config_.lifetimeMs};
  if (nextTrackId_ == 0) nextTrackId_ = 1;
  return result;
}

void DynamicObstacleMap::expire(std::uint32_t nowMs) noexcept {
  std::size_t write = 0;
  for (std::size_t read = 0; read < size_; ++read) {
    if (time::beforeDeadline(nowMs, obstacles_[read].expiresAtMs)) {
      if (write != read) obstacles_[write] = obstacles_[read];
      ++write;
    }
  }
  size_ = write;
}

void DynamicObstacleMap::clear() noexcept { size_ = 0; }

std::size_t DynamicObstacleMap::size() const noexcept { return size_; }

const DynamicObstacle* DynamicObstacleMap::at(std::size_t index) const noexcept {
  return index < size_ ? &obstacles_[index] : nullptr;
}

}  // namespace aon::navigation
