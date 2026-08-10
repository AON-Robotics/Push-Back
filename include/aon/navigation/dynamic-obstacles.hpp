#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "aon/field/geometry.hpp"

namespace aon::navigation {

enum class ObstacleShape : std::uint8_t { Circle, Rectangle };

struct ObstacleDetection {
  ObstacleShape shape = ObstacleShape::Circle;
  field::Point2 center;
  double radiusInches = 0.0;
  double halfWidthInches = 0.0;
  double halfHeightInches = 0.0;
  double headingRadians = 0.0;
  double confidence = 0.0;
  std::uint32_t timestampMs = 0;
};

struct DynamicObstacle {
  std::uint16_t trackId = 0;
  ObstacleShape shape = ObstacleShape::Circle;
  field::Point2 center;
  double radiusInches = 0.0;
  double halfWidthInches = 0.0;
  double halfHeightInches = 0.0;
  double headingRadians = 0.0;
  double velocityXInchesPerSecond = 0.0;
  double velocityYInchesPerSecond = 0.0;
  double confidence = 0.0;
  std::uint32_t lastObservedMs = 0;
  std::uint32_t expiresAtMs = 0;
};

struct DynamicObstacleConfig {
  double associationDistanceInches = 6.0;
  std::uint32_t lifetimeMs = 500;
  double velocitySmoothing = 0.5;
};

enum class ObstacleUpdateResult : std::uint8_t {
  Inserted,
  Updated,
  Replaced,
  CapacityRejected,
  Invalid,
  OutOfOrder,
};

class DynamicObstacleMap {
 public:
  static constexpr std::size_t kMaximumObstacles = 16;

  explicit DynamicObstacleMap(DynamicObstacleConfig config) noexcept;

  [[nodiscard]] ObstacleUpdateResult update(
      const ObstacleDetection& detection) noexcept;
  void expire(std::uint32_t nowMs) noexcept;
  void clear() noexcept;

  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] const DynamicObstacle* at(std::size_t index) const noexcept;

 private:
  [[nodiscard]] bool valid(const ObstacleDetection& detection) const noexcept;
  [[nodiscard]] std::size_t associationFor(
      const ObstacleDetection& detection) const noexcept;
  [[nodiscard]] std::size_t replacementIndex() const noexcept;

  DynamicObstacleConfig config_;
  std::array<DynamicObstacle, kMaximumObstacles> obstacles_{};
  std::size_t size_ = 0;
  std::uint16_t nextTrackId_ = 1;
};

}  // namespace aon::navigation
