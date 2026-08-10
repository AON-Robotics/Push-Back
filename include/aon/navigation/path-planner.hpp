#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "aon/field/field-map.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"

namespace aon::navigation {

struct Path {
  static constexpr std::size_t kMaximumPoints = 32;
  std::array<field::Point2, kMaximumPoints> points{};
  std::size_t size = 0;
};

enum class PlanStatus : std::uint8_t {
  Success,
  InvalidRequest,
  StartBlocked,
  GoalBlocked,
  Unreachable,
  CapacityExceeded,
};

struct PlanResult {
  PlanStatus status = PlanStatus::InvalidRequest;
  Path path;
  double costInches = 0.0;
};

struct PathPlannerConfig {
  double safetyMarginInches = 1.0;
};

class PathPlanner {
 public:
  explicit PathPlanner(PathPlannerConfig config) noexcept;

  /**
   * Plans without allocation by reusing owned bounded scratch storage.
   * Calls on the same planner instance must not overlap.
   */
  [[nodiscard]] PlanResult plan(
      field::Point2 start, field::Point2 goal, double robotRadiusInches,
      const field::FieldMap& field,
      const DynamicObstacleMap& obstacles) noexcept;

  [[nodiscard]] bool pathHasClearance(
      const Path& path, double robotRadiusInches,
      const field::FieldMap& field,
      const DynamicObstacleMap& obstacles) const noexcept;

  /** Checks clearance without constructing the planner scratch workspace. */
  [[nodiscard]] static bool pathHasClearance(
      const Path& path, double robotRadiusInches,
      double safetyMarginInches, const field::FieldMap& field,
      const DynamicObstacleMap& obstacles) noexcept;

 private:
  static constexpr std::size_t kMaximumNodes =
      2 + DynamicObstacleMap::kMaximumObstacles * 4;

  struct Workspace {
    std::array<field::Point2, kMaximumNodes> nodes{};
    std::array<double, kMaximumNodes> costs{};
    std::array<int, kMaximumNodes> previous{};
    std::array<bool, kMaximumNodes> visited{};
    std::array<field::Point2, kMaximumNodes> reverse{};
  };

  [[nodiscard]] static bool pointBlocked(
      field::Point2 point, double clearanceInches,
      const DynamicObstacleMap& obstacles) noexcept;
  [[nodiscard]] static bool segmentVisible(
      const field::Segment& segment, double clearanceInches,
      const field::FieldMap& field,
      const DynamicObstacleMap& obstacles) noexcept;

  PathPlannerConfig config_;
  Workspace workspace_{};
};

}  // namespace aon::navigation
