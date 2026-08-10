#include "aon/navigation/path-planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::navigation {
namespace {

constexpr double kCornerEpsilonInches = 1e-3;

double distance(field::Point2 first, field::Point2 second) noexcept {
  return std::hypot(second.xInches - first.xInches,
                    second.yInches - first.yInches);
}

double pointSegmentDistance(field::Point2 point,
                            const field::Segment& segment) noexcept {
  const double dx = segment.end.xInches - segment.start.xInches;
  const double dy = segment.end.yInches - segment.start.yInches;
  const double lengthSquared = dx * dx + dy * dy;
  if (lengthSquared <= 0.0) return distance(point, segment.start);
  const double projection = std::clamp(
      ((point.xInches - segment.start.xInches) * dx +
       (point.yInches - segment.start.yInches) * dy) /
          lengthSquared,
      0.0, 1.0);
  return distance(point,
                  {segment.start.xInches + projection * dx,
                   segment.start.yInches + projection * dy});
}

void obstacleExtents(const DynamicObstacle& obstacle, double clearance,
                     double& halfWidth, double& halfHeight) noexcept {
  if (obstacle.shape == ObstacleShape::Circle) {
    halfWidth = obstacle.radiusInches + clearance;
    halfHeight = halfWidth;
    return;
  }
  const double cosine = std::abs(std::cos(obstacle.headingRadians));
  const double sine = std::abs(std::sin(obstacle.headingRadians));
  halfWidth = cosine * obstacle.halfWidthInches +
              sine * obstacle.halfHeightInches + clearance;
  halfHeight = sine * obstacle.halfWidthInches +
               cosine * obstacle.halfHeightInches + clearance;
}

bool pointInsideObstacle(field::Point2 point,
                         const DynamicObstacle& obstacle,
                         double clearance) noexcept {
  if (obstacle.shape == ObstacleShape::Circle) {
    return distance(point, obstacle.center) <=
           obstacle.radiusInches + clearance;
  }
  double halfWidth = 0.0;
  double halfHeight = 0.0;
  obstacleExtents(obstacle, clearance, halfWidth, halfHeight);
  return std::abs(point.xInches - obstacle.center.xInches) <= halfWidth &&
         std::abs(point.yInches - obstacle.center.yInches) <= halfHeight;
}

bool segmentIntersectsAabb(const field::Segment& segment, field::Point2 center,
                           double halfWidth, double halfHeight) noexcept {
  double minimum = 0.0;
  double maximum = 1.0;
  const double starts[2] = {segment.start.xInches - center.xInches,
                            segment.start.yInches - center.yInches};
  const double deltas[2] = {
      segment.end.xInches - segment.start.xInches,
      segment.end.yInches - segment.start.yInches};
  const double extents[2] = {halfWidth, halfHeight};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    if (std::abs(deltas[axis]) < 1e-12) {
      if (std::abs(starts[axis]) <= extents[axis]) continue;
      return false;
    }
    double first = (-extents[axis] - starts[axis]) / deltas[axis];
    double second = (extents[axis] - starts[axis]) / deltas[axis];
    if (first > second) std::swap(first, second);
    minimum = std::max(minimum, first);
    maximum = std::min(maximum, second);
    if (minimum > maximum) return false;
  }
  return true;
}

bool segmentIntersectsObstacle(const field::Segment& segment,
                               const DynamicObstacle& obstacle,
                               double clearance) noexcept {
  if (obstacle.shape == ObstacleShape::Circle) {
    return pointSegmentDistance(obstacle.center, segment) <=
           obstacle.radiusInches + clearance;
  }
  double halfWidth = 0.0;
  double halfHeight = 0.0;
  obstacleExtents(obstacle, clearance, halfWidth, halfHeight);
  return segmentIntersectsAabb(segment, obstacle.center, halfWidth,
                               halfHeight);
}

}  // namespace

PathPlanner::PathPlanner(PathPlannerConfig config) noexcept : config_(config) {}

bool PathPlanner::pointBlocked(
    field::Point2 point, double clearanceInches,
    const DynamicObstacleMap& obstacles) const noexcept {
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const DynamicObstacle* obstacle = obstacles.at(index);
    if (obstacle != nullptr &&
        pointInsideObstacle(point, *obstacle, clearanceInches)) {
      return true;
    }
  }
  return false;
}

bool PathPlanner::segmentVisible(
    const field::Segment& segment, double clearanceInches,
    const field::FieldMap& field,
    const DynamicObstacleMap& obstacles) const noexcept {
  if (!field.segmentHasClearance(segment, clearanceInches)) return false;
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const DynamicObstacle* obstacle = obstacles.at(index);
    if (obstacle != nullptr &&
        segmentIntersectsObstacle(segment, *obstacle, clearanceInches)) {
      return false;
    }
  }
  return true;
}

PlanResult PathPlanner::plan(field::Point2 start, field::Point2 goal,
                             double robotRadiusInches,
                             const field::FieldMap& field,
                             const DynamicObstacleMap& obstacles) const noexcept {
  PlanResult result;
  if (!field::isFinite(start) || !field::isFinite(goal) ||
      !std::isfinite(robotRadiusInches) || robotRadiusInches < 0.0 ||
      !std::isfinite(config_.safetyMarginInches) ||
      config_.safetyMarginInches < 0.0 ||
      field.validate() != field::FieldMapIssue::None) {
    return result;
  }
  const double clearance =
      robotRadiusInches + config_.safetyMarginInches;
  if (!field.contains(start, clearance) ||
      pointBlocked(start, clearance, obstacles)) {
    result.status = PlanStatus::StartBlocked;
    return result;
  }
  if (!field.contains(goal, clearance) ||
      pointBlocked(goal, clearance, obstacles)) {
    result.status = PlanStatus::GoalBlocked;
    return result;
  }
  if (distance(start, goal) <= 1e-9) {
    result.status = PlanStatus::Success;
    result.path.points[0] = start;
    result.path.size = 1;
    return result;
  }

  std::array<field::Point2, kMaximumNodes> nodes{};
  std::size_t nodeCount = 2;
  nodes[0] = start;
  nodes[1] = goal;
  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const DynamicObstacle* obstacle = obstacles.at(index);
    if (obstacle == nullptr) continue;
    double halfWidth = 0.0;
    double halfHeight = 0.0;
    obstacleExtents(*obstacle, clearance + kCornerEpsilonInches,
                    halfWidth, halfHeight);
    const field::Point2 corners[4] = {
        {obstacle->center.xInches - halfWidth,
         obstacle->center.yInches - halfHeight},
        {obstacle->center.xInches - halfWidth,
         obstacle->center.yInches + halfHeight},
        {obstacle->center.xInches + halfWidth,
         obstacle->center.yInches - halfHeight},
        {obstacle->center.xInches + halfWidth,
         obstacle->center.yInches + halfHeight},
    };
    for (field::Point2 corner : corners) {
      if (nodeCount < nodes.size() && field.contains(corner, clearance) &&
          !pointBlocked(corner, clearance, obstacles)) {
        nodes[nodeCount++] = corner;
      }
    }
  }

  const double infinity = std::numeric_limits<double>::infinity();
  std::array<double, kMaximumNodes> costs{};
  std::array<int, kMaximumNodes> previous{};
  std::array<bool, kMaximumNodes> visited{};
  costs.fill(infinity);
  previous.fill(-1);
  costs[0] = 0.0;

  for (std::size_t iteration = 0; iteration < nodeCount; ++iteration) {
    std::size_t current = nodeCount;
    for (std::size_t index = 0; index < nodeCount; ++index) {
      if (!visited[index] &&
          (current == nodeCount || costs[index] < costs[current])) {
        current = index;
      }
    }
    if (current == nodeCount || !std::isfinite(costs[current])) break;
    if (current == 1) break;
    visited[current] = true;
    for (std::size_t neighbor = 0; neighbor < nodeCount; ++neighbor) {
      if (neighbor == current || visited[neighbor] ||
          !segmentVisible({nodes[current], nodes[neighbor]}, clearance,
                          field, obstacles)) {
        continue;
      }
      const double candidate =
          costs[current] + distance(nodes[current], nodes[neighbor]);
      if (candidate < costs[neighbor]) {
        costs[neighbor] = candidate;
        previous[neighbor] = static_cast<int>(current);
      }
    }
  }

  if (!std::isfinite(costs[1])) {
    result.status = PlanStatus::Unreachable;
    return result;
  }
  std::array<field::Point2, kMaximumNodes> reverse{};
  std::size_t reverseCount = 0;
  for (int current = 1; current >= 0;
       current = previous[static_cast<std::size_t>(current)]) {
    if (reverseCount == reverse.size()) {
      result.status = PlanStatus::CapacityExceeded;
      return result;
    }
    reverse[reverseCount++] = nodes[static_cast<std::size_t>(current)];
    if (current == 0) break;
    if (previous[static_cast<std::size_t>(current)] < 0) {
      result.status = PlanStatus::Unreachable;
      return result;
    }
  }
  if (reverseCount > result.path.points.size()) {
    result.status = PlanStatus::CapacityExceeded;
    return result;
  }
  result.path.size = reverseCount;
  for (std::size_t index = 0; index < reverseCount; ++index) {
    result.path.points[index] = reverse[reverseCount - index - 1];
  }
  result.status = PlanStatus::Success;
  result.costInches = costs[1];
  return result;
}

bool PathPlanner::pathHasClearance(
    const Path& path, double robotRadiusInches,
    const field::FieldMap& field,
    const DynamicObstacleMap& obstacles) const noexcept {
  if (path.size == 0 || path.size > path.points.size() ||
      !std::isfinite(robotRadiusInches) || robotRadiusInches < 0.0) {
    return false;
  }
  const double clearance =
      robotRadiusInches + config_.safetyMarginInches;
  if (!field.contains(path.points[0], clearance) ||
      pointBlocked(path.points[0], clearance, obstacles)) {
    return false;
  }
  for (std::size_t index = 1; index < path.size; ++index) {
    if (!segmentVisible({path.points[index - 1], path.points[index]},
                        clearance, field, obstacles)) {
      return false;
    }
  }
  return true;
}

}  // namespace aon::navigation
