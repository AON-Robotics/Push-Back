#include "aon/navigation/replanner.hpp"

#include <cmath>

namespace aon::navigation {

Replanner::Replanner(ReplannerConfig config) noexcept : config_(config) {}

bool Replanner::remainingRouteIsClear(
    const RouteState& route, const DynamicObstacleMap& obstacles,
    const field::FieldMap& field) const noexcept {
  if (route.path.size == 0 || route.path.size > route.path.points.size() ||
      route.nextPointIndex >= route.path.size) {
    return false;
  }
  const std::size_t first =
      route.nextPointIndex == 0 ? 0 : route.nextPointIndex - 1;
  Path remaining;
  for (std::size_t index = first; index < route.path.size; ++index) {
    if (remaining.size == remaining.points.size()) return false;
    remaining.points[remaining.size++] = route.path.points[index];
  }
  return PathPlanner::pathHasClearance(
      remaining, config_.robotRadiusInches, config_.safetyMarginInches,
      field, obstacles);
}

ReplanDecision Replanner::evaluate(
    const RouteState& route, const DynamicObstacleMap& obstacles,
    const field::FieldMap& field, std::uint32_t currentWorldRevision,
    std::uint32_t currentObjectiveRevision, std::uint32_t nowMs) noexcept {
  ReplanDecision decision{ReplanReason::None, currentWorldRevision,
                          currentObjectiveRevision};
  if (!std::isfinite(config_.robotRadiusInches) ||
      config_.robotRadiusInches < 0.0 ||
      !std::isfinite(config_.safetyMarginInches) ||
      config_.safetyMarginInches < 0.0 ||
      !std::isfinite(config_.poseCorrectionThresholdInches) ||
      config_.poseCorrectionThresholdInches < 0.0 || route.path.size == 0 ||
      route.path.size > route.path.points.size() ||
      route.nextPointIndex >= route.path.size) {
    decision.reason = ReplanReason::InvalidRoute;
  } else if (currentObjectiveRevision != route.objectiveRevision) {
    decision.reason = ReplanReason::GoalChanged;
  } else if (route.followerBlocked) {
    decision.reason = ReplanReason::FollowerBlocked;
  } else if (!std::isfinite(route.lastPoseCorrectionInches) ||
             std::abs(route.lastPoseCorrectionInches) >=
                 config_.poseCorrectionThresholdInches) {
    decision.reason = ReplanReason::LocalizationShift;
  } else if (currentWorldRevision != route.worldRevision &&
             !remainingRouteIsClear(route, obstacles, field)) {
    decision.reason = ReplanReason::RouteObstructed;
  }

  if (decision.reason == ReplanReason::InvalidRoute) return decision;
  if (decision.reason == ReplanReason::None) return decision;
  if (hasTriggered_ && nowMs - lastTriggerMs_ < config_.minimumIntervalMs) {
    decision.reason = ReplanReason::RateLimited;
    return decision;
  }
  hasTriggered_ = true;
  lastTriggerMs_ = nowMs;
  return decision;
}

void Replanner::reset() noexcept {
  hasTriggered_ = false;
  lastTriggerMs_ = 0;
}

}  // namespace aon::navigation
