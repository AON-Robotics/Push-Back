#pragma once

#include <cstddef>
#include <cstdint>

#include "aon/field/field-map.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/navigation/path-planner.hpp"

namespace aon::navigation {

struct RouteState {
  Path path;
  std::size_t nextPointIndex = 0;
  std::uint32_t worldRevision = 0;
  std::uint32_t objectiveRevision = 0;
  double lastPoseCorrectionInches = 0.0;
  bool followerBlocked = false;
};

enum class ReplanReason : std::uint8_t {
  None,
  GoalChanged,
  RouteObstructed,
  LocalizationShift,
  FollowerBlocked,
  RateLimited,
  InvalidRoute,
};

struct ReplanDecision {
  ReplanReason reason = ReplanReason::None;
  std::uint32_t worldRevision = 0;
  std::uint32_t objectiveRevision = 0;
};

struct ReplannerConfig {
  double robotRadiusInches = 0.0;
  double safetyMarginInches = 0.0;
  double poseCorrectionThresholdInches = 0.0;
  std::uint32_t minimumIntervalMs = 0;
};

class Replanner {
 public:
  explicit Replanner(ReplannerConfig config) noexcept;

  [[nodiscard]] ReplanDecision evaluate(
      const RouteState& route, const DynamicObstacleMap& obstacles,
      const field::FieldMap& field, std::uint32_t currentWorldRevision,
      std::uint32_t currentObjectiveRevision,
      std::uint32_t nowMs) noexcept;

  void reset() noexcept;

 private:
  [[nodiscard]] bool remainingRouteIsClear(
      const RouteState& route, const DynamicObstacleMap& obstacles,
      const field::FieldMap& field) const noexcept;

  ReplannerConfig config_;
  bool hasTriggered_ = false;
  std::uint32_t lastTriggerMs_ = 0;
};

}  // namespace aon::navigation
