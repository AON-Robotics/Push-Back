#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/navigation/path-planner.hpp"
#include "aon/navigation/replanner.hpp"
#include "aon/field/push-back-field.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

using aon::field::Point2;
using aon::navigation::DynamicObstacleMap;
using aon::navigation::ObstacleDetection;
using aon::navigation::ObstacleShape;
using aon::navigation::ObstacleUpdateResult;
using aon::navigation::PathPlanner;
using aon::navigation::PlanStatus;

ObstacleDetection circle(double x, double y, std::uint32_t timeMs,
                         double confidence = 0.8) {
  return {ObstacleShape::Circle, {x, y}, 4.0, 0.0, 0.0, 0.0, confidence,
          timeMs};
}

ObstacleDetection rectangle(double x, double y, double halfWidth,
                            double halfHeight, std::uint32_t timeMs) {
  return {ObstacleShape::Rectangle, {x, y}, 0.0, halfWidth, halfHeight,
          0.0, 0.9, timeMs};
}

void detectionsAssociateIntoMovingTracksAndExpire() {
  DynamicObstacleMap obstacles({6.0, 500, 0.5});

  CHECK(obstacles.update(circle(1.0, 0.0, 100)) ==
        ObstacleUpdateResult::Inserted);
  CHECK(obstacles.size() == 1);
  const std::uint16_t trackId = obstacles.at(0)->trackId;

  CHECK(obstacles.update(circle(3.0, 0.0, 200)) ==
        ObstacleUpdateResult::Updated);
  CHECK(obstacles.size() == 1);
  CHECK(obstacles.at(0)->trackId == trackId);
  CHECK(std::abs(obstacles.at(0)->velocityXInchesPerSecond - 10.0) < 1e-9);
  CHECK(std::abs(obstacles.at(0)->velocityYInchesPerSecond) < 1e-9);

  obstacles.expire(699);
  CHECK(obstacles.size() == 1);
  obstacles.expire(700);
  CHECK(obstacles.size() == 0);
}

void olderDetectionCannotForkAnExistingTrack() {
  DynamicObstacleMap obstacles({6.0, 500, 0.5});
  CHECK(obstacles.update(circle(1.0, 0.0, 100)) ==
        ObstacleUpdateResult::Inserted);
  CHECK(obstacles.update(circle(1.5, 0.0, 90)) ==
        ObstacleUpdateResult::OutOfOrder);
  CHECK(obstacles.size() == 1);
  CHECK(std::abs(obstacles.at(0)->center.xInches - 1.0) < 1e-9);
}

void fullMapPreservesStrongerTracks() {
  DynamicObstacleMap obstacles({6.0, 5000, 0.5});
  for (std::size_t index = 0;
       index < DynamicObstacleMap::kMaximumObstacles; ++index) {
    CHECK(obstacles.update(circle(static_cast<double>(index) * 10.0, 0.0,
                                  static_cast<std::uint32_t>(index + 1),
                                  0.9)) ==
          ObstacleUpdateResult::Inserted);
  }
  CHECK(obstacles.update(circle(200.0, 0.0, 100, 0.1)) ==
        ObstacleUpdateResult::CapacityRejected);
  CHECK(obstacles.size() == DynamicObstacleMap::kMaximumObstacles);
  CHECK(obstacles.update(circle(200.0, 0.0, 101, 0.95)) ==
        ObstacleUpdateResult::Replaced);
  CHECK(obstacles.size() == DynamicObstacleMap::kMaximumObstacles);
}

void invalidGeometryAndConfigurationFailClosed() {
  DynamicObstacleMap obstacles({6.0, 500, 0.5});
  ObstacleDetection invalid = circle(0.0, 0.0, 1);
  invalid.center.xInches = std::numeric_limits<double>::quiet_NaN();
  CHECK(obstacles.update(invalid) == ObstacleUpdateResult::Invalid);
  CHECK(obstacles.size() == 0);

  DynamicObstacleMap invalidConfig({-1.0, 0, 1.5});
  CHECK(invalidConfig.update(circle(0.0, 0.0, 1)) ==
        ObstacleUpdateResult::Invalid);
  CHECK(invalidConfig.size() == 0);
}

void plannerUsesDirectRoutesAndDetoursAroundInflatedObstacles() {
  using namespace aon::field;
  using namespace aon::navigation;

  const FieldMap& field = pushBackField();
  DynamicObstacleMap clearObstacles({6.0, 500, 0.5});
  PathPlanner planner({1.0});

  const PlanResult direct =
      planner.plan({-20.0, 0.0}, {20.0, 0.0}, 2.0, field,
                   clearObstacles);
  CHECK(direct.status == PlanStatus::Success);
  CHECK(direct.path.size == 2);

  DynamicObstacleMap blockedObstacles({6.0, 500, 0.5});
  CHECK(blockedObstacles.update(circle(0.0, 0.0, 100)) ==
        ObstacleUpdateResult::Inserted);
  const PlanResult detour =
      planner.plan({-20.0, 0.0}, {20.0, 0.0}, 2.0, field,
                   blockedObstacles);
  CHECK(detour.status == PlanStatus::Success);
  CHECK(detour.path.size > 2);
  CHECK(planner.pathHasClearance(detour.path, 2.0, field,
                                 blockedObstacles));
}

void plannerHandlesSatisfiedAndUnreachableRequests() {
  using namespace aon::field;
  using namespace aon::navigation;

  const FieldMap& field = pushBackField();
  PathPlanner planner({1.0});
  DynamicObstacleMap obstacles({6.0, 500, 0.5});
  const PlanResult satisfied =
      planner.plan({5.0, 5.0}, {5.0, 5.0}, 2.0, field, obstacles);
  CHECK(satisfied.status == PlanStatus::Success);
  CHECK(satisfied.path.size == 1);
  CHECK(std::abs(satisfied.costInches) < 1e-9);

  CHECK(obstacles.update(rectangle(0.0, 0.0, 70.0, 2.0, 100)) ==
        ObstacleUpdateResult::Inserted);
  const PlanResult unreachable =
      planner.plan({0.0, -20.0}, {0.0, 20.0}, 1.0, field, obstacles);
  CHECK(unreachable.status == PlanStatus::Unreachable);
  CHECK(unreachable.path.size == 0);
}

void replanningOnlyTriggersForMeaningfulRouteChanges() {
  using namespace aon::field;
  using namespace aon::navigation;

  const FieldMap& field = pushBackField();
  DynamicObstacleMap obstacles({6.0, 500, 0.5});
  RouteState route;
  route.path.points[0] = {-20.0, 0.0};
  route.path.points[1] = {20.0, 0.0};
  route.path.size = 2;
  route.nextPointIndex = 1;
  route.worldRevision = 1;
  route.objectiveRevision = 4;

  Replanner replanner({2.0, 1.0, 5.0, 250});
  CHECK(replanner.evaluate(route, obstacles, field, 1, 4, 100).reason ==
        ReplanReason::None);

  CHECK(obstacles.update(circle(0.0, 30.0, 110)) ==
        ObstacleUpdateResult::Inserted);
  CHECK(replanner.evaluate(route, obstacles, field, 2, 4, 120).reason ==
        ReplanReason::None);

  CHECK(obstacles.update(circle(0.0, 0.0, 130)) ==
        ObstacleUpdateResult::Inserted);
  CHECK(replanner.evaluate(route, obstacles, field, 3, 4, 140).reason ==
        ReplanReason::RouteObstructed);
  CHECK(replanner.evaluate(route, obstacles, field, 3, 5, 150).reason ==
        ReplanReason::RateLimited);
  CHECK(replanner.evaluate(route, obstacles, field, 3, 5, 400).reason ==
        ReplanReason::GoalChanged);

  RouteState invalidRoute;
  CHECK(replanner.evaluate(invalidRoute, obstacles, field, 3, 5, 410).reason ==
        ReplanReason::InvalidRoute);
}

}  // namespace

int main() {
  detectionsAssociateIntoMovingTracksAndExpire();
  olderDetectionCannotForkAnExistingTrack();
  fullMapPreservesStrongerTracks();
  invalidGeometryAndConfigurationFailClosed();
  plannerUsesDirectRoutesAndDetoursAroundInflatedObstacles();
  plannerHandlesSatisfiedAndUnreachableRequests();
  replanningOnlyTriggersForMeaningfulRouteChanges();
  std::cout << "navigation tests passed\n";
  return 0;
}
