#include "aon/navigation/dynamic-obstacles.hpp"

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

ObstacleDetection circle(double x, double y, std::uint32_t timeMs,
                         double confidence = 0.8) {
  return {ObstacleShape::Circle, {x, y}, 4.0, 0.0, 0.0, 0.0, confidence,
          timeMs};
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

}  // namespace

int main() {
  detectionsAssociateIntoMovingTracksAndExpire();
  olderDetectionCannotForkAnExistingTrack();
  fullMapPreservesStrongerTracks();
  invalidGeometryAndConfigurationFailClosed();
  std::cout << "navigation tests passed\n";
  return 0;
}
