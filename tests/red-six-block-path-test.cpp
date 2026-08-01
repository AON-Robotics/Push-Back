#include "aon/auton/red-six-block.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define CHECK(value)                                                        \
  do {                                                                      \
    if (!(value)) {                                                         \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #value << '\n'; \
      std::exit(1);                                                         \
    }                                                                       \
  } while (false)

namespace {

struct PathPoint {
  double x;
  double y;
  double speed;
};

double cross(const PathPoint& a, const PathPoint& b, const PathPoint& c) {
  return (b.x - a.x) * (c.y - a.y) -
         (b.y - a.y) * (c.x - a.x);
}

bool properIntersection(const PathPoint& a, const PathPoint& b,
                        const PathPoint& c, const PathPoint& d) {
  return cross(a, b, c) * cross(a, b, d) < -0.000001 &&
         cross(c, d, a) * cross(c, d, b) < -0.000001;
}

double headingOf(const PathPoint& from, const PathPoint& to) {
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  double heading = std::atan2(to.x - from.x, to.y - from.y) *
                   kRadiansToDegrees;
  if (heading < 0.0) heading += 360.0;
  return heading;
}

double headingError(double actual, double expected) {
  return std::abs(std::remainder(actual - expected, 360.0));
}

std::vector<PathPoint> readPath(const char* filename) {
  std::ifstream input(filename);
  CHECK(input.is_open());
  std::vector<PathPoint> points;
  std::string line;
  bool terminated = false;
  while (std::getline(input, line)) {
    if (line == "endData") {
      terminated = true;
      break;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    PathPoint point{};
    CHECK(static_cast<bool>(values >> point.x >> point.y >> point.speed));
    CHECK(std::isfinite(point.x));
    CHECK(std::isfinite(point.y));
    CHECK(std::isfinite(point.speed));
    points.push_back(point);
  }
  CHECK(terminated);
  return points;
}

void checkPath(const std::vector<PathPoint>& points,
               const aon::auton::RedSixPose& start,
               const aon::auton::RedSixPose& end, double maximumSpeed) {
  CHECK(points.size() >= 12);
  CHECK(std::hypot(points.front().x - start.x,
                   points.front().y - start.y) <= 0.05);
  CHECK(std::hypot(points.back().x - end.x,
                   points.back().y - end.y) <= 0.05);
  CHECK(headingError(headingOf(points[0], points[1]), start.heading) <= 8.0);
  CHECK(headingError(headingOf(points[points.size() - 2], points.back()),
                     end.heading) <= 8.0);

  for (std::size_t index = 0; index < points.size(); ++index) {
    const PathPoint& point = points[index];
    CHECK(point.speed >= 0.0);
    CHECK(point.speed <= maximumSpeed);
    CHECK(index + 1 == points.size() ? point.speed == 0.0
                                     : point.speed > 0.0);
    CHECK(point.x - 8.0 >= -18.0);
    CHECK(point.x + 8.0 <= 18.0);
    CHECK(point.y - 8.0 >= -8.0);
    CHECK(point.y + 8.0 <= 40.0);

    if (index > 0) {
      CHECK(std::hypot(point.x - points[index - 1].x,
                       point.y - points[index - 1].y) <= 2.0);
      if (index + 1 < points.size()) {
        CHECK(std::abs(point.speed - points[index - 1].speed) <= 15.0);
      }
    }
    if (index > 0 && index + 1 < points.size()) {
      const PathPoint& before = points[index - 1];
      const PathPoint& after = points[index + 1];
      const double a = std::hypot(point.x - before.x, point.y - before.y);
      const double b = std::hypot(after.x - point.x, after.y - point.y);
      const double c = std::hypot(after.x - before.x, after.y - before.y);
      const double twiceArea = std::abs(cross(before, point, after));
      if (twiceArea > 0.0001) {
        CHECK(a * b * c / (2.0 * twiceArea) >= 6.25);
      }
    }
  }

  for (std::size_t first = 0; first + 1 < points.size(); ++first) {
    for (std::size_t second = first + 2; second + 1 < points.size(); ++second) {
      CHECK(!properIntersection(points[first], points[first + 1],
                                points[second], points[second + 1]));
    }
  }
}

}  // namespace

int main() {
  using aon::auton::RedSixBlock;
  constexpr int configuredBudget =
      RedSixBlock::loaderPathTimeoutMs +
      RedSixBlock::loaderContactTimeoutMs + RedSixBlock::collectTimeoutMs +
      RedSixBlock::reverseClearanceTimeoutMs +
      RedSixBlock::reverseAlignmentTimeoutMs +
      RedSixBlock::goalPathTimeoutMs + RedSixBlock::goalContactTimeoutMs +
      RedSixBlock::scoreTimeoutMs;
  static_assert(configuredBudget <=
                RedSixBlock::autonomousLimitMs -
                    RedSixBlock::requiredMarginMs);

  const auto loader = readPath("static/red-six-loader-approach.jerryio.txt");
  const auto goal = readPath("static/red-six-goal-transfer.jerryio.txt");
  CHECK(loader.size() ==
        aon::auton::RedSixBlockGenerated::loaderPointCount);
  CHECK(goal.size() == aon::auton::RedSixBlockGenerated::goalPointCount);
  checkPath(loader, RedSixBlock::start, RedSixBlock::loaderStage, 90.0);
  checkPath(goal, RedSixBlock::reverseAlignment, RedSixBlock::goalStage, 45.0);

  std::cout << "red six-block path tests passed\n";
}
