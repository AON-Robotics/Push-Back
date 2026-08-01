#include "aon/auton/jerryio-path-auton.hpp"
#include "aon/auton/routines.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
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

double initialHeading(const PathPoint& first, const PathPoint& second) {
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  double heading = std::atan2(second.x - first.x, second.y - first.y) *
                   kRadiansToDegrees;
  if (heading < 0.0) heading += 360.0;
  return heading;
}

std::vector<PathPoint> readPath(const char* filename) {
  std::ifstream input(filename);
  CHECK(input.is_open());

  std::vector<PathPoint> points;
  bool foundTerminator = false;
  std::string line;
  while (std::getline(input, line)) {
    if (line == "endData") {
      foundTerminator = true;
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

  CHECK(foundTerminator);
  return points;
}

}  // namespace

int main() {
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunJerryIoPathAuton), int (*)()>);
  using aon::auton::JerryIoPathAuton;

  CHECK(std::string(JerryIoPathAuton::name) == "TEST JerryIO Path");
  CHECK(JerryIoPathAuton::lookahead == 10.0F);
  CHECK(JerryIoPathAuton::timeoutMs == 14000);

  const auto points = readPath("static/path.jerryio.txt");
  CHECK(points.size() >= 80);
  CHECK(points.front().x == JerryIoPathAuton::startX);
  CHECK(points.front().y == JerryIoPathAuton::startY);
  CHECK(std::abs(initialHeading(points[0], points[1]) -
                 JerryIoPathAuton::startHeading) <= 0.1);
  CHECK(points.back().speed == 0.0);

  for (const PathPoint& point : points) {
    CHECK(point.speed >= 0.0);
    CHECK(point.speed <= JerryIoPathAuton::maximumPathSpeed);
  }

  std::cout << "JerryIO path auton tests passed\n";
}
