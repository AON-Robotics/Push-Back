#include "aon/auton/figure-eight-validation.hpp"
#include "aon/auton/lemlib-routes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#define CHECK(value) do { if (!(value)) { \
  std::cerr << #value << '\n'; std::exit(1); } } while (false)

struct Point {
  double x;
  double y;
  double speed;
};

double cross(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) -
         (b.y - a.y) * (c.x - a.x);
}

bool properIntersection(const Point& a, const Point& b,
                        const Point& c, const Point& d) {
  return cross(a, b, c) * cross(a, b, d) < -0.000001 &&
         cross(c, d, a) * cross(c, d, b) < -0.000001;
}

int main() {
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunLemLibFigureEightValidation),
                int (*)()>);
  using aon::auton::FigureEightValidation;
  CHECK(FigureEightValidation::startX == 0.0);
  CHECK(FigureEightValidation::startY == 18.0);
  CHECK(FigureEightValidation::startHeading == 90.0);
  CHECK(FigureEightValidation::lookahead == 7.0F);
  CHECK(FigureEightValidation::timeoutMs == 12000);
  CHECK(FigureEightValidation::maximumPathSpeed == 100);

  std::ifstream input("static/figure-eight.jerryio.txt");
  CHECK(input.is_open());
  std::vector<Point> points;
  bool foundTerminator = false;
  std::string line;
  while (std::getline(input, line)) {
    if (line == "endData") {
      foundTerminator = true;
      break;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    Point point{};
    CHECK(static_cast<bool>(values >> point.x >> point.y >> point.speed));
    points.push_back(point);
  }

  CHECK(foundTerminator);
  CHECK(points.size() >= 360);
  CHECK(points.front().x == FigureEightValidation::startX);
  CHECK(points.front().y == FigureEightValidation::startY);
  CHECK(points[1].x > points.front().x);
  CHECK(points.front().speed > 0.0);
  CHECK(points.back().speed == 0.0);
  CHECK(std::hypot(points.back().x - points.front().x,
                   points.back().y - points.front().y) >= 2.9);

  for (std::size_t first = 0; first + 1 < points.size(); ++first) {
    for (std::size_t second = first + 61;
         second + 1 < points.size(); ++second) {
      CHECK(!properIntersection(points[first], points[first + 1],
                                points[second], points[second + 1]));
      CHECK(std::hypot(points[first].x - points[second].x,
                       points[first].y - points[second].y) >= 2.0);
    }
  }

  double pathLength = 0.0;
  double minimumTurnRadius = 1000000.0;
  double speedSum = 0.0;
  int lowCenterPasses = 0;
  int highCenterPasses = 0;
  bool wasInLowCenter = false;
  bool wasInHighCenter = false;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Point& point = points[index];
    CHECK(point.speed >= 0.0);
    CHECK(point.speed <= FigureEightValidation::maximumPathSpeed);
    if (index + 1 < points.size()) CHECK(point.speed > 0.0);
    speedSum += point.speed;

    const bool inLowCenter =
        std::abs(point.x) <= 0.5 && std::abs(point.y - 18.0) <= 0.5;
    const bool inHighCenter =
        std::abs(point.x) <= 0.5 && std::abs(point.y - 30.0) <= 0.5;
    if (inLowCenter && !wasInLowCenter) ++lowCenterPasses;
    if (inHighCenter && !wasInHighCenter) ++highCenterPasses;
    wasInLowCenter = inLowCenter;
    wasInHighCenter = inHighCenter;

    const Point& before = points[index == 0 ? index : index - 1];
    const Point& after =
        points[index + 1 < points.size() ? index + 1 : index];
    const double tangentX = after.x - before.x;
    const double tangentY = after.y - before.y;
    const double tangentLength = std::hypot(tangentX, tangentY);
    CHECK(tangentLength > 0.0);
    const double forwardX = tangentX / tangentLength;
    const double forwardY = tangentY / tangentLength;

    constexpr double halfLength = 9.0;
    constexpr double halfWidth = 8.0;
    const double footprintX =
        halfLength * std::abs(forwardX) + halfWidth * std::abs(forwardY);
    const double footprintY =
        halfLength * std::abs(forwardY) + halfWidth * std::abs(forwardX);
    CHECK(point.x - footprintX >= -24.0);
    CHECK(point.x + footprintX <= 24.0);
    CHECK(point.y - footprintY >= 0.0);
    CHECK(point.y + footprintY <= 48.0);

    if (index > 0) {
      const double spacing = std::hypot(
          point.x - points[index - 1].x,
          point.y - points[index - 1].y);
      CHECK(spacing <= 0.5);
      pathLength += spacing;
    }
    if (index > 0 && index + 1 < points.size()) {
      const double a = std::hypot(point.x - before.x,
                                  point.y - before.y);
      const double b = std::hypot(after.x - point.x,
                                  after.y - point.y);
      const double c = std::hypot(after.x - before.x,
                                  after.y - before.y);
      const double twiceArea = std::abs(cross(before, point, after));
      if (twiceArea > 0.0001) {
        const double radius = a * b * c / (2.0 * twiceArea);
        minimumTurnRadius = std::min(minimumTurnRadius, radius);
        CHECK(radius >= 6.25);
      }
    }
  }

  CHECK(lowCenterPasses == 1);
  CHECK(highCenterPasses == 1);
  CHECK(pathLength >= 60.0);
  CHECK(pathLength <= 90.0);
  CHECK(minimumTurnRadius >= 6.25);
  CHECK(speedSum / static_cast<double>(points.size()) >= 45.0);
  std::cout << "figure-eight path tests passed\n";
}
