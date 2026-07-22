#include "aon/auton/figure-eight-validation.hpp"
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

#define CHECK(value) do { if (!(value)) { \
  std::cerr << #value << '\n'; std::exit(1); } } while (false)

struct Point {
  double x;
  double y;
  double speed;
};

int main() {
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunLemLibFigureEightValidation),
                int (*)()>);
  using aon::auton::FigureEightValidation;
  CHECK(FigureEightValidation::startX == -20.0);
  CHECK(FigureEightValidation::startY == 22.0);
  CHECK(FigureEightValidation::startHeading == 0.0);
  CHECK(FigureEightValidation::lookahead == 8.0F);
  CHECK(FigureEightValidation::timeoutMs == 12000);
  CHECK(FigureEightValidation::maximumPathSpeed == 65);

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
  CHECK(points.size() >= 40);
  CHECK(points.front().x == FigureEightValidation::startX);
  CHECK(points.front().y == FigureEightValidation::startY);
  CHECK(points.front().speed <= 20.0);
  CHECK(points.back().speed <= 20.0);
  CHECK(points[1].y > points[0].y);
  CHECK(std::abs(points[1].x - points[0].x) < 0.1);
  CHECK(std::hypot(points.back().x - points.front().x,
                   points.back().y - points.front().y) < 0.01);

  bool reachedLeft = false;
  bool reachedRight = false;
  bool wasInCenter = false;
  int centerEntries = 0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Point& point = points[index];
    CHECK(point.x >= -22.0 && point.x <= 22.0);
    CHECK(point.y >= 0.0 && point.y <= 44.0);
    CHECK(point.speed >= 0.0 &&
          point.speed <= FigureEightValidation::maximumPathSpeed);
    reachedLeft = reachedLeft || point.x <= -12.0;
    reachedRight = reachedRight || point.x >= 12.0;
    const bool inCenter = std::abs(point.x) <= 1.5 &&
                          std::abs(point.y - 22.0) <= 1.5;
    if (inCenter && !wasInCenter) ++centerEntries;
    wasInCenter = inCenter;
    if (index != 0) {
      const double dx = point.x - points[index - 1].x;
      const double dy = point.y - points[index - 1].y;
      CHECK(std::hypot(dx, dy) <= 2.5);
      CHECK(std::abs(point.speed - points[index - 1].speed) <= 3.01);
    }
    if (index > 0 && index + 1 < points.size()) {
      const Point& before = points[index - 1];
      const Point& after = points[index + 1];
      const double a = std::hypot(point.x - before.x,
                                  point.y - before.y);
      const double b = std::hypot(after.x - point.x,
                                  after.y - point.y);
      const double c = std::hypot(after.x - before.x,
                                  after.y - before.y);
      const double twiceArea = std::abs(
          (point.x - before.x) * (after.y - before.y) -
          (point.y - before.y) * (after.x - before.x));
      if (twiceArea > 0.0001) {
        const double radius = a * b * c / (2.0 * twiceArea);
        if (radius <= 4.0) CHECK(point.speed <= 38.0);
      }
    }
  }
  CHECK(reachedLeft);
  CHECK(reachedRight);
  CHECK(centerEntries >= 2);
  std::cout << "figure-eight path tests passed\n";
}
