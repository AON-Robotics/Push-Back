#include "aon/auton/jerryio-path-auton.hpp"
#include "aon/auton/jerryio-sequence.hpp"
#include "aon/auton/lemlib-routes.hpp"

#include <algorithm>
#include <array>
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

double distance(const PathPoint& point, double x, double y) {
  return std::hypot(point.x - x, point.y - y);
}

void checkPathLeg(const char* filename, double startX, double startY,
                  double endX, double endY) {
  using aon::auton::JerryIoPathAuton;
  const auto points = readPath(filename);
  CHECK(points.size() >= 8);
  CHECK(distance(points.front(), startX, startY) <= 0.05);
  CHECK(distance(points.back(), endX, endY) <= 0.05);
  CHECK(points.back().speed == 0.0);
  for (std::size_t index = 0; index < points.size(); ++index) {
    CHECK(points[index].speed >= 0.0);
    CHECK(points[index].speed <= JerryIoPathAuton::maximumPathSpeed);
    if (index > 0) {
      CHECK(distance(points[index], points[index - 1].x,
                     points[index - 1].y) <= 2.5);
    }
  }
}

void checkSequence() {
  using aon::auton::JerryIoCallbacks;
  using aon::auton::JerryIoPhase;
  constexpr std::array<JerryIoPhase, 6> phases{{
      JerryIoPhase::FollowToIntake,
      JerryIoPhase::Intake,
      JerryIoPhase::FollowToOuttake,
      JerryIoPhase::Outtake,
      JerryIoPhase::FollowToPistons,
      JerryIoPhase::PulsePistons,
  }};

  std::vector<JerryIoPhase> visited;
  int stopCount = 0;
  JerryIoCallbacks successCallbacks{
      [&](JerryIoPhase phase) {
        visited.push_back(phase);
        return true;
      },
      [&] { ++stopCount; },
  };
  const auto success = aon::auton::runJerryIoSequence(successCallbacks);
  CHECK(success.succeeded);
  CHECK(visited == std::vector<JerryIoPhase>(phases.begin(), phases.end()));
  CHECK(stopCount == 1);

  for (std::size_t failure = 0; failure < phases.size(); ++failure) {
    visited.clear();
    stopCount = 0;
    JerryIoCallbacks failureCallbacks{
        [&](JerryIoPhase phase) {
          visited.push_back(phase);
          return phase != phases[failure];
        },
        [&] { ++stopCount; },
    };
    const auto result = aon::auton::runJerryIoSequence(failureCallbacks);
    CHECK(!result.succeeded);
    CHECK(result.failedPhase == phases[failure]);
    CHECK(visited.size() == failure + 1);
    CHECK(stopCount == 1);
  }
}

}  // namespace

int main() {
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunJerryIoPathAuton), int (*)()>);
  using aon::auton::JerryIoPathAuton;

  CHECK(std::string(JerryIoPathAuton::name) == "TEST JerryIO Path");
  CHECK(JerryIoPathAuton::lookahead == 10.0F);
  CHECK(JerryIoPathAuton::timeoutMs == 14000);

  const auto firstLeg = readPath("static/path-jerryio-intake.txt");
  CHECK(std::abs(initialHeading(firstLeg[0], firstLeg[1]) -
                 JerryIoPathAuton::startHeading) <= 0.1);
  checkPathLeg("static/path-jerryio-intake.txt", JerryIoPathAuton::startX,
               JerryIoPathAuton::startY, JerryIoPathAuton::intakeX,
               JerryIoPathAuton::intakeY);
  checkPathLeg("static/path-jerryio-outtake.txt", JerryIoPathAuton::intakeX,
               JerryIoPathAuton::intakeY, JerryIoPathAuton::outtakeX,
               JerryIoPathAuton::outtakeY);
  checkPathLeg("static/path-jerryio-pistons.txt", JerryIoPathAuton::outtakeX,
               JerryIoPathAuton::outtakeY, JerryIoPathAuton::pistonsX,
               JerryIoPathAuton::pistonsY);
  checkSequence();

  std::cout << "JerryIO path auton tests passed\n";
}
