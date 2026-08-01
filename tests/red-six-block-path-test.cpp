#include "aon/auton/red-six-block.hpp"
#include "aon/auton/hybrid-sequence.hpp"
#include "aon/auton/routines.hpp"

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

std::string readText(const char* filename) {
  std::ifstream input(filename);
  CHECK(input.is_open());
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

std::string functionBody(const std::string& source, const char* name,
                         const char* nextName) {
  const std::size_t begin = source.find(name);
  CHECK(begin != std::string::npos);
  const std::size_t end = source.find(nextName, begin + 1);
  CHECK(end != std::string::npos);
  return source.substr(begin, end - begin);
}

void checkRegistration() {
  const std::string selectors =
      readText("src/aon/auton/routine-selectors.cpp");
  const std::string redThree =
      functionBody(selectors, "int RedRoutine3()", "int BlueRoutine1()");
  const std::string blueThree =
      functionBody(selectors, "int BlueRoutine3()", "int SkillsRoutine1()");
  const std::string skillsThree =
      functionBody(selectors, "int SkillsRoutine3()", "int RunShadowPlayback()");
  CHECK(redThree.find("RunRedSixBlockHybridFull") != std::string::npos);
  CHECK(blueThree.find("RunLemLibFigureEightValidation") !=
        std::string::npos);
  CHECK(skillsThree.find("RunRedSixBlock") == std::string::npos);
  CHECK(selectors.find("int RedRoutine1()") != std::string::npos);
  CHECK(selectors.find("int RedRoutine2()") != std::string::npos);

  const std::string gui = readText("include/aon/tools/gui/gui.hpp");
  CHECK(gui.find("{aon::auton::RedSixBlock::name, "
                 "aon::routines::RedRoutine3}") != std::string::npos);
  CHECK(gui.find("{aon::auton::FigureEightValidation::name, "
                 "aon::routines::BlueRoutine3}") != std::string::npos);
  const std::size_t skillsStart = gui.find("skillsAutonOptions");
  CHECK(skillsStart != std::string::npos);
  CHECK(gui.find("RedSixBlock", skillsStart) == std::string::npos);
}

std::size_t occurrenceCount(const std::string& text, const char* needle) {
  std::size_t count = 0;
  std::size_t offset = 0;
  while ((offset = text.find(needle, offset)) != std::string::npos) {
    ++count;
    offset += std::char_traits<char>::length(needle);
  }
  return count;
}

void checkSafetyWiring() {
  const std::string routines = readText("src/aon/auton/lemlib-routines.cpp");
  const std::string redRoutine = functionBody(
      routines, "int RunRedSixBlockHybridAuton", "int RunRedSixBlockHybridFull");
  CHECK(occurrenceCount(redRoutine, "OdometryMonitoring::FailClosed") == 7);

  const std::string reverseAlignment = functionBody(
      redRoutine, "case RedSixPhase::ReverseAlignment",
      "case RedSixPhase::GoalPursuit");
  CHECK(reverseAlignment.find(".minSpeed") == std::string::npos);
  CHECK(reverseAlignment.find(".earlyExitRange") == std::string::npos);

  const std::string collection = functionBody(
      redRoutine, "case RedSixPhase::CollectSix",
      "case RedSixPhase::ReverseClearance");
  CHECK(collection.find("arcadeFor") != std::string::npos);

  const std::string generator =
      readText("tools/generate-red-six-block-paths.ps1");
  CHECK(generator.find("ToString('F5', $invariant)") != std::string::npos);
}

void checkSequence() {
  using aon::auton::RedSixCallbacks;
  using aon::auton::RedSixPhase;
  constexpr std::array<RedSixPhase, 8> phases{{
      RedSixPhase::LoaderPursuit, RedSixPhase::LoaderContact,
      RedSixPhase::CollectSix, RedSixPhase::ReverseClearance,
      RedSixPhase::ReverseAlignment, RedSixPhase::GoalPursuit,
      RedSixPhase::GoalContact, RedSixPhase::ScoreSix}};

  for (std::size_t failure = 0; failure < phases.size(); ++failure) {
    std::vector<RedSixPhase> visited;
    int stopCount = 0;
    RedSixCallbacks callbacks{
        [&](RedSixPhase phase) {
          visited.push_back(phase);
          return phase != phases[failure];
        },
        [&] { ++stopCount; }};
    const auto result = aon::auton::runRedSixSequence(callbacks);
    CHECK(!result.succeeded);
    CHECK(result.failedPhase == phases[failure]);
    CHECK(visited.size() == failure + 1);
    CHECK(stopCount == 1);
  }

  for (const RedSixPhase stopAfter :
       {RedSixPhase::LoaderPursuit, RedSixPhase::LoaderContact,
        RedSixPhase::CollectSix, RedSixPhase::ReverseAlignment,
        RedSixPhase::GoalContact, RedSixPhase::ScoreSix}) {
    std::vector<RedSixPhase> visited;
    int stopCount = 0;
    RedSixCallbacks callbacks{
        [&](RedSixPhase phase) {
          visited.push_back(phase);
          return true;
        },
        [&] { ++stopCount; }};
    const auto result = aon::auton::runRedSixSequence(callbacks, stopAfter);
    CHECK(result.succeeded);
    CHECK(visited.back() == stopAfter);
    CHECK(stopCount == 1);
  }

  int runCount = 0;
  int stopCount = 0;
  RedSixCallbacks invalidCallbacks{
      [&](RedSixPhase) {
        ++runCount;
        return true;
      },
      [&] { ++stopCount; }};
  const auto invalid = aon::auton::runRedSixSequence(
      invalidCallbacks, static_cast<RedSixPhase>(255));
  CHECK(!invalid.succeeded);
  CHECK(runCount == 0);
  CHECK(stopCount == 1);
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
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunRedSixBlockHybridAuton),
                int (*)(aon::auton::RedSixPhase)>);
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
  checkSequence();
  checkRegistration();
  checkSafetyWiring();

  std::cout << "red six-block path tests passed\n";
}
