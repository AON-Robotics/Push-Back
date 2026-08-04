#include "aon/auton/lemlib-validation.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                       \
      std::fprintf(stderr, "CHECK failed at line %d: %s\n", __LINE__,       \
                   #condition);                                              \
      return EXIT_FAILURE;                                                    \
    }                                                                         \
  } while (false)

using aon::auton::LemLibValidationMotion;
using aon::auton::LemLibValidationPose;
using aon::auton::LemLibValidationTest;

struct Command {
  std::string kind;
  double first;
  double second;
  bool forwards;
  int maxSpeed;
  int timeoutMs;
};

class FakeMotion final : public LemLibValidationMotion {
 public:
  void setPose(const LemLibValidationPose& pose) override {
    poses.push_back(pose);
  }

  bool moveToPoint(const char*, double x, double y, bool forwards,
                   int maxSpeed, int timeoutMs) override {
    commands.push_back({"point", x, y, forwards, maxSpeed, timeoutMs});
    return nextResult();
  }

  bool turnToHeading(const char*, double heading, int maxSpeed,
                     int timeoutMs) override {
    commands.push_back({"turn", heading, 0.0, true, maxSpeed, timeoutMs});
    return nextResult();
  }

  LemLibValidationPose pose() const override { return finalPose; }

  void stop() noexcept override { ++stopCount; }

  bool nextResult() {
    if (results.empty()) return true;
    const bool result = results.front();
    results.erase(results.begin());
    return result;
  }

  LemLibValidationPose finalPose{0.25, 11.75, 1.5};
  std::vector<LemLibValidationPose> poses;
  std::vector<Command> commands;
  std::vector<bool> results;
  int stopCount = 0;
};

std::string readFile(std::FILE* file) {
  std::rewind(file);
  std::string text;
  char buffer[256];
  while (std::fgets(buffer, sizeof(buffer), file) != nullptr) text += buffer;
  return text;
}

constexpr const char* kOutputPath =
    "bin/host-tests/lemlib-validation-output.txt";

std::FILE* openOutput() { return std::fopen(kOutputPath, "w+"); }

void closeOutput(std::FILE* output) {
  std::fclose(output);
  std::remove(kOutputPath);
}

bool near(double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-9; }

int checkLinear(LemLibValidationTest test, double expectedY,
                bool expectedForwards) {
  FakeMotion motion;
  std::FILE* output = openOutput();
  CHECK(output != nullptr);
  const auto result = aon::auton::runLemLibValidation(test, motion, output);
  closeOutput(output);

  CHECK(result.succeeded);
  CHECK(motion.poses.size() == 1);
  CHECK(near(motion.poses[0].x, 0.0));
  CHECK(near(motion.poses[0].y, 0.0));
  CHECK(near(motion.poses[0].heading, 0.0));
  CHECK(motion.commands.size() == 1);
  CHECK(motion.commands[0].kind == "point");
  CHECK(near(motion.commands[0].first, 0.0));
  CHECK(near(motion.commands[0].second, expectedY));
  CHECK(motion.commands[0].forwards == expectedForwards);
  CHECK(motion.commands[0].maxSpeed == 40);
  CHECK(motion.commands[0].timeoutMs == 3000);
  CHECK(motion.stopCount == 1);
  return EXIT_SUCCESS;
}

int checkTurn(LemLibValidationTest test, double expectedHeading) {
  FakeMotion motion;
  std::FILE* output = openOutput();
  CHECK(output != nullptr);
  const auto result = aon::auton::runLemLibValidation(test, motion, output);
  closeOutput(output);

  CHECK(result.succeeded);
  CHECK(motion.poses.size() == 1);
  CHECK(motion.commands.size() == 1);
  CHECK(motion.commands[0].kind == "turn");
  CHECK(near(motion.commands[0].first, expectedHeading));
  CHECK(motion.commands[0].maxSpeed == 40);
  CHECK(motion.commands[0].timeoutMs == 2500);
  CHECK(motion.stopCount == 1);
  return EXIT_SUCCESS;
}

}  // namespace

int main() {
  CHECK(checkLinear(LemLibValidationTest::Forward, 12.0, true) ==
        EXIT_SUCCESS);
  CHECK(checkLinear(LemLibValidationTest::Reverse, -12.0, false) ==
        EXIT_SUCCESS);
  CHECK(checkTurn(LemLibValidationTest::Clockwise90, 90.0) == EXIT_SUCCESS);
  CHECK(checkTurn(LemLibValidationTest::Counterclockwise90, -90.0) ==
        EXIT_SUCCESS);

  FakeMotion outputMotion;
  std::FILE* output = openOutput();
  CHECK(output != nullptr);
  const auto outputResult = aon::auton::runLemLibValidation(
      LemLibValidationTest::Forward, outputMotion, output);
  const std::string text = readFile(output);
  closeOutput(output);
  CHECK(outputResult.succeeded);
  CHECK(text.find("LEMLIB_VALIDATION test=forward expected_x=0.000 "
                  "expected_y=12.000 expected_h=0.000 actual_x=0.250 "
                  "actual_y=11.750 actual_h=1.500 succeeded=1") !=
        std::string::npos);

  FakeMotion failedMotion;
  failedMotion.results = {false};
  output = openOutput();
  CHECK(output != nullptr);
  const auto failed = aon::auton::runLemLibValidation(
      LemLibValidationTest::Forward, failedMotion, output);
  closeOutput(output);
  CHECK(!failed.succeeded);
  CHECK(failedMotion.stopCount == 1);

  FakeMotion combinedMotion;
  output = openOutput();
  CHECK(output != nullptr);
  CHECK(aon::auton::runAllLemLibValidations(combinedMotion, output));
  closeOutput(output);
  CHECK(combinedMotion.commands.size() == 4);
  CHECK(combinedMotion.poses.size() == 4);
  CHECK(combinedMotion.stopCount == 4);

  FakeMotion failFastMotion;
  failFastMotion.results = {true, false, true, true};
  output = openOutput();
  CHECK(output != nullptr);
  CHECK(!aon::auton::runAllLemLibValidations(failFastMotion, output));
  closeOutput(output);
  CHECK(failFastMotion.commands.size() == 2);
  CHECK(failFastMotion.stopCount == 2);

  std::puts("LemLib validation tests passed");
  return EXIT_SUCCESS;
}
