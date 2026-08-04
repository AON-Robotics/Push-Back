#include "aon/auton/lemlib-validation.hpp"

#include <array>

namespace aon::auton {
namespace {

constexpr double kStraightDistanceInches = 12.0;
constexpr double kTurnDegrees = 90.0;
constexpr int kMaximumSpeed = 40;
constexpr int kStraightTimeoutMs = 3000;
constexpr int kTurnTimeoutMs = 2500;

struct ValidationDefinition {
  const char* name;
  LemLibValidationPose expected;
};

ValidationDefinition definitionFor(LemLibValidationTest test) {
  switch (test) {
    case LemLibValidationTest::Forward:
      return {"forward", {0.0, kStraightDistanceInches, 0.0}};
    case LemLibValidationTest::Reverse:
      return {"reverse", {0.0, -kStraightDistanceInches, 0.0}};
    case LemLibValidationTest::Clockwise90:
      return {"clockwise_90", {0.0, 0.0, kTurnDegrees}};
    case LemLibValidationTest::Counterclockwise90:
      return {"counterclockwise_90", {0.0, 0.0, -kTurnDegrees}};
  }
  return {"invalid", {0.0, 0.0, 0.0}};
}

bool execute(LemLibValidationTest test, const ValidationDefinition& definition,
             LemLibValidationMotion& motion) {
  switch (test) {
    case LemLibValidationTest::Forward:
      return motion.moveToPoint(definition.name, definition.expected.x,
                                definition.expected.y, true, kMaximumSpeed,
                                kStraightTimeoutMs);
    case LemLibValidationTest::Reverse:
      return motion.moveToPoint(definition.name, definition.expected.x,
                                definition.expected.y, false, kMaximumSpeed,
                                kStraightTimeoutMs);
    case LemLibValidationTest::Clockwise90:
    case LemLibValidationTest::Counterclockwise90:
      return motion.turnToHeading(definition.name,
                                  definition.expected.heading, kMaximumSpeed,
                                  kTurnTimeoutMs);
  }
  return false;
}

}  // namespace

LemLibValidationResult runLemLibValidation(LemLibValidationTest test,
                                            LemLibValidationMotion& motion,
                                            std::FILE* output) {
  const ValidationDefinition definition = definitionFor(test);
  motion.setPose({0.0, 0.0, 0.0});
  const bool succeeded = execute(test, definition, motion);
  motion.stop();
  const LemLibValidationPose actual = motion.pose();
  std::fprintf(output,
               "LEMLIB_VALIDATION test=%s expected_x=%.3f expected_y=%.3f "
               "expected_h=%.3f actual_x=%.3f actual_y=%.3f actual_h=%.3f "
               "succeeded=%d\n",
               definition.name, definition.expected.x, definition.expected.y,
               definition.expected.heading, actual.x, actual.y,
               actual.heading, succeeded ? 1 : 0);
  return {succeeded, actual};
}

bool runAllLemLibValidations(LemLibValidationMotion& motion,
                             std::FILE* output) {
  constexpr std::array<LemLibValidationTest, 4> tests = {
      LemLibValidationTest::Forward, LemLibValidationTest::Reverse,
      LemLibValidationTest::Clockwise90,
      LemLibValidationTest::Counterclockwise90};
  for (const LemLibValidationTest test : tests) {
    if (!runLemLibValidation(test, motion, output).succeeded) return false;
  }
  return true;
}

}  // namespace aon::auton
