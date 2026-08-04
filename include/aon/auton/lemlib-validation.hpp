#pragma once

#include <cstdio>

namespace aon::auton {

/** @brief Isolated motion primitives used to validate LemLib calibration. */
enum class LemLibValidationTest {
  Forward,
  Reverse,
  Clockwise90,
  Counterclockwise90,
};

/**
 * @brief Tracking-center pose reported by a LemLib validation run.
 *
 * Linear coordinates are inches. Heading is degrees, with zero along +Y and
 * positive rotation clockwise, matching this project's LemLib convention.
 */
struct LemLibValidationPose {
  double x;
  double y;
  double heading;
};

/**
 * @brief Hardware boundary required by the validation sequencer.
 *
 * Implementations do not transfer ownership and must remain valid for the
 * entire synchronous validation call. Calls command drivetrain hardware and
 * are not thread-safe.
 */
class LemLibValidationMotion {
 public:
  virtual ~LemLibValidationMotion() = default;

  /** @brief Resets the tracking-center pose before an isolated motion. */
  virtual void setPose(const LemLibValidationPose& pose) = 0;

  /**
   * @brief Drives to a point expressed in inches.
   * @return True only when the motion controller reports success.
   */
  virtual bool moveToPoint(const char* name, double x, double y,
                           bool forwards, int maxSpeed, int timeoutMs) = 0;

  /**
   * @brief Turns to an absolute heading expressed in degrees.
   * @return True only when the motion controller reports success.
   */
  virtual bool turnToHeading(const char* name, double heading, int maxSpeed,
                             int timeoutMs) = 0;

  /** @brief Returns the latest tracking-center pose. */
  virtual LemLibValidationPose pose() const = 0;

  /** @brief Stops the drivetrain after every success or failure path. */
  virtual void stop() noexcept = 0;
};

/** @brief Result of one isolated validation primitive. */
struct LemLibValidationResult {
  bool succeeded;
  LemLibValidationPose actual;
};

/**
 * @brief Runs one conservative validation primitive and prints one stable
 * measurement record.
 *
 * Each call resets pose to `(0, 0, 0)`, commands exactly one motion, stops the
 * drivetrain, and reports expected and actual pose. Straight targets are 12
 * inches; turn targets are 90 degrees; output is capped at 40/127.
 *
 * @param test Primitive to run.
 * @param motion Required non-owning hardware boundary.
 * @param output Open stream that receives the stable result record.
 * @return Motion success and the final reported pose.
 * @pre `output` is non-null and writable.
 * @warning Synchronous and not thread-safe; commands drivetrain hardware.
 */
LemLibValidationResult runLemLibValidation(
    LemLibValidationTest test, LemLibValidationMotion& motion,
    std::FILE* output = stdout);

/**
 * @brief Runs all four isolated primitives in safety-first order.
 * @return True only when every primitive succeeds; stops at the first failure.
 * @warning Use only after each primitive has passed its five-run physical gate.
 */
bool runAllLemLibValidations(LemLibValidationMotion& motion,
                             std::FILE* output = stdout);

}  // namespace aon::auton
