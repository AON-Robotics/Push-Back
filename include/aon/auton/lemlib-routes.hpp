#pragma once

#include "aon/auton/hybrid-sequence.hpp"

namespace aon::routines {

/**
 * @brief Runs an isolated LemLib heading characterization.
 * @param name Stable label written to autonomous logs; must remain valid for
 * the synchronous call.
 * @param heading Target heading in degrees using the configured LemLib heading
 * convention.
 * @return One on success, otherwise zero.
 * @warning Commands initialized drivetrain hardware synchronously and is not
 * thread-safe.
 */
int RunLemLibTurnCharacterization(const char* name, double heading);

/** @brief Runs the isolated forward LemLib validation motion. */
int RunLemLibForwardValidation();
/** @brief Runs the isolated reverse LemLib validation motion. */
int RunLemLibReverseValidation();
/** @brief Runs the isolated clockwise 90-degree validation motion. */
int RunLemLibClockwiseTurnValidation();
/** @brief Runs the isolated counterclockwise 90-degree validation motion. */
int RunLemLibCounterclockwiseTurnValidation();
/**
 * @brief Runs all isolated LemLib primitives in safety-first order.
 * @warning Use only after every primitive passes five repeatable physical runs.
 */
int RunLemLibCombinedValidation();
/** @brief Runs the configured LemLib figure-eight path validation. */
int RunLemLibFigureEightValidation();
/** @brief Runs the configured JerryIO path autonomous experiment. */
int RunJerryIoPathAuton();
/** @brief Runs the currently enabled staged loader-scoring experiment. */
int RunStagedLoaderScoreExperiment();

/**
 * @brief Runs the red-side six-block loader-to-long-goal autonomous.
 *
 * Starting configuration:
 * - Tracking center at the documented local red starting origin.
 * - Robot front facing +Y at heading zero degrees.
 * - Loader cart and scorer clear for initialization.
 *
 * @param stopAfter Inclusive phase used by progressive physical gates.
 * @return One when all requested phases succeed, otherwise zero.
 * @warning Commands drivetrain and mechanism hardware synchronously and is not
 * thread-safe.
 */
int RunRedSixBlockHybridAuton(
    aon::auton::RedSixPhase stopAfter = aon::auton::RedSixPhase::ScoreSix);

/** @brief Runs the complete red six-block route for selector registration. */
int RunRedSixBlockHybridFull();

/**
 * @brief Runs the armed Shadow recording through the autonomous route wrapper.
 * @return One when playback completes successfully, otherwise zero.
 */
int RunShadowPlayback();

}  // namespace aon::routines
