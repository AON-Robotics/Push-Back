#pragma once

#include <cstdint>

namespace aon::auton::mechanisms {

/** @brief Enables the existing asynchronous loader collection behavior. */
void beginLoaderCollection();

/** @brief Disables asynchronous loader collection without stopping motors. */
void finishLoaderCollection();

/** @brief Deploys the cart into its loader position. */
void prepareLoaderCart();

/** @brief Returns the cart to its travel and scoring position. */
void resetLoaderCart();

/**
 * @brief Raises the dedicated top scorer when the selected robot has one.
 *
 * The big robot scores through its elevator and has no equivalent scorer
 * piston, so this operation intentionally does nothing in that build.
 */
void prepareTopScorer();

/**
 * @brief Runs the existing top-scoring sequence.
 * @param durationMs Time in milliseconds passed directly to Intake::score.
 */
void scoreTopBlocks(std::uint32_t durationMs);

/** @brief Extends the existing Brooks parking mechanism. */
void deployParkMechanism();

/**
 * @brief Stops intake motors without changing scan or sort state.
 *
 * Preserving those state flags keeps this adapter equivalent to the existing
 * route cleanup calls.
 */
void stopAll();

}  // namespace aon::auton::mechanisms
