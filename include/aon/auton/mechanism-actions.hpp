#pragma once

#include <cstdint>

namespace aon::core {
class Hardware;
}

namespace aon::auton::mechanisms {

/**
 * @brief Enables the existing asynchronous loader collection behavior.
 * @param hardware Required process-lifetime hardware owner.
 */
void beginLoaderCollection(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void beginLoaderCollection();

/**
 * @brief Disables asynchronous loader collection without stopping motors.
 * @param hardware Required process-lifetime hardware owner.
 */
void finishLoaderCollection(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void finishLoaderCollection();

/**
 * @brief Deploys the cart into its loader position.
 * @param hardware Required process-lifetime hardware owner.
 */
void prepareLoaderCart(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void prepareLoaderCart();

/**
 * @brief Returns the cart to its travel and scoring position.
 * @param hardware Required process-lifetime hardware owner.
 */
void resetLoaderCart(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void resetLoaderCart();

/**
 * @brief Raises the dedicated top scorer when the selected robot has one.
 *
 * The big robot scores through its elevator and has no equivalent scorer
 * piston, so this operation intentionally does nothing in that build.
 * @param hardware Required process-lifetime hardware owner.
 */
void prepareTopScorer(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void prepareTopScorer();

/**
 * @brief Runs the existing top-scoring sequence.
 * @param hardware Required process-lifetime hardware owner.
 * @param durationMs Time in milliseconds passed directly to Intake::score.
 */
void scoreTopBlocks(aon::core::Hardware& hardware, std::uint32_t durationMs);
/** @brief Compatibility overload using the process hardware owner. */
void scoreTopBlocks(std::uint32_t durationMs);

/**
 * @brief Extends the existing Brooks parking mechanism.
 * @param hardware Required process-lifetime hardware owner.
 */
void deployParkMechanism(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void deployParkMechanism();

/**
 * @brief Stops intake motors without changing scan or sort state.
 *
 * Preserving those state flags keeps this adapter equivalent to the existing
 * route cleanup calls.
 * @param hardware Required process-lifetime hardware owner.
 */
void stopAll(aon::core::Hardware& hardware);
/** @brief Compatibility overload using the process hardware owner. */
void stopAll();

}  // namespace aon::auton::mechanisms
