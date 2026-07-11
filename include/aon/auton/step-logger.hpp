#pragma once

namespace aon::auton {

/**
 * @brief Records entry into a meaningful autonomous behavior step.
 * @param routine Stable route name used to group serial output.
 * @param step Short description of the behavior that is about to run.
 */
void logStep(const char* routine, const char* step);

}  // namespace aon::auton
