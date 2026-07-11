#include "aon/auton/step-logger.hpp"

#include "pros/rtos.hpp"

#include <cstdio>

namespace aon::auton {

void logStep(const char* routine, const char* step) {
  std::printf("AUTON_STEP routine=%s step=%s time=%lu\n", routine, step,
              static_cast<unsigned long>(pros::millis()));
}

}  // namespace aon::auton
