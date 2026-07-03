#include "aon/core/robot.hpp"

namespace {
aon::core::Robot robot;
}

void initialize() { robot.initialize(); }

void disabled() { robot.disabled(); }

void competition_initialize() { robot.competitionInitialize(); }

void autonomous() { robot.autonomous(); }

void opcontrol() { robot.opcontrol(); }
