#pragma once

#ifndef AON_API_HPP_
#define AON_API_HPP_

#include "../main.hpp"
#include "./constants.hpp"
#include "./controls/smart_motor.hpp"
#include "./globals.hpp"

#include "./controls/pid/pid.hpp"

#include "./competition/operator-control.hpp"
#include "./competition/autonomous-routines.hpp"

#include "./tools/logging.hpp"
#include "./tools/json.hpp"
#include "./tools/gui/base-gui.hpp"

#include "./controls/trapezoid-profile/trapezoid.hpp"
#include "./controls/exponential-profile.hpp"
#include "./sensing/odometry.hpp"

#if !USING_BLACK_ROBOT
#include "./controls/holonomic-motion.hpp"
#endif

#endif  // AON_API_HPP_
