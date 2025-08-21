#pragma once

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS
// Trick to appease the intellisense: https://stackoverflow.com/a/26065433
#define _USE_MATH_DEFINES

#include "api.h"
#include "okapi/api.hpp"
#include "pros/imu.hpp"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "./aon/api.hpp"

#endif