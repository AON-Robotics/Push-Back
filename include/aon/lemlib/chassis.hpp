#pragma once

#include "aon/core/task-start.hpp"

namespace lemlib {
class Chassis;
}

namespace aon::lemlib_integration {

/// Returns the LemLib chassis configured for the selected robot.
///
/// The chassis and its devices are constructed lazily. Merely linking this
/// module does not initialize sensors or command the drivetrain.
lemlib::Chassis& chassis();

/// Calibrates LemLib sensors and starts its odometry task for normal operation.
void initializeChassis();

/// True only when the selected localization path and required task are ready.
bool localizationReady();

/// Starts the authorized publisher or returns a typed, fail-closed result.
[[nodiscard]] core::TaskStartResult startFusedLocalization();

/// Calibrates LemLib odometry and continuously displays its pose.
/// This function never commands a motor.
void startSensorTest();

/// Runs one low-speed autonomous turn and displays the final heading error.
void runTurnTest(double targetHeading);

/// Runs one full turn in each direction and reports tracking-wheel offsets.
void runTrackingCalibrationTest();

}  // namespace aon::lemlib_integration
