#pragma once

namespace lemlib {
class Chassis;
}

namespace aon::lemlib_integration {

/// Returns the small robot's LemLib chassis.
///
/// The chassis and its devices are constructed lazily. Merely linking this
/// module does not initialize sensors or command the drivetrain.
lemlib::Chassis& chassis();

/// Calibrates LemLib odometry and continuously displays its pose.
/// This function never commands a motor.
void startSensorTest();

/// Runs one low-speed autonomous turn and displays the final heading error.
void runTurnTest(double targetHeading);

/// Runs one full turn in each direction and reports tracking-wheel offsets.
void runTrackingCalibrationTest();

}  // namespace aon::lemlib_integration
