#ifndef AON_CONSTANTS_HPP_
#define AON_CONSTANTS_HPP_

#include "pros/colors.h"

#define PROS_USE_SIMPLE_NAMES

// PROS 4 keeps screen color constants in pros::c instead of exporting the
// legacy global names used by the existing GUI.
inline constexpr auto COLOR_ANTIQUE_WHITE = pros::c::COLOR_ANTIQUE_WHITE;
inline constexpr auto COLOR_BLACK = pros::c::COLOR_BLACK;
inline constexpr auto COLOR_BLUE = pros::c::COLOR_BLUE;
inline constexpr auto COLOR_CRIMSON = pros::c::COLOR_CRIMSON;
inline constexpr auto COLOR_CYAN = pros::c::COLOR_CYAN;
inline constexpr auto COLOR_DARK_BLUE = pros::c::COLOR_DARK_BLUE;
inline constexpr auto COLOR_DARK_GRAY = pros::c::COLOR_DARK_GRAY;
inline constexpr auto COLOR_DARK_GREEN = pros::c::COLOR_DARK_GREEN;
inline constexpr auto COLOR_DARK_RED = pros::c::COLOR_DARK_RED;
inline constexpr auto COLOR_GRAY = pros::c::COLOR_GRAY;
inline constexpr auto COLOR_GREEN = pros::c::COLOR_GREEN;
inline constexpr auto COLOR_LIGHT_GRAY = pros::c::COLOR_LIGHT_GRAY;
inline constexpr auto COLOR_LIGHT_GREEN = pros::c::COLOR_LIGHT_GREEN;
inline constexpr auto COLOR_LIGHT_PINK = pros::c::COLOR_LIGHT_PINK;
inline constexpr auto COLOR_ORANGE = pros::c::COLOR_ORANGE;
inline constexpr auto COLOR_RED = pros::c::COLOR_RED;
inline constexpr auto COLOR_SKY_BLUE = pros::c::COLOR_SKY_BLUE;
inline constexpr auto COLOR_STEEL_BLUE = pros::c::COLOR_STEEL_BLUE;
inline constexpr auto COLOR_WHITE = pros::c::COLOR_WHITE;
inline constexpr auto COLOR_YELLOW = pros::c::COLOR_YELLOW;
inline constexpr auto COLOR_YELLOW_GREEN = pros::c::COLOR_YELLOW_GREEN;

#define BRAIN_SCREEN_WIDTH 480
#define BRAIN_SCREEN_HEIGHT 240
#define DEGREES_PER_REVOLUTION 360.0
#define TILE_WIDTH 23.6220472441
#define TILE_DIAG_LENGTH 33.4066195836 // Calculated with the Pythagorean theorem

// NOT using big robot = Using small robot
#define USING_BIG_ROBOT false
#define TESTING_AUTONOMOUS false
#define LEMLIB_SENSOR_TEST false
#define LEMLIB_TURN_TEST false
#define LEMLIB_TRACKING_CALIBRATION_TEST false
#define LEMLIB_TURN_TEST_ANGLE 90.0
#define USE_LEMLIB_CURVATURE_DRIVER true

#if USING_BIG_ROBOT

#define SENSITIVITY 10 // 3-10 works good, currently undergoing testing // Higher is more sensitivity
#define DRIVE_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 2
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 1.572
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 1.572
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 1.572
#define MOTOR_TO_DRIVE_RATIO 1.3 // NumTeethMotorGear / NumTeethWheelGear
#define GYRO_ENABLED true
#define GYRO_CONFIDENCE 1
#define GYRO_FILTER_LENGTH 1
#define ENCODER_CONFIDENCE 0
#define OFFSET_X_ENCODER_MID 3.250

#define DRIVE_WIDTH 15.5 // distance between front wheels
#define DRIVE_LENGTH 9 // distance from back wheel to front wheel
#define DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER 7.55
#define DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER 7.55
#define DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER 7.55 //PYTHAG
#define DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER 7.55
#define AVG_DRIVETRAIN_RADIUS (DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER) / 4
// This number may be dependent on the degrees being turned in which case it will not be a constant
#define CLOCKWISE_ROTATION_DEGREES_OFFSET 0

// Depend on the robot and the routine
#define INITIAL_ODOMETRY_X 0.0
#define INITIAL_ODOMETRY_Y 0.0
#define INITIAL_ODOMETRY_THETA 0.0

// These next four (4) are in meters (all else is inches)
#define GPS_X_OFFSET 0 // CAD
#define GPS_Y_OFFSET 0.193878095306 // CAD
#define GPS_INITIAL_X -1.42 // Field
#define GPS_INITIAL_Y -0.47 // Field
#define GPS_INITIAL_HEADING 298.8 // Field (in Degrees)

#define MAX_RPM 600 // For the drivetrain
#define INTAKE_VELOCITY 600

/// @brief Maximum acceleration without slippage
///
/// @see https://www.desmos.com/calculator/9e23f1f7b6
#define MAX_ACCEL 4991.46340024

/// @brief Maximum deceleration without tipping (limited to not confuse encoders right now)
///
/// @see https://www.desmos.com/calculator/f523970d6f
#define MAX_DECEL 500.0 // 100.0 // 2413.22817284

#define INTAKE_ACTIVATION_DISTANCE 45

#define ORBIT_HEIGHT 12.5
// ORBIT Limiting to protect when it does not have 360° of freedom
#define ORBIT_LIMITED true
#define ORBIT_LEFT_LIMIT 210
#define ORBIT_RIGHT_LIMIT 90

#else // NOT USING_BIG_ROBOT

#define SENSITIVITY 10 // 3-10 works good, currently undergoing testing // Higher is more sensitivity
#define DRIVE_WHEEL_DIAMETER 2.75
#define TRACKING_WHEEL_DIAMETER 2
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 1.125
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 1.125
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 1.572
#define MOTOR_TO_DRIVE_RATIO 0.75 // NumTeethMotorGear / NumTeethWheelGear
#define GYRO_ENABLED true
#define GYRO_CONFIDENCE 1
#define GYRO_FILTER_LENGTH 1
#define ENCODER_CONFIDENCE 0
#define OFFSET_X_ENCODER_MID 3.250

#define DRIVE_WIDTH 12.5 // distance between front wheels // 11.0625 for kevin autons
#define DRIVE_LENGTH 10.5 // distance from back wheel to front wheel
#define DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER 7.77817459305
#define DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER 7.77817459305
#define DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER 7.77817459305 //PYTHAG
#define DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER 7.77817459305
#define AVG_DRIVETRAIN_RADIUS (DISTANCE_FRONT_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_LEFT_DRIVE_WHEEL_CENTER + DISTANCE_FRONT_RIGHT_DRIVE_WHEEL_CENTER + DISTANCE_BACK_RIGHT_DRIVE_WHEEL_CENTER) / 4
// This number may be dependent on the degrees being turn in which case it will not be a constant
#define CLOCKWISE_ROTATION_DEGREES_OFFSET 0

// Depend on the robot and the routine
#define INITIAL_ODOMETRY_X 0.0
#define INITIAL_ODOMETRY_Y 0.0
#define INITIAL_ODOMETRY_THETA 0.0

// These next four (4) are in meters (all else is inches)
#define GPS_X_OFFSET 0 // CAD
#define GPS_Y_OFFSET 0.193878095306 // CAD
#define GPS_INITIAL_X -1.42 // Field
#define GPS_INITIAL_Y -0.47 // Field
#define GPS_INITIAL_HEADING 298.8 // Field (in Degrees)

#define MAX_RPM 600 // For the drivetrain
#define INTAKE_VELOCITY 600

/// @brief Maximum acceleration without slippage
///
/// @see https://www.desmos.com/calculator/9e23f1f7b6
#define MAX_ACCEL 2500.0

/// @brief Maximum deceleration without tipping (limited to not confuse encoders right now)
///
/// @see https://www.desmos.com/calculator/f523970d6f
#define MAX_DECEL 200 // 2413.22817284

#define INTAKE_ACTIVATION_DISTANCE 120

#define ORBIT_HEIGHT 12.5
// ORBIT Limiting to protect when it does not have 360° of freedom
#define ORBIT_LIMITED true
#define ORBIT_LEFT_LIMIT 210
#define ORBIT_RIGHT_LIMIT 90
#endif // NOT USING_BIG_ROBOT

/// Alliance color selected from the GUI before the match.
enum Alliance : int { Red, Blue, Skills };

#endif  // AON_CONSTANTS_HPP_
