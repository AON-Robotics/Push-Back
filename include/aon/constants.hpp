#ifndef AON_CONSTANTS_HPP_
#define AON_CONSTANTS_HPP_

#define BRAIN_SCREEN_WIDTH 480
#define BRAIN_SCREEN_HEIGHT 240
#define DEGREES_PER_REVOLUTION 360.0
#define TILE_WIDTH 23.6220472441
#define TILE_DIAG_LENGTH 33.4066195836 // Calculated with the Pythagorean theorem

// NOT using black robot = Using green robot
#define USING_BLACK_ROBOT true
#define TESTING_AUTONOMOUS false 

#define SENSITIVITY 1 // 3-10 works good, currently undergoing testing // Higher is more sensitivity
#define DRIVE_WHEEL_DIAMETER 3.25
#define TRACKING_WHEEL_DIAMETER 1.959
#define DISTANCE_LEFT_TRACKING_WHEEL_CENTER 1.572
#define DISTANCE_RIGHT_TRACKING_WHEEL_CENTER 1.572
#define DISTANCE_BACK_TRACKING_WHEEL_CENTER 1.572
#define MOTOR_TO_DRIVE_RATIO .8 // NumTeethMotorGear / NumTeethWheelGear
#define GYRO_ENABLED true
#define GYRO_CONFIDENCE 1
#define GYRO_FILTER_LENGTH 1
#define ENCODER_CONFIDENCE 0
#define OFFSET_X_ENCODER_MID 3.250

// ============================================================================
//   _____  _    _  _ _  __  ___  ___ _____   _____ 
//  |_   _|/_\  | \| | |/ / |   \| _ \_ _\ \ / / __|
//    | | / _ \ | .` | ' <  | |) |   /| | \ V /| _| 
//    |_|/_/ \_\|_|\_|_|\_\ |___/|_|_|___| \_/ |___|
//
// ============================================================================

// Tank Drive Physical Constants
#define TANK_TRACK_WIDTH 12.15                    // distance between wheels
#define TANK_WHEELBASE_LENGTH 9.0                 // distance from back to front wheel  
#define TANK_ROTATION_OFFSET 0.0                  // clockwise rotation offset in degrees

// Tank Drive Motion Profile Constants
#define TANK_MAX_VELOCITY 600.0                   // Maximum RPM
#define TANK_DEFAULT_WHEEL_CENTER 7.55            // Default wheel center distance

// Tank Drive Safety Limits
#define TANK_MIN_VELOCITY 0.0
// [refactor-note]: TANK_MAX_VELOCITY ya definido arriba, eliminando duplicación
// #define TANK_MAX_VELOCITY 600.0
#define TANK_MIN_DISTANCE -1000.0                 // inches
#define TANK_MAX_DISTANCE 1000.0                  // inches
#define TANK_MIN_ANGLE -720.0                     // degrees
#define TANK_MAX_ANGLE 720.0                      // degrees
#define TANK_MIN_SPEED -1.0
#define TANK_MAX_SPEED 1.0

// Tank Drive PID Default Values
#define TANK_DRIVE_PID_KP 0.02
#define TANK_DRIVE_PID_KI 0.0
#define TANK_DRIVE_PID_KD 0.0
#define TANK_TURN_PID_KP 0.002
#define TANK_TURN_PID_KI 0.0
#define TANK_TURN_PID_KD 0.0

#define MAX_RPM 600.0
#define DRIVE_LENGTH 9.0
#define DRIVE_WIDTH 12.15
#define AVG_DRIVETRAIN_RADIUS 7.55
#define CLOCKWISE_ROTATION_DEGREES_OFFSET 0.0

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

#define INTAKE_VELOCITY (int)intake.getGearing() * .8

/**
 * \brief Maximum acceleration without slippage 
 * 
 * \see https://www.desmos.com/calculator/uuiafowges
 */ 
#define MAX_ACCEL 4991.46340024

/**
 * \brief Maximum deceleration without tipping 
 * 
 * \see https://www.desmos.com/calculator/kgdezwvxrr
 */
#define MAX_DECEL 206.258818192

#define DISTANCE 90 // This is the activation distance for the intake (the distance under which we know a ring is waiting in front of the intake)

#define ORBIT_HEIGHT 12.5
// ORBIT Limiting to protect when it does not have 360° of freedom
#define ORBIT_LIMITED true
#define ORBIT_LEFT_LIMIT 210
#define ORBIT_RIGHT_LIMIT 90

#endif  // AON_CONSTANTS_HPP_
