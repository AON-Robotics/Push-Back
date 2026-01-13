// /**
//  * \file holonomic_motion.hpp
//  *
//  * \brief Implementation of holonomic motion and some control algorithms that
//  * make use of it.
//  * */
// #pragma once

// #include <cmath>
// #include <algorithm>
// #include "../constants.hpp"
// #include "../aon/controls/trapezoid-profile/trapezoid.hpp"
// #include "../aon/controls/exponential-profile.hpp"
// #include "../sensing/odometry.hpp"
// #include "../globals.hpp"

// namespace aon::holonomic_motionH {

// /**
//  * \brief Move drive at desired velocity with respect to plane of reference
//  *
//  * \details When used with odometry, moves the drive in the corresponding X or Y
//  * component and rotates it using the field as a plane of reference. Therefore,
//  * motions only depend on the plane and it's coordinates, not on the current
//  * orientation
//  *
//  * \param vx Linear velocity in the X component (relative to reference plane)
//  * \param vy Linear velocity in the Y component (relative to reference plane)
//  * \param vT Angular velocity with respect to the robot's center of rotation
//  * \param use_odom Use odometry for external reference plane
//  *
//  * \note
//  *  Do not use odometry when function is used just to move drive taking
//  * advantage of this function's vector addition for velocities
//  *
//  * \attention
//  *    Positive (+) X is to the "right" and positive (+) Y to the "top" like a
//  * normal Cartesian plane
//  */
// void MoveHolonomicMotionH(double vx, double vy, double vT,
//                          bool use_odom = true) {

//   const double d = DRIVE_WIDTH / 2.0; // Half the width of the robot
//   const double r = DRIVE_WHEEL_DIAMETER / 2.0; // Wheel radio

//   // Odometry uses CW as positive while conventionally CCW is positive
//   const double theta = (use_odom) ? -aon::odometry::GetRadians() : 0.0; // orientation relative to the field

//   // Accounts for rotation
//   // d converts the angular velocity (vT) into linear velocity for the wheel
//   // physics v (wheel) = w (angular velocity) x r (distance from center)
//   const double rotation = -d * vT;

//   // Use the rotation matrix to transform field-oriented velocities into robot relative
//   // Use sin(45) because the wheel in X drive train are in 45 degrees
//   const double vx_field = vx * std::cos(theta) + vy * std::sin(theta);
//   const double vy_field = vy * std::cos(theta) - vx * std::sin(theta);

//   // Compute wheel linear velocities (inches/sec)
//   // We apply the rotation to the right and left motors because they are the ones
//   // in charge to do the rotation during the movement of the robot
//   const double vl = vy_field + rotation;  // left wheel
//   const double vr = vy_field - rotation;  // right wheel
//   const double vm = vx_field;             // middle wheel

//   // Compute conversion factor in order to get RPMs.
//   // 2*PI*WheelRadius = 1rev, 60 secs = 1 min
//   // in per s * (1 rev / (2*pi*WheelRadius) in) * (60 s / 1 min) = rev per min
//   const double INPS2RPM = 60.0 / (M_PI * DRIVE_WHEEL_DIAMETER);

//   // Move motors
//   drivetrainH.motorsLeft(vl * INPS2RPM);
//   drivetrainH.motorsRight(vr * INPS2RPM);
//   drivetrainH.motorsMid(vm * INPS2RPM);
// }

// inline void emptyFunction(int t) {}

// // Helper function to calculate the time limit
// double computeTimeout(double distance, double maxVel, double maxAccel) {
//     double t_ideal = (distance / maxVel) + (maxVel / maxAccel);
//     return t_ideal;
// }

// /**
//  * \brief Move drive using Trapezoid Speed Profile
//  *
//  * \param X Target X position
//  * \param Y Target Y position
//  * \param T Target angular position in \b degrees
//  * \param timeout Maximum amount of time function will block for in \b seconds
//  * \param max_accel Desired maximum acceleration for trapezoidal motion
//  * \param function Function of `t` that will run once every iteration
//  * \param v0 Initial speed for trapezoidal motion
//  * \param vf Final speed for trapezoidal motion
//  * \param max_speed Maximum speed throughout trapezoidal motion
//  * \param max_accel Absolute maximum acceleration and deceleration for
// trapezoidal motion
//  *
//  *
//  * */
// void MoveTrapezoidH(double X, double Y, double T,
//                    double max_accel = MAX_ACCEL,
//                    std::function<void(int)> function = emptyFunction,
//                    double v0 = DEFAULT_INITIAL_SPEED,
//                    double vf = DEFAULT_FINAL_SPEED,
//                    double max_speed = MAX_VELOCITY_LINEAR,
//                    PID drivePID = drivePID, PID turnPID = turnPID) {

//   const double d = DRIVE_WIDTH / 2.0; // Half the width of the robot

//   // Instantiate Trapezoid objects
//   // Its a motion profile that the acceleration and desacceletarion looks like a trapezoid
//   aon::TrapezoidProfile xMotionProfile = aon::TrapezoidProfile();
//   aon::TrapezoidProfile yMotionProfile = aon::TrapezoidProfile();
//   aon::ExponentialProfile TMotionProfile = aon::ExponentialProfile(); // better for rotation
//   drivePID.Reset();
//   turnPID.Reset();

//   // Instantiate PID objects
//   PID vxPID = drivePID;
//   PID vyPID = drivePID;
//   PID thetaPID = turnPID;

//   // Store initial conditions
//   const double start_x = aon::odometry::GetX();
//   const double start_y = aon::odometry::GetY();
//   const double start_theta = -aon::odometry::GetDegrees();   // Odometry uses CW as positive, but we want CCW to be positive
//   const double start_time = pros::millis() / 1000.0;
//   double last_t = start_time;

//   // Determine how much base should move in each component
//   const double dx = X - start_x;
//   const double dy = Y - start_y;
//   const double dT = T - start_theta;

//   // Decompose max_speed and max_accel magnitudes into x and y components
//   // using angle made by dx and dy as angle for the vector
//   const double cos_max_speed = fabs(dx / std::hypot(dx, dy));
//   const double sin_max_speed = fabs(dy / std::hypot(dx, dy));
//   // const double cos_max_speed = fabs(std::cos(std::atan2(dy, dx)));
//   // const double sin_max_speed = fabs(std::sin(std::atan2(dy, dx)));

//   const double max_vx = max_speed * cos_max_speed;
//   const double max_vy = max_speed * sin_max_speed;
//   const double max_vt = max_speed / d;
//   const double v0x = v0 * cos_max_speed;
//   const double v0y = v0 * sin_max_speed;
//   const double vfx = vf * cos_max_speed;
//   const double vfy = vf * sin_max_speed;
//   const double max_ax = max_accel * cos_max_speed;
//   const double max_ay = max_accel * sin_max_speed;

//   double timeoutX = computeTimeout(std::abs(dx), max_vx, max_ax);
//   double timeoutY = computeTimeout(std::abs(dy), max_vy, max_ay);
//   double timeoutTheta = computeTimeout(std::abs(dT), max_vt, max_accel); /// FIND MAX FOR THETA

//   // pick the largest
//   double timeout = std::max({timeoutX, timeoutY, timeoutTheta});

//   pros::lcd::print(0, "Before setting position parameters");
//   xMotionProfile.SetParams(dx, max_vx, v0x, vfx, max_ax, max_ax, 0.5);
//   yMotionProfile.SetParams(dy, max_vy, v0y, vfy, max_ay, max_ax, 0.5);

//   pros::lcd::print(1, "Before setting angle parameters");
//   TMotionProfile.SetParams(dT, timeout / 2.0);

//   // vx, vy and vTheta dictated by the s_curve computations
//   #define vx xMotionProfile.SpeedProfile(tx)
//   #define vy yMotionProfile.SpeedProfile(ty)
//   #define vT TMotionProfile.SpeedProfile(tT)


//   // Initialize timing variables
//   double tx = FLT_EPSILON;
//   double ty = FLT_EPSILON;
//   double tT = FLT_EPSILON;
//   double t = pros::millis() / 1000.0 - start_time;

//   pros::lcd::print(2, "Before Loop");
//   // Run loop while time hasn't run out
//   while (t < timeout) {
//     // Run moveHolonomicMotion function with odometry
//     std::cout << "vx = " << vx << ", vy = " << vy << ", vT = " << vT << std::endl;
//     double t = pros::millis() / 1000.0 - start_time; // get time
//     double dt = t - last_t; // calculate dt
//     last_t = t; // save for next iteration

//     // Compute feedforward velocities (vx, vy, vT)
//     double vx_ff = vx;
//     double vy_ff = vy;
    
//     // Convert vT to rad/s for MoveHolonomicMotion
//     double vT_ff_deg = vT;
//     double vT_ff = vT_ff_deg * M_PI / 180.0;

//     // Current change
//     Vector current_position = aon::odometry::GetPosition();
//     double current_dT = aon::odometry::GetDegrees() - start_theta;
//     double current_dx = current_position.GetX() - start_x;
//     double current_dy = current_position.GetY() - start_y;

//     // Apply PID corrections
//     double vx_corr = vxPID.OutputDt(dx, current_dx, dt);
//     double vy_corr = vyPID.OutputDt(dy, current_dy, dt);
//     double vT_corr = thetaPID.OutputDt(dT, current_dT, dt);
    
//     // Convert angular correction from deg/s to rad/s
//     vT_corr *= M_PI / 180.0;

//     // Final command = feedforward + correction
//     MoveHolonomicMotionH(vx_ff + vx_corr,
//                         vy_ff + vy_corr,
//                         vT_ff + vT_corr,
//                         true);

//     // Run custom user function at the current time (millisecond)
//     function(t * 1000);

//     pros::delay(10);

//     // aon::odometry::Update();

//     // Update these values BEFORE while loop comparison
//     t = pros::millis() / 1000.0 - start_time;

//     pros::lcd::print(3, "Before Inverses");
//     tx = xMotionProfile.PositionProfileInverse(
//         fabs(aon::odometry::GetX() - start_x));
//     ty = yMotionProfile.PositionProfileInverse(
//         fabs(aon::odometry::GetY() - start_y));
//     tT = TMotionProfile.PositionProfileInverse(
//         fabs(aon::odometry::GetDegrees() - start_theta));
//     pros::lcd::print(3, "After Inverses");
//   }

//   pros::lcd::print(4, "After Loop");
//   MoveHolonomicMotionH(0, 0, 0, false);
// #undef vx
// #undef vy
// #undef vT
// }

// };  // namespace aon::holonomic_motion
