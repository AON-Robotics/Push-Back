// /**
//  * \file holonomic_motion.hpp
//  *
//  * \brief Implementation of holonomic motion and some control algorithms that
//  * make use of it.
//  * */
// #if !USING_BLACK_ROBOT

// #pragma once

// #include <cmath>
// #include <algorithm>
// #include "../constants.hpp"
// #include "./trapezoid-profile/trapezoid.hpp"
// #include "./exponential-profile.hpp"
// #include "../sensing/odometry.hpp"
// #include "../globals.hpp"

// namespace aon::holonomic_motion {

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
// void MoveHolonomicMotion(double vx, double vy, double vT,
//                          bool use_odom = true) {
//   const double kSin45 = M_SQRT2 / 2.0;
//   const double d = DRIVE_WIDTH / 2.0;
//   const double r = DRIVE_WHEEL_DIAMETER / 2.0;

//   // Odometry uses CW as positive while conventionally CCW is positive
//   const double phi = (use_odom) ? -aon::odometry::GetRadians() : 0.0;

//   const double term1 = -d * vT;
//   const double term2 = kSin45 * (vx * std::cos(phi) + vy * std::sin(phi));
//   const double term3 = kSin45 * (vy * std::cos(phi) - vx * std::sin(phi));

//   // Compute velocity for each of the wheels (inches per second)
//   const double u1 = (1.0 / r * (term1 + term2 + term3));
//   const double u2 = (1.0 / r * (term1 - term2 + term3));
//   const double u3 = (1.0 / r * (term1 + term2 - term3));
//   const double u4 = (1.0 / r * (term1 - term2 - term3));

//   // Compute conversion factor in order to get RPMs.
//   // 2*PI*WheelRadius = 1rev, 60 secs = 1 min
//   // in per s * (1 rev / (2*pi*WheelRadius) in) * (60 s / 1 min) = rev per min
//   const double INPS2RPM = 60.0 / (M_PI * DRIVE_WHEEL_DIAMETER);

//   drive_front_left.moveVelocity(u1 * INPS2RPM);
//   drive_back_left.moveVelocity(u2 * INPS2RPM);
//   drive_front_right.moveVelocity(-u3 * INPS2RPM);
//   drive_back_right.moveVelocity(-u4 * INPS2RPM);
// }

// inline void emptyFunction(int t) {}

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
//   \code{.cpp}
// // Move back while closing pneumatics and dropping some balls
// MoveTrapezoid(12.0, 12.0, -45.0, 10.0, [](int t){
//     if(t <= 100){
//       close_pneumatics();
//     } else if(t <= 500){
//       set_intake(-50);
//     } else{
//       stop_intake();
//     }
//   });

//   \endcode
//  *
//  * */
// void MoveTrapezoid(double X, double Y, double T, double timeout,
//                    double max_accel = MAX_ACCELERATION,
//                    std::function<void(int)> function = emptyFunction,
//                    double v0 = DEFAULT_INITIAL_SPEED,
//                    double vf = DEFAULT_FINAL_SPEED,
//                    double max_speed = MAX_SPEED) {
//   // Instantiate Trapezoid objects
//   aon::TrapezoidProfile xMotionProfile = aon::TrapezoidProfile();
//   aon::TrapezoidProfile yMotionProfile = aon::TrapezoidProfile();
//   aon::ExponentialProfile TMotionProfile = aon::ExponentialProfile();

//   // Store initial conditions
//   const double start_x = aon::odometry::GetX();
//   const double start_y = aon::odometry::GetY();
//   // Odometry uses CW as positive, but we want CCW to be positive
//   const double start_Theta = -aon::odometry::GetDegrees();

//   // Determine how much base should move in each component
//   const double dx = X - start_x;
//   const double dy = Y - start_y;
//   const double dT = T - start_Theta;

//   // Decompose max_speed and max_accel magnitudes into x and y components
//   //   using angle made by dx and dy as angle for the vector
//   const double cos_max_speed = fabs(dx / std::hypot(dx, dy));
//   const double sin_max_speed = fabs(dy / std::hypot(dx, dy));
//   // const double cos_max_speed = fabs(std::cos(std::atan2(dy, dx)));
//   // const double sin_max_speed = fabs(std::sin(std::atan2(dy, dx)));

//   const double max_vx = max_speed * cos_max_speed;
//   const double max_vy = max_speed * sin_max_speed;
//   const double v0x = v0 * cos_max_speed;
//   const double v0y = v0 * sin_max_speed;
//   const double vfx = vf * cos_max_speed;
//   const double vfy = vf * sin_max_speed;
//   const double max_ax = max_accel * cos_max_speed;
//   const double max_ay = max_accel * sin_max_speed;

//   pros::lcd::print(0, "Before setting position parameters");
//   xMotionProfile.SetParams(dx, max_vx, v0x, vfx, max_ax, max_ax, 0.5);
//   yMotionProfile.SetParams(dy, max_vy, v0y, vfy, max_ay, max_ax, 0.5);

//   pros::lcd::print(1, "Before setting angle parameters");
//   TMotionProfile.SetParams(dT, timeout / 2.0);

//   // vx, vy and vTheta dictated by the s_curve computations
// #define vx xMotionProfile.SpeedProfile(tx)
// #define vy yMotionProfile.SpeedProfile(ty)
// #define vT TMotionProfile.SpeedProfile(tT)

//   const double start_time = pros::millis() / 1000.0;

//   double tx = FLT_EPSILON;
//   double ty = FLT_EPSILON;
//   double tT = FLT_EPSILON;
//   double t = pros::millis() / 1000.0 - start_time;

//   aon::odometry::Update();

//   pros::lcd::print(2, "Before Loop");
//   // Run loop while time hasn't run out
//   while (t < timeout) {
//     // Run moveHolonomicMotion function with odometry
//     std::cout << "vx = " << vx << ", vy = " << vy << ", vT = " << vT
//               << std::endl;
//     MoveHolonomicMotion(vx, vy, vT * M_PI / 180.0, true);

//     // Run custom user function at the current time (millisecond)
//     function(t * 1000);

//     pros::delay(10);

//     aon::odometry::Update();

//     // Update these values BEFORE while loop comparison
//     t = pros::millis() / 1000.0 - start_time;

//     pros::lcd::print(3, "Before Inverses");
//     tx = xMotionProfile.PositionProfileInverse(
//         fabs(aon::odometry::GetX() - start_x));
//     ty = yMotionProfile.PositionProfileInverse(
//         fabs(aon::odometry::GetY() - start_y));
//     tT = TMotionProfile.PositionProfileInverse(
//         fabs(aon::odometry::GetDegrees() - start_Theta));
//     pros::lcd::print(3, "After Inverses");
//   }

//   pros::lcd::print(4, "After Loop");
//   MoveHolonomicMotion(0, 0, 0, false);
// #undef vx
// #undef vy
// #undef vT
// }

// };  // namespace aon::holonomic_motion

// #endif
