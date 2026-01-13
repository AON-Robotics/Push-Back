#pragma once

#include "../include/aon/tank-drive/tank-drive.hpp"

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../controls/trapezoid-profile/trapezoid.hpp"
#include "../controls/exponential-profile.hpp"

/**
 * \file h-drive.hpp
 *
 * \brief Functions accord to a H Drive Train
 * 
 * */

namespace aon {

// TODO: move this to the Odom class file

class PoseH {
 public:
  /// @brief Position of the robot on the x-axis in \b `inches` with respect to the field using (0,0) as the center of the field
  double x;
  /// @brief Position of the robot on the y-axis in \b `inches` with respect to the field using (0,0) as the center of the field
  double y;
  /// @brief Orirentation of the robot in \b `radians` with respect to angle 90º in the VEX Field
  double theta;

  PoseH(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
};

class HDrive : public TankDrive {
 private:
  // Motor
  okapi::Motor middleMotor;
  
  // Odometry
  PoseH pose;
  
  public:
  
  // Constructor
  HDrive(const okapi::Motor& mid = okapi::Motor(17))
    : TankDrive(),
      middleMotor(mid),
      pose() {}


  // Controller function
  /// @brief Configures the general settings for the motors
  /// @param brakeMode The braking paradigm we will use, usually `holding` for
  /// auton and `brake` for drivers
  /// @param gearset The gearbox the physical motors contain, they MUST be all
  /// the same
  void configure(okapi::AbstractMotor::brakeMode brakeMode,
                 okapi::AbstractMotor::gearset gearset) override;

  /// @brief Move robot with the controller. Holonomic motion with left joysick
  /// and turning with right.
  /// @param horizontal Velocity in horizontal motion
  /// @param vertical Velocity in vertical motion 
  /// @param turn Velocity for turning
  /// @param nothing A fill parameter
  void drive(double horizontal, double vertical, double turn, double nothing);

  /// @brief Move robot with the controller. Holonomic motion with left joysick
  /// and turning with right.
  /// @param horizontal Velocity in horizontal motion
  /// @param vertical Velocity in vertical motion 
  /// @param turn Velocity for turning
  /// @param nothing A fill parameter
  void driveV2(double horizontal, double vertical, double turn, double nothing, bool middleRight, bool middleLeft);
  
  // Movement functions

  /// @brief Stop all the motors for H drive train
  void stop() override;

  /// @brief Moves mid motor the same `rpm` to move forward
  /// @param rpm The speed in which to move all motors in \b rpm
  void motorsMid(const double &rpm);

  /// @brief Move robot in 2 directions at the same time and turn.
  /// @param x Movement in x axis in inches.
  /// @param y Movement in y axis in inches.
  /// @param theta Turn in degrees.
  void move2D(double x, double y, double theta);

  /// @brief Move horizontally to given distance using PID (default right)
  /// @param pid The PID used for the driving
  /// @param dist The distance to be moved in \b inches
  /// @param MAX_REVS The maximum RPM to send to the movement
  void moveHorizontalPID(double dist, PID pid = PID(0.02, 0, 0), const double &MAX_REVS=100);

  /// @brief Move horizontally using Motion profile (default right)
  /// @param dist The distance to be moved in \b inches
  void moveHorizontalProfiled(double dist);

  /// @brief Empty function for 
  /// @param t Time to run empty function.
  static void emptyFunction(int t);

  /// @brief Determines the speed of the robot given drivetrain motors' `RPM`
  /// @param RPM The RPM for which to calculate the velocity (default max RPM)
  /// @return The speed in \b in/s at which the robot would move at the given RPM
  /// @note Test the accuracy precision of the `getActualVelocity()` method which is used as a default value,
  /// @note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
  /// @note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`.
  // inline double getSpeed(const double &RPM = (int)driveFull.getActualVelocity()){
  double getSpeed(const double &RPM = MAX_RPM);

  /// @brief Helper function to calculate the time limit
  double computeTimeout(double distance, double maxVel, double maxAccel);

  /**
   * \brief Move drive at desired velocity with respect to plane of reference
   *
   * \details When used with odometry, moves the drive in the corresponding X or Y
   * component and rotates it using the field as a plane of reference. Therefore,
   * motions only depend on the plane and it's coordinates, not on the current
   * orientation
   *
   * \param vx Linear velocity in the X component (relative to reference plane)
   * \param vy Linear velocity in the Y component (relative to reference plane)
   * \param vT Angular velocity with respect to the robot's center of rotation
   * \param use_odom Use odometry for external reference plane
   *
   * \note
   *  Do not use odometry when function is used just to move drive taking
   * advantage of this function's vector addition for velocities
   *
   * \attention
   *    Positive (+) X is to the "right" and positive (+) Y to the "top" like a
   * normal Cartesian plane
   */
  void MoveHolonomicMotionH(double vx, double vy, double vT, bool use_odom = true);

  /**
 * \brief Move drive using Trapezoid Speed Profile
 *
 * \param X Target X position
 * \param Y Target Y positionM
 * \param T Target angular position in \b degrees
 * \param timeout Maximum amount of time function will block for in \b seconds
 * \param max_accel Desired maximum acceleration for trapezoidal motion
 * \param function Function of `t` that will run once every iteration
 * \param v0 Initial speed for trapezoidal motion
 * \param vf Final speed for trapezoidal motion
 * \param max_speed Maximum speed throughout trapezoidal motion
 * \param max_accel Absolute maximum acceleration and deceleration for
trapezoidal motion
 *
 *
 * */
void MoveTrapezoidH(double X, double Y, double T,
                   double max_accel = MAX_ACCEL,
                   std::function<void(int)> function = emptyFunction,
                   double v0 = DEFAULT_INITIAL_SPEED,
                   double vf = DEFAULT_FINAL_SPEED,
                   double max_speed = MAX_ACCEL,
                   PID drivePID = PID(0.02, 0, 0), PID turnPID = PID(0.002, 0, 0));
};
} // namespace aon