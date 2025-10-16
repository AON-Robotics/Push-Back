#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../odometry/odometry.hpp"

#include "../math/misc/misc.hpp"
#include "../controls/pid/pid.hpp"
#include <cfloat>
#include "../math/pose.hpp"

namespace aon {

// TODO: move this to the Odom class file



class TankDrive {
 private:
  Odometry odometry;
  okapi::MotorGroup leftMotors;
  okapi::MotorGroup rightMotors;
  MotionProfile motionProfile;
  Pose pose;

  // TODO: add the odom object once it is done, use namespace temporarily

 public:
  TankDrive(const std::initializer_list<okapi::Motor> &leftPorts = {-20, 19, -18},
            const std::initializer_list<okapi::Motor> &rightPorts = {9, -8, 7})
      : leftMotors(leftPorts),
        rightMotors(rightPorts),
        motionProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL),
        pose(),
        odometry() {}

  /// @brief Moves all motors the same `rpm` to move forward
  /// @param rpm The speed in which to move all motors in \b rpm
  void motors(const double &rpm);

  /// @brief Moves all motors the same `rpm` to rotate clockwise
  /// @param rpm The speed in which to move all motors in \b rpm
  void rotate(const double &rpm);

  /// @brief Moves the robot forward while also turning
  /// @param forward The \b RPM to send to the motors for linear movement
  /// (positive is forward)
  /// @param turn The \b RPM to send to the motors for rotative movement
  /// (positive is clockwise)
  void driveWhileTurning(const double &forward, const double &turn);

  Pose getPose() { return this->pose; }
  void setPose(Pose p) { pose = p; }

  double getX() { return this->pose.x; }
  void setX(double x) { pose.x = x; }

  double getY() { return this->pose.y; }
  void setY(double y) { pose.y = y; }

  double getTheta() { return this->pose.theta; }
  void setTheta(double theta) { pose.theta = theta; }

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param midSpeed The speed with which to drive in \b RPM (positive speed
  /// will go forward and negative speed will go backwards)
  /// @note A positive `radius` will cause a clockwise rotation, while a
  /// negative `radius` will cause a counter-clockwise rotation
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  void driveInArc(double radius, const double &midSpeed = 200);

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// for a given `angle`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param angle The angle of the arc we want to cover in \b degrees, a
  /// negative angle will cause the robot to go in reverse
  /// @note A positive `radius` will cause a rotation with reference to a point
  /// to the right, while a negative `radius` will cause a rotation with
  /// reference to a point to the left
  /// @note A positive `angle` will cause a forward movement, while a negative
  /// `angle` will cause a backwards movement
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  void driveAngleOfArc(const double &radius = DRIVE_WIDTH,
                       const double &angle = 90);

  /// @brief Makes the robot drive in an arc motion to a specified point in the
  /// field
  /// @param x The x coordinate of the point we want to go to in \b meters
  /// @param y The y coordinate of the point we want to go to in \b meters
  /// @note Odometry must be working for global positioning on the field
  /// @see https://www.desmos.com/calculator/5abb373276
  void driveInArcTo(const double &x, const double &y);

  /// @brief Drives the robot in the direction of the left joystick while
  /// turning it with the right joystick
  /// @param leftX The value of the left joystick on the x-axis in the range
  /// [-1, 1]
  /// @param leftY The value of the left joystick on the y-axis in the range
  /// [-1, 1]
  /// @param rightX The value of the right joystick on the x-axis in the range
  /// [-1, 1]
  /// @param rightY The value of the right joystick on the y-axis in the range
  /// [-1, 1]
  void drive(double leftX, double leftY, double rightX, double rightY);

  /// @brief Stops all motors
  void stop();

  /// @brief Configures the general settings for the motors
  /// @param brakeMode The braking paradigm we will use, usually `holding` for
  /// auton and `brake` for drivers
  /// @param gearset The gearbox the physical motors contain, they MUST be all
  /// the same
  void configure(okapi::AbstractMotor::brakeMode brakeMode,
                 okapi::AbstractMotor::gearset gearset);

  /// @brief Calculates average RPM forward
  /// @return The RPM of the motors with respect to the front of the robot
  double getRPM();

  /// @brief Moves the robot a given distance (default forward)
  /// @param pid The PID used for the driving
  /// @param dist The distance to be moved in \b inches
  /// @param MAX_REVS The maximum RPM to send to the movement
  void drivePID(PID pid = PID(0.02, 0, 0), double dist = TILE_WIDTH,
                const double &MAX_REVS = 100.0);

  /// @brief Turns the robot by a given angle (default clockwise)
  /// @param pid The PID to be used for the turn
  /// @param angle The angle to make the robot turn in \b degrees
  /// @param MAX_REVS The maximum RPM to send to the movement
  void turnPID(PID pid = PID(0.002, 0, 0), double angle = 90,
               const double &MAX_REVS = 50.0);

  /// @brief S-graph motion profile for linear movement
  /// @param dist The distance to be moved in \b inches, positive values will
  /// move forward and negative values backwards
  void driveProfiled(double dist = TILE_WIDTH);

  /// @brief S-graph motion profile for rotations
  /// @param angle The angle in \b degrees we wish to rotate the robot, positive
  /// is clockwise and negative is counter-clockwise
  void turnProfiled(double angle = 90);

  /// @brief Moves the robot a given distance
  /// @param dist The distance to move in \b inches
  /// @details A positive `dist` makes the robot go forward while a negative
  /// `dist` makes the robot go backwards
  void move(const double &dist = TILE_WIDTH);

  /// @brief Turn the robot a given angle (default is clockwise)
  /// @param angle The angle to turn in \b degrees
  /// @details Clockwise is positive and counter-clockwise is negative
  void turn(const double &angle = 90);

  /// @brief Sets the max velocity for the drivetrains motion profile
  /// @param rpm The max velocity in \b RPM to pass to the motion profile
  void setMaxVelocity(const double &rpm);

  /// @brief Calculates the target velocity to send to the motors for smooth and
  /// precise movements using an S-curve profile.
  /// @param distance The remaining distance to the target in \b inches.
  /// @param dt The time elapsed since the last function call in \b seconds.
  /// @return The updated velocity in \b RPM.
  double updateProfile(const double &distance, const double &dt);

  /// @brief Turns the robot towards a specific direction
  /// @param x The x component of the point we wish to face
  /// @param y The y component of the point we wish to face
  /// @note Uses coordinate system from GPS in \b meters
  void turnTo(const double &x, const double &y);

  /// @brief Goes to the target point
  /// @param x The x component of the place where we want to go using the gps
  /// coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @param y The y component of the place where we want to go using the gps
  /// coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @note Uses coordinate system from GPS in \b meters
  void goTo(const double &x, const double &y);
};
}  // namespace aon
