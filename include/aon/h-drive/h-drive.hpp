#pragma once

#include "../include/aon/tank-drive/tank-drive.hpp"

#include <cmath>
#include <algorithm>
#include "../constants.hpp"
#include "../controls/trapezoid-profile/trapezoid.hpp"
#include "../controls/exponential-profile.hpp"
#include "../controls/s-curve-profile.hpp"

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
  HDrive(const std::initializer_list<okapi::Motor> &leftPorts = {0},
            const std::initializer_list<okapi::Motor> &rightPorts = {0},
            const okapi::Motor &mid = okapi::Motor(0))
    : TankDrive(leftPorts, rightPorts),
      middleMotor(mid),
      pose() {}

  PoseH getPose() { return this->pose; }
  void setPose(PoseH p) { pose = p; }

  double getX() { return this->pose.x; }
  void setX(double x) { pose.x = x; }

  double getY() { return this->pose.y; }
  void setY(double y) { pose.y = y; }

  double getTheta() { return this->pose.theta; }
  void setTheta(double theta) { pose.theta = theta; }

  /// @brief Move robot with the controller. Holonomic motion with left joysick
  /// and turning with right.
  /// @param horizontal Velocity in horizontal motion
  /// @param vertical Velocity in vertical motion 
  /// @param turn Velocity for turning
  /// @param nothing A fill parameter
  void drive(double leftX, double leftY, double rightX, double rightY);

  /// @brief Stop all the motors for H drive train
  void stop() override;

  /// @brief Moves mid motor the same `rpm` to move sideways
  /// @param rpm The speed in which to move all motors in \b rpm
  void motorMid(const double &rpm);

  /// @brief Move horizontally to given distance using PID (default right)
  /// @param pid The PID used for the driving
  /// @param dist The distance to be moved in \b inches
  /// @param MAX_REVS The maximum RPM to send to the movement
  void moveHorizontalPID(double dist, PID pid = PID(0.02, 0, 0), const double &MAX_REVS=100);

  /// @brief Move horizontally using Motion profile (default right)
  /// @param dist The distance to be moved in \b inches
  void strafe(double dist);

  /**
  * \brief Holonomic motion with motion profile. Move in x, y and theta at the same time.
  *
  * \param X Target X position
  * \param Y Target Y positionM
  * \param T Target angular position in \b degrees
  * \param max_speed Maximum speed throughout trapezoidal motion
  * \param max_accel Desired maximum acceleration for trapezoidal motion
  * \param max_speed_angular Desired maximum angular speed for trapezoidal motion
  * \param max_accel_angular Desired maximum angular acceleration for trapezoidal motion
  * \param drivePID Drive PID to correct motion
  * \param turnPID Turn PID to correct motion
  *
  * */
  void HolonomicMotion(
    double X, double Y, double T,
    double max_speed = MAX_RPM, // in/s to RPM
    double max_accel = MAX_ACCEL, // RPM/s
    double max_deccel = MAX_DECEL,
    PID drivePID = PID(0.02, 0, 0), PID turnPID = PID(0.002, 0, 0)
  );
  // void HolonomicMotion(
  //   double X, double Y, double T,
  //   double max_speed = (MAX_LINEAR_VELOCITY * 60) / (M_PI / (DRIVE_WHEEL_DIAMETER / 2)), // in/s to RPM
  //   double max_accel = MAX_ACCEL, // RPM/s
  //   double max_ang_speed = MAX_ANGULAR_VELOCITY * (60 / (2 * M_PI)), // rad/s to RPM 
  //   double max_ang_accel = MAX_ANGULAR_ACCEL * (60 / (2 * M_PI)), // rad/s^2 to RPM/s
  //   PID drivePID = PID(0.02, 0, 0), PID turnPID = PID(0.002, 0, 0)
  // );

  /// @brief Move robot in 2 directions at the same time and turn.
  /// @param x Movement in x axis in inches.
  /// @param y Movement in y axis in inches.
  /// @param theta Turn in degrees.
  void goToPose(double x, double y, double theta);

};
} // namespace aon