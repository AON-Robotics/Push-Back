#pragma once

#include "../include/aon/tank-drive/tank-drive.hpp"

/**
 * \file h-drive.hpp
 *
 * \brief Functions accord to a H Drive Train
 * 
 * \note The function move2D() is commented because its hard to make it part of
 * the class, because it needs the drive and turn PID declare in globals.hpp. If we 
 * include globals, it will create a dependencies problem. To use holonomic motion
 * include the file holonomic-motion-h.hpp wherever you need it and use it function.
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
  
  // Controller
  // pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);
  
  // Odometry
  PoseH pose;
  
  public:
  
  // Constructor
  HDrive(const okapi::Motor& mid = okapi::Motor(1))
    : TankDrive(),
      middleMotor(mid),
      pose() {}

  // Helper functions 
  // inline double AnalogInputScaling(const double x, const double t);

  // Controller function

  /// @brief Move robot with the controller. Holonomic motion with left joysick
  /// and turning with right.
  // void opcontrol();
  
  // Movement functions

  /// @brief Stop all the motors for H drive train
  void stop() override;

  /// @brief Moves mid motor the same `rpm` to move forward
  /// @param rpm The speed in which to move all motors in \b rpm
  void motorsMid(const double &rpm);

  /// @brief Use holonomic motion to move in 2 directions and turn at the same time.
  /// @param x Position in x we want to move
  /// @param y Position in y we want to move
  /// @param t Theta we want to turn
  // void move2D(double x, double y, double t = aon::odometry::GetDegrees());

  /// @brief Move horizontally to given distance using PID (default right)
  /// @param pid The PID used for the driving
  /// @param dist The distance to be moved in \b inches
  /// @param MAX_REVS The maximum RPM to send to the movement
  void moveHorizontalPID(PID pid, double dist, const double &MAX_REVS);

  /// @brief Move horizontally using Motion profile (default right)
  /// @param dist The distance to be moved in \b inches
  void moveHorizontalProfiled(double dist);
};
} // namespace aon