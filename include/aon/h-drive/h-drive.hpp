#pragma once

#include "../tank-drive.hpp"

#include "./constants.hpp"
#include "holonomic-motion-h.hpp" 

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
  pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);

  // Odometry
  PoseH pose;

 public:
  // Constructor
  HDrive(const okapi::Motor mid = {1}) : 
    TankDrive(const std::initializer_list<okapi::Motor> &leftPorts = {-20, 19, -18},
              const std::initializer_list<okapi::Motor> &rightPorts = {9, -8, 7}), 
    middleMotor(mid);

  // Helper functions 
  inline double AnalogInputScaling(const double x, const double t);

  // Controller function
  /// @brief Move robot with the controller. Holonomic motion with left joysick
  /// and turning with right.
  void opcontrol();
  
  // Movement functions
  /// @brief Use holonomic motion to move in 2 directions and turn at the same time.
  /// @param x Position in x we want to move
  /// @param y Position in y we want to move
  /// @param t Theta we want to turn
  void move2D(double x, double y, double t = aon::odometry::GetDegrees());

  /// @brief Move horizontally using 
  /// @param dist Distance to move horizontally
  void moveHorizontal(double dist);
};
}