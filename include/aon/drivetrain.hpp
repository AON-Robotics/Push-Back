#pragma once

#include "./okapi/api.hpp"
#include "./controls/s-curve-profile.hpp"
#include "./controls/pid/pid.hpp"
#include "./odometry/odometry.hpp"

namespace aon {


class Drivetrain {
 protected:
  std::unique_ptr<Odometry> odometry;
  Pose pose;
  bool turbo = false;
  
  public:

  Drivetrain(std::unique_ptr<Odometry> odom): pose(), odometry(std::move(odom)) {}


  virtual void initialize() = 0;
  

  Pose getPose() { return this->pose; }
  void setPose(Pose p) { this->pose = p; }

  double getX() { 
    return this->odometry->getX();
  }
  void setX(double x) { this->pose.x = x; }

  double getY() { 
    return this->odometry->getY();
  }
  void setY(double y) { this->pose.y = y; }

  double getTheta() { 
    return this->odometry->getDegrees();
  }
  void setTheta(double theta) { this->pose.theta = theta; }

  void resetPose(double x = 0.0, double y = 0.0, double theta = 0.0) {
  this->odometry->resetCurrent(x, y, theta);
  }


  bool isTurbo() { return this->turbo; }
  void setTurbo(bool turbo) { this->turbo = turbo; }
  void toggleTurbo() { this->turbo = !this->turbo; }

  /// @brief Moves all motors the same `rpm` to move forward
  /// @param rpm The speed in which to move all motors in \b rpm
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  virtual void motors(const double &rpm = MAX_RPM, const int& delay = 0) = 0;

  /// @brief Moves all motors the same `rpm` to rotate clockwise
  /// @param rpm The speed in which to move all motors in \b rpm
  virtual void rotate(const double &rpm) = 0;

  /// @brief Moves the robot forward while also turning
  /// @param forward The \b RPM to send to the motors for linear movement
  /// (positive is forward)
  /// @param turn The \b RPM to send to the motors for rotative movement
  /// (positive is clockwise)
  virtual void driveWhileTurning(const double &forward, const double &turn) = 0;

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param speed The speed with which to drive in \b RPM (positive speed
  /// will go forward and negative speed will go backwards)
  /// @note A positive `radius` will cause a clockwise rotation, while a
  /// negative `radius` will cause a counter-clockwise rotation
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  virtual void driveInArc(double radius, const double &speed = 200) = 0;

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// for a given `angle`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param angle The angle of the arc we want to cover in \b degrees, a
  /// negative angle will cause the robot to go in reverse
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @note A positive `radius` will cause a rotation with reference to a point
  /// to the right, while a negative `radius` will cause a rotation with
  /// reference to a point to the left
  /// @note A positive `angle` will cause a forward movement, while a negative
  /// `angle` will cause a backwards movement
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  virtual void driveAngleOfArc(const double &radius = DRIVE_WIDTH,
                               const double &angle = 90,
                               bool settle = true) = 0;

  /// @brief Makes the robot drive in an arc motion to a specified point in the
  /// field
  /// @param x The x coordinate of the point we want to go to in \b meters
  /// @param y The y coordinate of the point we want to go to in \b meters
  /// @note Odometry must be working for global positioning on the field
  /// @see https://www.desmos.com/calculator/5abb373276
  virtual void driveInArcTo(const double &x, const double &y) = 0;

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
  virtual void drive(double leftX, double leftY, double rightX,
                     double rightY) = 0;

  /// @brief Stops all motors
  virtual void stop() { this->motors(0); }

  /// @brief Configures the general settings for the motors
  /// @param brakeMode The braking paradigm we will use, usually `holding` for
  /// auton and `brake` for drivers
  /// @param gearset The gearbox the physical motors contain, they MUST be all
  /// the same
  /// @param slew The slew rate for the motors, if 0, slew rate is `inf`
  void configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset, double slew) {
    this->setBrakeMode(brakeMode);
    this->setGearset(gearset);
    this->setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);

    if(brakeMode == okapi::AbstractMotor::brakeMode::hold){
      this->setSlewRate(0);
    } else {
      this->setSlewRate(slew);
    }
  }

  /// @brief Sets the brake mode for all motors of the drivetrain
  /// @param brakeMode The new brake mode for the drivetrain
  virtual void setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode) = 0;

  /// @brief Sets the gearset for all motors of the drivetrain
  /// @param gearset The new gearset for the drivetrain
  virtual void setGearset(okapi::AbstractMotor::gearset gearset) = 0;

  /// @brief Sets the units for all encoders of the motors of the drivetrain
  /// @param units The new units for the drivetrain
  virtual void setEncoderUnits(okapi::AbstractMotor::encoderUnits units) = 0;

  /// @brief Sets the slew rate for all motors of the drivetrain
  /// @param slew The new slew rate for the drivetrain
  virtual void setSlewRate(double slew) = 0;

  /// @brief Calculates average RPM forward
  /// @return The RPM of the motors with respect to the front of the robot
  virtual double getRPM() = 0;

  /// @brief Moves the robot a given distance (default forward)
  /// @param pid The PID used for the driving
  /// @param dist The distance to be moved in \b inches
  /// @param MAX_REVS The maximum RPM to send to the movement
  virtual void drivePID(PID pid = PID(0.02, 0, 0), double dist = TILE_WIDTH,
                        const double &MAX_REVS = 100.0) = 0;

  /// @brief Turns the robot by a given angle (default clockwise)
  /// @param pid The PID to be used for the turn
  /// @param angle The angle to make the robot turn in \b degrees
  /// @param MAX_REVS The maximum RPM to send to the movement
  virtual void turnPID(PID pid = PID(0.002, 0, 0), double angle = 90,
                       const double &MAX_REVS = 50.0) = 0;

  /// @brief S-graph motion profile for linear movement
  /// @param dist The distance to be moved in \b inches, positive values will
  /// move forward and negative values backwards
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  virtual void driveProfiled(double dist = TILE_WIDTH, bool settle = true) = 0;

  /// @brief S-graph motion profile for rotations
  /// @param angle The angle in \b degrees we wish to rotate the robot, positive
  /// is clockwise and negative is counter-clockwise
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  virtual void turnProfiled(double angle = 90, bool settle = true) = 0;

  /// @brief Moves the robot a given distance
  /// @param dist The distance to move in \b inches
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details A positive `dist` makes the robot go forward while a negative
  /// `dist` makes the robot go backwards
  virtual void move(const double &dist = TILE_WIDTH, bool settle = true) = 0;

  /// @brief Turn the robot a given angle (default is clockwise)
  /// @param angle The angle to turn in \b degrees
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details Clockwise is positive and counter-clockwise is negative
  virtual void turn(const double &angle = 90, bool settle = true) = 0;

  /// @brief Sets the max velocity for the drivetrains motion profile
  /// @param rpm The max velocity in \b RPM to pass to the motion profile
  virtual void setMaxVelocity(const double &rpm) = 0;

  /// @brief Calculates the target velocity to send to the motors for smooth and
  /// precise movements using an S-curve profile.
  /// @param distance The remaining distance to the target in \b inches.
  /// @param dt The time elapsed since the last function call in \b seconds.
  /// @return The updated velocity in \b RPM.
  virtual double updateProfile(const double &distance, const double &dt) = 0;

  /// @brief Turns the robot towards a specific direction
  /// @param x The x component of the point we wish to face
  /// @param y The y component of the point we wish to face
  /// @note Uses coordinate system from GPS in \b meters
  virtual void turnTo(const double &x, const double &y) = 0;

  /// @brief Goes to the target point
  /// @param x The x component of the place where we want to go using the gps
  /// coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @param y The y component of the place where we want to go using the gps
  /// coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @note Uses coordinate system from GPS in \b meters
  virtual void goTo(const double &x, const double &y) = 0;

  /// @brief Goes to the target point
  /// @param pose The target pose
  /// @note Uses coordinate system from GPS in \b meters
  virtual void goToPose(const Pose &pose) = 0;

  /// @brief Scales a joystick input to drivetrain motor intensity according to a percentage
  /// @param input The joystick input to be scaled
  /// @param percentage The percentage of the drivetrain's `MAX_RPM` to scale to
  /// @return The `input` scaled to the `MAX_RPM` of the drivetrain as per `percentage`
  static double applySpeed(const double& input, const double& percentage){
    return input * MAX_RPM * percentage;
  }
};

}  // namespace aon