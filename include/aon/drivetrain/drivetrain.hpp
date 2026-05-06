#pragma once

#include "../../okapi/api.hpp"
#include "../odometry/odometry.hpp"
#include "../controls/smart_motor.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../math/timer.hpp"
#include "../controls/pure-pursuit.hpp"
#include <cfloat>


namespace aon {


class Drivetrain {
  public:

  /// @brief This struct holds the factors by which inputs will be scaled when driving during `turbo` and `!turbo` states
  struct SpeedFactors {
    double forwardNoTurbo = 0.5;
    double sidewaysNoTurbo = 0.5;
    double turnNoTurbo = 0.5;

    double forwardTurbo = 1;
    double sidewaysTurbo = 1;
    double turnTurbo = 1;

    SpeedFactors(
      double forwardNoTurbo = 0.5,
      double sidewaysNoTurbo = 0.5,
      double turnNoTurbo = 0.5,
      double forwardTurbo = 1,
      double sidewaysTurbo = 1,
      double turnTurbo = 1
    ) : forwardNoTurbo(forwardNoTurbo),
        sidewaysNoTurbo(sidewaysNoTurbo),
        turnNoTurbo(turnNoTurbo),
        forwardTurbo(forwardTurbo),
        sidewaysTurbo(sidewaysTurbo),
        turnTurbo(turnTurbo) {}
  };

  protected:

  std::unique_ptr<Odometry> odometry;
  Pose pose;
  bool turbo = false;

  /// @brief This applies only while using curvature drive to allow for turning without forward motion. Any forward motion below this will cause curvature drive to behave like arcade.
  static constexpr double QUICK_TURN_THRESHOLD = 0.05 * MAX_RPM;

  SpeedFactors speedFactors;

  std::unique_ptr<MotionProfile> xProfile;
  std::unique_ptr<MotionProfile> yProfile;
  std::unique_ptr<MotionProfile> thetaProfile;
  
  public:

  Drivetrain(Pose pose, std::unique_ptr<Odometry> odom, SpeedFactors speedFactors, 
             std::unique_ptr<MotionProfile> xProfile, std::unique_ptr<MotionProfile> yProfile, std::unique_ptr<MotionProfile> thetaProfile): 
             pose(pose), odometry(std::move(odom)), speedFactors(speedFactors),
             xProfile(std::move(xProfile)), yProfile(std::move(yProfile)), thetaProfile(std::move(thetaProfile)) {}

  enum DriveMode {
    TANK,
    ARCADE,
    SPLIT_ARCADE,
    CURVATURE,
    SPLIT_CURVATURE,
    HOLONOMIC,
  };

  // TODO: move all implementations to a dedicated cpp file

  /// @brief Starts the underlying odometry thread
  void initialize() { this->odometry->initialize(); }
  
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
  void motors(const double &rpm = MAX_RPM, const int& delay = 0) {
    this->tank(rpm, rpm);
    if(delay == 0) return;
    pros::delay(delay);
    this->stop();
  }

  /// @brief Moves all motors the same `rpm` to move sideways
  /// @param rpm The speed in which to move all motors in \b rpm
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  virtual void sideways(const double &rpm = MAX_RPM, const int& delay = 0) {}

  /// @brief Moves all motors the same `rpm` to rotate clockwise
  /// @param rpm The speed in which to move all motors in \b rpm
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  void rotate(const double &rpm = MAX_RPM, const int& delay = 0) {
    this->tank(rpm, -rpm);
    if(delay == 0) return;
    pros::delay(delay);
    this->stop();
  }

  /// @brief Drives the robot using tank control, mapping left and right inputs directly to each side of the drivetrain
  /// @param left The \b RPM to send to the left-side motors (positive is forward)
  /// @param right The \b RPM to send to the right-side motors (positive is forward)
  virtual void tank(const double &left, const double &right) = 0;
  
  /// @brief Drives the robot using arcade control, combining a forward and a turn input into left/right motor outputs
  /// @param forward The \b RPM to send to all motors for linear movement (positive is forward)
  /// @param turn The \b RPM to add/subtract from each side for rotational movement (positive is clockwise)
  void arcade(const double &forward, const double &turn) {
    double left = forward + turn;
    double right = forward - turn;

    // Normalize if either side exceeds MAX_RPM
    double maxVal = std::max(std::abs(left), std::abs(right));
    if (maxVal > MAX_RPM) {
      left  = (left  / maxVal) * MAX_RPM;
      right = (right / maxVal) * MAX_RPM;
    }

    this->tank(left, right);
  };

  /// @brief Drives the robot using curvature (cheesy drive) control, scaling the turn rate by the forward speed for smoother high-speed arcing
  /// @param forward The \b RPM to send to all motors for linear movement (positive is forward)
  /// @param turn The curvature input used to scale the rotational output relative to forward speed (positive is clockwise)
  void curvature(const double &forward, const double &turn) {
    bool quickTurn = std::abs(forward) < QUICK_TURN_THRESHOLD;

    double left, right;
    if (quickTurn) {
      // Fall back to arcade-style turning in place
      left  = forward + turn;
      right = forward - turn;
    } else {
      left  = forward + std::abs(forward) * turn;
      right = forward - std::abs(forward) * turn;
    }

    // Normalize if either side exceeds MAX_RPM
    double maxVal = std::max(std::abs(left), std::abs(right));
    if (maxVal > MAX_RPM) {
      left  = (left  / maxVal) * MAX_RPM;
      right = (right / maxVal) * MAX_RPM;
    }

    this->tank(left, right);
  };

  /// @brief Drives a holonomic (e.g. mecanum or X-drive) robot with independent forward, sideways, and rotational control
  /// @param forward The \b RPM to send to all motors for linear forward/backward movement (positive is forward)
  /// @param sideways The \b RPM to send to all motors for lateral strafe movement (positive is rightward)
  /// @param turn The \b RPM to send to all motors for rotational movement (positive is clockwise)
  virtual void holonomic(const double &forward, const double &sideways, const double &turn) {
    this->arcade(forward, turn);
    this->sideways(sideways);
  };

  /// @brief Drives the robot in the direction of the left joystick while turning it with the right joystick
  /// @param leftX The value of the left joystick on the x-axis in the range [-1, 1]
  /// @param leftY The value of the left joystick on the y-axis in the range [-1, 1]
  /// @param rightX The value of the right joystick on the x-axis in the range [-1, 1]
  /// @param rightY The value of the right joystick on the y-axis in the range [-1, 1]
  void drive(double leftX, double leftY, double rightX, double rightY, DriveMode mode = HOLONOMIC) {
    double left, right, forward, sideways, turn;

    // Determine movement scaling factors depending on turbo status
    double forwardFactor = this->isTurbo() ? speedFactors.forwardTurbo : speedFactors.forwardNoTurbo;
    double sidewaysFactor = this->isTurbo() ? speedFactors.sidewaysTurbo : speedFactors.sidewaysNoTurbo;
    double turnFactor = this->isTurbo() ? speedFactors.turnTurbo : speedFactors.turnNoTurbo;

    switch (mode) {
      case TANK:
        left = applySpeed(leftY, forwardFactor);
        right = applySpeed(rightY, forwardFactor);

        this->tank(left, right);
        break;

      case ARCADE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(leftX, turnFactor);

        this->arcade(forward, turn);
        break;

      case SPLIT_ARCADE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(rightX, turnFactor);

        this->arcade(forward, turn);
        break;

      case CURVATURE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(leftX, turnFactor);

        this->curvature(forward, turn);
        break;

      case SPLIT_CURVATURE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(rightX, turnFactor);

        this->curvature(forward, turn);
        break;

      case HOLONOMIC:
        forward = applySpeed(leftY, forwardFactor);
        sideways = applySpeed(leftX, sidewaysFactor);
        turn = applySpeed(rightX, turnFactor);

        this->holonomic(forward, sideways, turn);
        break;

      default:
        left = applySpeed(leftY, forwardFactor);
        right = applySpeed(rightY, forwardFactor);

        this->tank(left, right);
        break;
    }
  }

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

  /// @brief Moves the robot back and forth a set amount of times
  /// @param count The amount of jiggles to do
  /// @param rpm The revolutions per minute at which to do the jiggles
  /// @param delay The time each jiggle lasts
  void jiggle(int count = 3, double rpm = 125, int delay = 225){
    while(count--) {
      this->motors(rpm, delay);
      this->motors(-rpm, delay);
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
  void drivePID(PID pid = PID(0.02, 0, 0), double dist = TILE_WIDTH, const double &MAX_REVS = 100.0) {
    const int sign = dist / abs(dist);  // Getting the direction of the movement
    dist = abs(dist);                   // Setting the magnitude to positive
    pid.Reset();
    
    Vector initialPos = odometry->getPosition();

    const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
    const double start_time = pros::micros() / 1E6;
    #define time (pros::micros() / 1E6) - start_time  // every time the variable is called it is recalculated automatically

    while ((odometry->getPosition() - initialPos).GetMagnitude() < dist) {
      double currentDisplacement = (odometry->getPosition() - initialPos).GetMagnitude();
      double output = pid.Output(dist, currentDisplacement);

      pros::lcd::print(0, "Time Limit %.2f", timeLimit);
      pros::lcd::print(1, "Time: %.2f", time);
      pros::lcd::print(2, "Odometry Displacement %.2f", currentDisplacement);

      this->motors(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
      pros::delay(10);
    }

    // Stop the motors
    this->stop();

    #undef time
  }

  /// @brief Turns the robot by a given angle (default clockwise)
  /// @param pid The PID to be used for the turn
  /// @param angle The angle to make the robot turn in \b degrees
  /// @param MAX_REVS The maximum RPM to send to the movement
  void turnPID(PID pid = PID(0.002, 0, 0), double angle = 90, const double &MAX_REVS = 50.0) {
    const int sign = angle / abs(angle);  // Getting the direction of the movement
    angle = abs(angle);                   // Setting the magnitude to positive
    pid.Reset();
    odometry->gyroscope.tare();  // .tare() or .reset(true) depending on the time issue
    const double startAngle = odometry->getDegrees();  // Angle relative to the start
    
    double timeLimit = math::getTimetoTurnDeg(angle);
    
    if (sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
    if (sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

    const double startTime = pros::micros() / 1E6;
    #define time (pros::micros() / 1E6) - startTime

    while (time < 3 * timeLimit) {

      double traveledAngle = abs(odometry->getDegrees() - startAngle);
      
      double output = pid.Output(angle, traveledAngle);

      pros::lcd::print(0, "Time Limit %.2f", timeLimit);
      pros::lcd::print(1, "Time: %.2f", time);
      pros::lcd::print(2, "Gyroscope Displacement %.2f", traveledAngle);
      
      // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
      this->rotate(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));

      pros::delay(10);
    }

    this->stop();

    #undef time
  }

  /// @brief S-graph motion profile for linear movement
  /// @param dist The distance to be moved in \b inches, positive values will move forward and negative values backwards
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void driveProfiled(double dist = TILE_WIDTH, bool settle = true) {
    if (dist == 0) { return; }
    const int sign = dist / abs(dist);  // Direction of the movement
    dist = abs(dist);                   // Setting the magnitude to positive
    
    // Timeout determined experimentally
    const uint32_t timeoutMs = (dist / 3.0) * 1E3;
    Timer timer;
    timer.start(timeoutMs);
    
    double dt = 0.02;                   // (s)
    double currVelocity = 0;
    double traveledDist = 0;
    Vector startPos = odometry->getPosition();
    
    double now = pros::micros() / 1E6;
    double lastTime = now;
    
    this->yProfile->setVelocity(this->getRPM());
    this->yProfile->setFinalVelocity(settle ? 0 : 100);

    while (traveledDist < dist && !timer.isCompleted()) {
      traveledDist = (odometry->getPosition() - startPos).GetMagnitude();
      double remainingDist = dist - traveledDist;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      lastTime = now;

      // Debugging output
      pros::lcd::print(1, "Traveled %.2f / %.2f", traveledDist, dist);
      pros::c::controller_print(pros::controller_id_e_t::E_CONTROLLER_MASTER, 0, 0, "Trav %.2f / %.2f", traveledDist, dist);

      currVelocity = this->yProfile->update(remainingDist, dt);
      this->motors(sign * currVelocity);

      if (remainingDist <= 0) { break; }  // Overshoot prevention

      pros::delay(20);
    }

    if (settle) { this->stop(); }
  }

  /// @brief S-graph motion profile for linear movement
  /// @param dist The distance to be moved in \b inches, positive values will move right and negative values left
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void strafeProfiled(double dist = TILE_WIDTH, bool settle = true) {
    if(dist == 0) { return; }
    const int sign = dist / abs(dist); // Getting the direction of the movement
    dist = abs(dist); // Setting the magnitude to positive

    // Timeout determined experimentally
    const uint32_t timeoutMs = (dist / 3.0) * 1E3;
    Timer timer;
    timer.start(timeoutMs);
    
    double dt = 0.02; // (s)
    double currVelocity = 0;
    double traveledDist = 0;
    Vector startPos = odometry->getPosition();

    double now = pros::micros() / 1E6;
    double lastTime = now;

    this->xProfile->setVelocity(this->getRPM());
    this->xProfile->setFinalVelocity(settle ? 0 : 100);

    while(traveledDist < dist && !timer.isCompleted()){
      traveledDist = (odometry->getPosition() - startPos).GetMagnitude();
      // traveledDist += getSpeed(this->getRPM()) * dt; //# in case of odom failure

      double remainingDist = dist - traveledDist;
      now = pros::micros() / 1E6;
      dt =  now - lastTime;
      lastTime = now;

      currVelocity = this->xProfile->update(remainingDist, dt);
      this->sideways(sign * currVelocity);

      if(remainingDist <= 0) { break; } // Overshoot prevention

      pros::delay(20);
    }

    if(settle) this->stop();
  }

  /// @brief S-graph motion profile for rotations
  /// @param angle The angle in \b degrees we wish to rotate the robot, positive is clockwise and negative is counter-clockwise
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void turnProfiled(double angle = 90, bool settle = true) {
    if (angle == 0) { return; }
    const int sign = angle / abs(angle);  // Getting the direction of the movement
    angle = abs(angle);                   // Setting the magnitude to positive

    // Timeout determined experimentally
    const uint32_t timeoutMs = (std::sqrt(angle / 2)) * 1E3;
    Timer timer;
    timer.start(timeoutMs);

    const double circumference = DRIVE_WIDTH * M_PI;  // Of the robot's rotation, used in the condition to calculate the length of arc remaining
    double dt = 0.02;                     // (s)
    double currVelocity = 0;
    double currAngle;
    double traveledAngle = 0;

    double startAngle = odometry->gyroscope.get_rotation(); // TODO: add a function for this in the future odom class
    // double startAngle = aon::odometry::GetDegrees();  //! this means we need an equivalent for the odometer but for gyro

    double now;
    double lastTime = pros::micros() / 1E6;

    this->thetaProfile->setFinalVelocity(settle ? 0 : 50);

    while (traveledAngle < angle && !timer.isCompleted()) {
      currAngle = odometry->gyroscope.get_rotation();
      traveledAngle = abs(currAngle - startAngle);
      // traveledAngle = abs(aon::odometry::GetDegrees() - startAngle);
      double remainingAngle = angle - traveledAngle;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      lastTime = now;
      
      // Debugging output
      pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
      // pros::c::controller_print(pros::controller_id_e_t::E_CONTROLLER_MASTER, 0, 0, "Trav %.2f / %.2f", traveledAngle, angle);

      currVelocity = this->thetaProfile->update(circumference * (remainingAngle / 360.0), dt);
      this->rotate(sign * currVelocity);

      if (traveledAngle >= angle) { break; }  // Overshoot prevention

      pros::delay(20);
    }
    if (settle) this->stop();
  }

  /// @brief Moves the robot a given distance
  /// @param dist The distance to move in \b inches
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details A positive `dist` makes the robot go forward while a negative `dist` makes the robot go backwards
  void move(const double &dist = TILE_WIDTH, bool settle = true) { // TODO: change this settle variable to a more direct `double finalVelocity = 0` or similar (ideally same concept just a better name)
    driveProfiled(dist, settle);
  }

  /// @brief Moves the robot a given distance
  /// @param dist The distance to move in \b inches
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details A positive `dist` makes the robot go right while a negative `dist` makes the robot go left
  void strafe(const double &dist = TILE_WIDTH, bool settle = true) { // TODO: add optional enum to pick which controller to use (for now just PID and MotionProfile available)
    strafeProfiled(dist, settle);
  }

  /// @brief Turn the robot a given angle (default is clockwise)
  /// @param angle The angle to turn in \b degrees
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details Clockwise is positive and counter-clockwise is negative
  void turn(const double &angle = 90, bool settle = true) {
    turnProfiled(angle, settle);
  }

  /// @brief Sets the max velocity for the drivetrains motion profile
  /// @param rpm The max velocity in \b RPM to pass to the motion profile
  void setMaxVelocity(const double &rpm) {
    this->yProfile->setMaxVelocity(rpm);
  }

  /// @brief Calculates the target velocity to send to the motors for smooth and
  /// precise movements using an S-curve profile.
  /// @param distance The remaining distance to the target in \b inches.
  /// @param dt The time elapsed since the last function call in \b seconds.
  /// @return The updated velocity in \b RPM.
  double updateProfile(const double &distance, const double &dt) {
    return this->yProfile->update(distance, dt);
  }

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param speed The speed with which to drive in \b RPM (positive speed
  /// will go forward and negative speed will go backwards)
  /// @note A positive `radius` will cause a clockwise rotation, while a
  /// negative `radius` will cause a counter-clockwise rotation
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  void driveInArc(double radius, const double &speed = 200) {
    if(radius == 0) return;
    const bool clockwise = radius > 0.0;
    radius = std::abs(radius);

    // Calculate wheel speeds based on center speed and arc geometry
    const double outerRatio =  (radius + (DRIVE_WIDTH / 2)) / radius;
    const double innerRatio = (radius - (DRIVE_WIDTH / 2)) / radius;
    const double outerSpeed = speed * outerRatio;
    const double innerSpeed = speed * innerRatio;

    double leftSpeed, rightSpeed;
    
    // Clockwise, more speed on the left
    if(clockwise) {
      leftSpeed = outerSpeed;
      rightSpeed = innerSpeed;
    }
    // Counter-clockwise, more speed on the right
    else {
      rightSpeed = outerSpeed;
      leftSpeed = innerSpeed;
    }

    this->tank(leftSpeed, rightSpeed);
  }

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
  void driveAngleOfArc(const double &radius = DRIVE_WIDTH, const double &angle = 90, bool settle = true) {
    if(angle == 0) { return; }
    if(radius == 0) {
      turn(angle, settle);
      return;
    }
    const short sign = angle / std::abs(angle);
    const double distance = std::abs((2 * radius * M_PI) * (angle / 360));
    double midSpeed;
    double traveledDist = 0, remainingDist = distance;
    double dt = 0.02;
    double now = pros::micros() / 1E6;
    double lastTime = now;
    const double rightEncStartPos = odometry->encoderRight.get_position(); //! Temporary
    const double leftEncStartPos = odometry->encoderLeft.get_position(); //! Temporary
    this->yProfile->setVelocity(this->getRPM());
    this->yProfile->setFinalVelocity(settle ? 0 : 100);
    // const double startDist = odometry::getTraveledDistance();

    // Timeout determined experimentally
    const uint32_t timeoutMs = (distance / 3.0) * 1E3;
    Timer timer;
    timer.start(timeoutMs);

    while(traveledDist < distance && !timer.isCompleted()){
      // traveledDist = odometry::getTraveledDistance() - startDist;
      const double rightEncDist = (std::abs(odometry->encoderRight.get_position() - rightEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
      const double leftEncDist = (std::abs(odometry->encoderLeft.get_position() - leftEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
      traveledDist = (rightEncDist + leftEncDist) / 2; //! Temporary
      remainingDist = distance - traveledDist;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      midSpeed = this->yProfile->update(remainingDist, dt);
      lastTime = now;

      this->driveInArc(radius, sign * midSpeed);

      pros::delay(20);
    }

    if(settle) this->stop();
  }

  /// @brief Makes the robot drive in an arc motion to a specified point in the
  /// field
  /// @param x The x coordinate of the point we want to go to in \b meters
  /// @param y The y coordinate of the point we want to go to in \b meters
  /// @note Odometry must be working for global positioning on the field
  /// @see https://www.desmos.com/calculator/5abb373276
  void driveInArcTo(const double &x, const double &y) {
    // Get the current pose
    Vector position = odometry->getPosition();
    position.SetPosition(math::inchesToMeters(position.GetX()), math::inchesToMeters(position.GetY()));
    double heading = odometry->getDegrees(); //? should this come in the same format as the GPS heading?
    Vector target = Vector().SetPosition(x, y);

    // Convert the heading to traditional math coordinates
    heading = (90 - heading); //? only do the `(90 - heading)` part if the heading comes in gps coordinates
    if (heading < 0) { heading += 360; }
    heading *=  M_PI / 180;

    // (heading - π/2) % π cannot be 0 because tan(heading) would not be defined
    const bool isTanHeadingDefined = std::fmod(heading - M_PI_2, M_PI) != 0;

    // Calculate slopes of tangent to circular path and secant that cuts through current point and desired point
    double m_t = isTanHeadingDefined ? std::tan(heading) : DBL_MAX;
    double m_s = (position.GetX() != x) ? (position.GetY() - y) / (position.GetX() - x) : DBL_MAX;

    // Avoid 0 division later by switching to a very small value if a 0 slope arises
    m_t = m_t == 0 ? DBL_MIN : m_t;
    m_s = m_s == 0 ? DBL_MIN : m_s;

    // Get midpoint of the secant
    Vector midpoint = Vector().SetPosition((position.GetX() + x) / 2, (position.GetY() + y) / 2);

    // Calculate the position of the center of the circular path
    double centerX = (midpoint.GetY() - position.GetY() - (position.GetX() / m_t) + (midpoint.GetX() / m_s)) / ((-1 / m_t) + (1 / m_s));
    double centerY = ((-1 / m_t) * (centerX - position.GetX())) + position.GetY();
    Vector center = Vector().SetPosition(centerX, centerY);

    // Get the radius using the pythagorean theorem
    double radius = std::hypot(position.GetX() - center.GetX(), position.GetY() - center.GetY());

    // Determine the angle with some geometry and trigonometry
    double angle = math::getAngleOfArc(position, target, center);

    // Use a projection to determine which way we are turning
    const double projectionStep = 0.001;
    const Vector projection = Vector().SetPosition(position.GetX() + (projectionStep * std::cos(heading)),
                                                  position.GetY() + (projectionStep * std::sin(heading)));
    const double projectionAngle = math::getAngleInCircle(projection, center);
    
    const double positionAngle = math::getAngleInCircle(position, center);
    const double targetAngle = math::getAngleInCircle(target, center);
    
    // If going clockwise, the center is to the right (positive radius) and to the left in a counter-clockwise movement (negative radius)
    const bool clockwise = (targetAngle < projectionAngle && projectionAngle < positionAngle) || (projectionAngle < positionAngle && positionAngle < targetAngle) || (positionAngle < targetAngle && targetAngle < projectionAngle);
    if (!clockwise) { radius *= -1; }
    
    // Check if we have to go the long way around
    const bool longWay = (math::getAngleOfArc(projection, target, center) > angle) || (positionAngle < targetAngle && targetAngle < projectionAngle);
    if (longWay) { angle = 360 - angle; }

    this->driveAngleOfArc(math::metersToInches(radius), angle);
  }

  /// @brief Turns the robot towards a specific direction
  /// @param x The x component of the point we wish to face
  /// @param y The y component of the point we wish to face
  /// @note Uses coordinate system from GPS in \b meters
  void turnToPoint(const double &x, const double &y) {
    Vector target = Vector().SetPosition(x, y);
    // Determine current position
    Pose current = odometry->getPose();
    // Do the movement
    turn(-math::calculateTurn(target, current));
  }

  /// @brief Turns the robot to an absolute heading
  /// @param heading The target heading in \b degrees (same convention as odometry)
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void turnToHeading(const double &heading, bool settle = true) {
    double delta = heading - odometry->getDegrees();
    // Normalize to [-180, 180] for the shortest path
    if (delta > 180) delta -= 360;
    else if (delta < -180) delta += 360;
    turn(delta, settle);
  }
  

  /// @brief Goes to the target point
  /// @param x The x component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @param y The y component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @note Uses coordinate system from GPS in \b meters
  void goToPoint(const double &x, const double &y) {
    Vector target = Vector().SetPosition(x, y);
    // Determine current position
    Vector current = odometry->gpsPosition();
    // Do the movement
    turn(-math::calculateTurn(target, odometry->getPose()));
    move(math::findDistance(target, current));
  }

  /// @brief Goes to the target point
  /// @param pose The target pose
  /// @note Uses coordinate system from GPS in \b meters
  virtual void goToPose(const Pose &pose) = 0;

  /// @brief Follows a path using a pure pursuit controller
  /// @param path The path to follow
  /// @note The `path`s intermediate headings are ignored, only the final one is actually aligned
  virtual void follow(const std::vector<Pose>& path) {
    PurePursuit controller = PurePursuit(*this->yProfile, *this->thetaProfile, 5, 2.5, 2.5);

    std::pair<double, double> output = {-1, -1};

    double dt = 0.02;
    double now = pros::micros() / 1E6;
    double lastTime = now;

    // Generous timeout
    const uint32_t timeoutMs = (math::length(path)) * 1E3;
    Timer timer;
    timer.start(timeoutMs);

    while (odometry->getPose().distanceTo(path.back()) > 2.0 && !timer.isCompleted()) {
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      output = controller.follow(path, this->odometry->getPose(), dt);
      lastTime = now;
      this->tank(output.first, output.second);

      pros::lcd::print(0, "Current: Pose(%.2f, %.2f, %.2f)", odometry->getX(), odometry->getY(), odometry->getDegrees());
      pros::lcd::print(1, "Target: Pose(%.2f, %.2f, %.2f)", path.back().x, path.back().y, path.back().theta);
      pros::lcd::print(2, "Distance: %.2f", odometry->getPose().distanceTo(path.back()));
      pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "Distance: %.2f", odometry->getPose().distanceTo(path.back()));

      if (output.first == 0 && output.second == 0) { break; }

      pros::delay(10);
    }

    this->turnToHeading(path.back().theta);

    this->stop();
  }

  /// @brief Scales a joystick input to drivetrain motor intensity according to a percentage
  /// @param input The joystick input to be scaled
  /// @param percentage The percentage of the drivetrain's `MAX_RPM` to scale to
  /// @return The `input` scaled to the `MAX_RPM` of the drivetrain as per `percentage`
  static double applySpeed(const double& input, const double& percentage){
    return input * MAX_RPM * percentage;
  }
};

}  // namespace aon
