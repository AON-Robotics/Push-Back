#include "../include/aon/h-drive/h-drive.hpp"

namespace aon {
// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ 
//   | |) | '_| \ V / -_) '_|
//   |___/|_| |_|\_/\___|_|
//
// ============================================================================

void HDrive::configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset){
    leftMotors.setBrakeMode(brakeMode);
    leftMotors.setGearing(gearset);
    leftMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    leftMotors.tarePosition();
    
    rightMotors.setBrakeMode(brakeMode);
    rightMotors.setGearing(gearset);
    rightMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    rightMotors.tarePosition();

    this->middleMotor.setBrakeMode(brakeMode);
    this->middleMotor.setGearing(okapi::AbstractMotor::gearset::green);
    this->middleMotor.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    this->middleMotor.tarePosition();

}

void HDrive::drive(double horizontal, double vertical, double turn, double nothing) {
  //////////// DRIVE ////////////
  // const double horizontal = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, SENSITIVITY);
  // const double vertical = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, SENSITIVITY);
  // const double turn = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY) * .8;
  
  leftMotors.moveVelocity(600 * std::clamp(vertical + turn, -1.0, 1.0));
  rightMotors.moveVelocity(600 * std::clamp(vertical - turn, -1.0, 1.0));
  middleMotor.moveVelocity(600 * std::clamp(horizontal, -1.0, 1.0));
}

void HDrive::driveV2(double horizontal, double vertical, double turn, double nothing, bool middleRight, bool middleLeft) {
  //////////// DRIVE ////////////
  // const double horizontal = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, SENSITIVITY);
  // const double vertical = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, SENSITIVITY);
  // const double turn = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY) * .8;
  
  leftMotors.moveVelocity(600 * std::clamp(vertical + turn, -1.0, 1.0));
  rightMotors.moveVelocity(600 * std::clamp(vertical - turn, -1.0, 1.0));
  if (middleRight){
    middleMotor.moveVelocity(600);
  }
  if (middleLeft) {
    middleMotor.moveVelocity(-600);
  }
  if (!middleRight && !middleLeft) {
    middleMotor.moveVelocity(0);
  }
  // else
  //   middleMotor.moveVelocity(0);

}

// ============================================================================
//   __  __  _____   _____ __  __ ___ _  _ _____
//  |  \/  |/ _ \ \ / / __|  \/  | __| \| |_   _|
//  | |\/| | (_) \ V /| _|| |\/| | _|| .` | | |  
//  |_|  |_|\___/ \_/ |___|_|  |_|___|_|\_| |_|  
//
// ============================================================================

void HDrive::stop(){
  this->motors(0);
  this->middleMotor.moveVelocity(0);
}

void HDrive::motorsMid(const double &rpm) {
  this->middleMotor.moveVelocity(rpm);
}

void HDrive::moveHorizontalPID(double dist, PID pid, const double &MAX_REVS) {
  const int sign = dist / abs(dist);  // Getting the direction of the movement
  dist = abs(dist);                   // Setting the magnitude to positive
  pid.Reset();
  
  Vector initialPos = aon::odometry::GetPosition();

  const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time  // every time the variable is called it is recalculated automatically

  while ((aon::odometry::GetPosition() - initialPos).GetMagnitude() < dist) {
    double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();
    double output = pid.Output(dist, currentDisplacement);
    pros::lcd::print(0, "Time Limit %.2f", timeLimit);
    pros::lcd::print(1, "Time: %.2f", time);
    pros::lcd::print(2, "Odometry Displacement %.2f", currentDisplacement);
    this->middleMotor.moveVelocity(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
    pros::delay(10);
  }

  // Stop the motors
  this->stop();

  #undef time
}

void HDrive::strafe(double dist) {
  if (dist == 0) { return; }
  const int sign = dist / abs(dist);  // Getting the direction of the movement
  dist = abs(dist);                   // Setting the magnitude to positive
  
  double dt = 0.02;                   // (s)
  double currVelocity = 0;
  double traveledDist = 0;
  double startPos = aon::odometry::encoderBack.get_position();
  // Vector startPos = aon::odometry::GetPosition();
  
  double now = pros::micros() / 1E6;
  double lastTime = now;
  
  this->motionProfile.setVelocity(this->middleMotor.getActualVelocity());
  
  while (traveledDist < dist) {
    traveledDist = (std::abs(aon::odometry::encoderBack.get_position() - startPos) / 100) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION;
    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    lastTime = now;
    
    pros::lcd::print(0, "Trav %.2f", traveledDist);

    currVelocity = motionProfile.update(remainingDist, dt);
    this->motorsMid(sign * currVelocity);

    if (traveledDist >= dist) { break; }  // Overshoot prevention

    pros::delay(20);
  }

  this->motorsMid(0);
  std::cout << "Strafe\n";
  pros::delay(5000);
}

// double timeoutX = computeTimeout(std::abs(dx), max_vx, max_ax);
  // double timeoutY = computeTimeout(std::abs(dy), max_vy, max_ay);
  // // double timeoutTheta = computeTimeout(std::abs(dT), max_vt, max_accel); /// FIND MAX FOR THETA
  // double timeoutTheta = 1.0; /// FIND MAX FOR THETA
  
  // double max_vx = max_speed * dir_x;
  // double max_vy = max_speed * dir_y;
  // double max_ax = max_accel * fabs(dir_x);
  // double max_ay = max_accel * fabs(dir_y);
  
  // pick the largest
  // double timeout = std::max({timeoutX, timeoutY, timeoutTheta});
  // double timeout = 1.0;
  // std::cout << "Timeout: " << timeout << "\n";
  
  

  // pros::lcd::print(2, "Before Loop");
  // Run loop while time hasn't run out
  // while (t < timeout) {
  //   t = pros::millis() / 1000.0 - start_time;
  //   double dt = t - last_t;
  //   last_t = t;

  //   // Current remaining distances
  //   Vector current_position = aon::odometry::GetPosition();
  //   double current_dx = X - current_position.GetX(); // remaining X
  //   double current_dy = Y - current_position.GetY(); // remaining Y
  //   double current_dT = T - (-aon::odometry::GetRadians()); // remaining rotation
  //   // Normalize angle error to [-π, π]
  //   // This prevents long rotations the wrong way
  //   while (current_dT >  M_PI) current_dT -= 2.0 * M_PI;
  //   while (current_dT < -M_PI) current_dT += 2.0 * M_PI;

  //   std::cout << "Remaining: " << current_dx << ", " << current_dy << ", " << current_dT <<"\n";

  //   // Convert degrees → linear inches for one side of the drivetrain
  //   // current_dT = current_dT * (M_PI * (DRIVE_WIDTH / 2.0) / 180.0);

  //   // Update S-curve profiles
  //   double vx_ff = xMotionProfile.update(current_dx, dt);
  //   double vy_ff = yMotionProfile.update(current_dy, dt);
  //   // MotionProfile must be configured with angular limits:
  //   //   max_omega (rad/s)
  //   //   max_alpha (rad/s^2)
  //   double vT_ff = TMotionProfile.update(current_dT, dt); // CHECK MOTION PROFILE FOR ROTATION

  //   // PID corrections
  //   double vx_corr = vxPID.OutputDt(0, current_dx, dt);
  //   double vy_corr = vyPID.OutputDt(0, current_dy, dt);
  //   double vT_corr = thetaPID.OutputDt(0, current_dT, dt);

  //   std::cout << "Velocity in RPM before moving: " << vx_ff << ", " << vy_ff << ", " << vT_ff << "\n";

  //   // Send to motors
  //   MoveHolonomicMotionH(vx_ff + vx_corr,
  //                        vy_ff + vy_corr,
  //                        vT_ff + vT_corr,
  //                        true);

  //   function(t * 1000);
  //   pros::delay(10);
  // }

void HDrive::MoveHolonomicMotionH(double vx, double vy, double vT, bool use_odom) {
  // ================= UNIT CONTRACT =================
  // vx, vy : inches / second (field-relative)
  // vT     : radians / second (CCW positive)
  // ================================================

  const double d = DRIVE_WIDTH / 2.0;               // inches
  const double wheel_diam = DRIVE_WHEEL_DIAMETER;   // inches

  // Robot heading (field → robot transform)
  // Odometry uses CW positive, so we negate it
  const double theta = use_odom ? -aon::odometry::GetRadians() : 0.0;

  // ============ FIELD → ROBOT TRANSFORM ============
  //
  // Rotate field-relative velocity into robot frame
  //
  // [ vx_r ]   [  cosθ   sinθ ] [ vx_f ]
  // [ vy_r ] = [ -sinθ   cosθ ] [ vy_f ]
  //
  const double vx_robot =  vx * std::cos(theta) + vy * std::sin(theta);
  const double vy_robot =  vy * std::cos(theta) - vx * std::sin(theta);

  // =============== ROTATION CONTRIBUTION ===========
  //
  // Convert angular velocity into linear velocity at wheels
  //
  // v = ω * r
  //
  const double v_rot = vT * d;   // inches / second

  // Wheel linear velocities
  const double v_left  = vy_robot + v_rot;
  const double v_right = vy_robot - v_rot;
  const double v_mid   = vx_robot;

  // ============ LINEAR VELOCITY → RPM ==============
  //
  // Wheel circumference = π * D
  // inches/sec → rev/sec → RPM
  //
  const double INPS_TO_RPM = 60.0 / (M_PI * wheel_diam);

  this->motorsLeft (v_left  * INPS_TO_RPM);
  this->motorsRight(v_right * INPS_TO_RPM);
  this->motorsMid  (v_mid   * INPS_TO_RPM);
}


void HDrive::MoveTrapezoidH(double X, double Y, double T,
                   double v0,
                   double vf,
                   double max_speed,
                   double max_accel,
                   double max_speed_angular,
                   double max_accel_angular,
                   PID drivePID, PID turnPID) {
  std::cout << "it enters move trapezoidH";

  const double d = DRIVE_WIDTH / 2.0; // Half the width of the robot
  const double target_theta = T * M_PI / 180;
  const double dt = 0.02;
  
  // Store initial conditions
  const double start_x = aon::odometry::GetX();
  const double start_y = aon::odometry::GetY();
  const double start_theta = -aon::odometry::GetRadians();   // Odometry uses CW as positive, but we want CCW to be positive
  const double start_time = pros::millis() / 1000.0;
  double last_t = start_time;
  
  // Determine how much base should move in each component
  const double dx = X - start_x;
  const double dy = Y - start_y;
  const double dT = target_theta - start_theta; // radians
  
  // Decompose max_speed and max_accel magnitudes into x and y components
  // using angle made by dx and dy as angle for the vector
  double h = std::hypot(dx, dy);
  double dir_x = 0.0;
  double dir_y = 0.0;

  // Dont divide by 0
  if (h > 1e-6) {
    dir_x = dx / h;
    dir_y = dy / h;
  }
  
  std::cout << "it enters move trapezoidH profile";
  
  // Create trapezoid profiles
  TrapezoidProfile xProfile(
      h,                     // total distance (in)
      max_speed,             // max velocity (in/s)
      max_accel,             // accel (in/s^2)
      max_accel              // decel (in/s^2)
  );

  TrapezoidProfile thetaProfile(
      dT,                    // radians
      max_speed_angular,     // rad/s
      max_accel_angular,     // rad/s^2
      max_accel_angular
  );

  // Reset PID before creating the objects
  drivePID.Reset();
  turnPID.Reset();
  std::cout << "it enters move trapezoidH PID";
  // Instantiate PID objects
  PID vxPID = drivePID;
  PID vyPID = drivePID;
  PID thetaPID = turnPID;
  
  while (true) {
    std::cout << "it enters move trapezoidH loop";
    double t = pros::millis() / 1000.0 - start_time;

    // Scalar speed along the path
    double v_path = xProfile.SpeedProfile(t);   // in/s

    // Decompose into X and Y
    double vx_ff = v_path * dir_x;
    double vy_ff = v_path * dir_y;

    // Angular velocity
    double vT_ff = thetaProfile.SpeedProfile(t); // rad/s

    // PID correction
    Vector pos = aon::odometry::GetPosition();
    double theta = -aon::odometry::GetRadians();

    auto wrapAngle = [](double a) {
      while (a > M_PI)  a -= 2 * M_PI;
      while (a < -M_PI) a += 2 * M_PI;
      return a;
    };

    // Position errors
    double ex = X - pos.GetX();
    double ey = Y - pos.GetY();
    double e_theta = wrapAngle(target_theta - theta);

    // PID correction
    double vx_corr = vxPID.OutputDt(X, ex, dt);
    double vy_corr = vyPID.OutputDt(Y, ey, dt);
    double vT_corr = thetaPID.OutputDt(target_theta, e_theta, dt);
    
    // Apply corrections
    double vx = vx_ff + vx_corr;
    double vy = vy_ff + vy_corr;
    double vT = vT_ff + vT_corr;

    // Save last velocity so the PID maintain realistic results
    static double last_vT = 0.0;

    double max_dvT = max_accel_angular * dt;
    vT = std::clamp(
      vT,
      last_vT - max_dvT,
      last_vT + max_dvT
    );

    vT = std::clamp(vT,
      -max_speed_angular,
      max_speed_angular
    );

    last_vT = vT;

    if (T == 0) vT = 0;

    std::cout << "vx: " << vx_ff << ", vy: " << vy_ff << "vt: " << vT_ff << "\n";
    std::cout << "vx: " << vx_corr << ", vy: " << vy_corr << "vt: " << vT_corr << "\n";
    
    // Termination condition
    const double POS_EPS = 0.25;        // inches
    const double ANG_EPS = 1.0 * M_PI / 180.0; // 1 degree

    bool pos_done = std::hypot(ex, ey) < POS_EPS;
    bool ang_done = std::abs(e_theta) < ANG_EPS;

    if (pos_done && ang_done) {
        std::cout << "DONE" << ex << ", " << ey << "\n";
        break;
    }

    if (std::abs(e_theta) < ANG_EPS) {
      vT = 0;
    }

    MoveHolonomicMotionH(vx, vy, vT, true);

    pros::delay(10);
  }
  
  pros::lcd::print(4, "After Loop");
  MoveHolonomicMotionH(0, 0, 0, false);
}

void HDrive::move2D(double x, double y, double theta) {
  std::cout << "It enters in move2d";
  if (x == 0 && y == 0) this->turn(theta);
  HDrive::MoveTrapezoidH(x, y, theta);
}

} // aon namespace