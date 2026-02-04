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

// ASK WHERE THIS FUNCTIONS SHOULD GO
/// @brief Scales analog joystick input for easier control.
/// @details Fine joystick control can be difficult, specially for tasks like
///          rotating. After researching the forums I found that teams scale their
///          joystick inputs using an exponential function of sorts. This makes small
///          inputs produce a smaller output and bigger inputs increase speed, so fine
///          movements can be done without as much of a hassle.
/// @param x The controller's user input between -127 and 127
/// @param t Sensitivity (higher is a steeper curve and vice-versa)
/// @return double between -1 and 1
///
/// @see Demonstration of scaling function in Desmos. https://www.desmos.com/calculator/kq9hgbxbwp
/// @warning Make sure that the input `x` is between -127 and 127!!!
inline double AnalogInputScaling(const double& x, const double& t) {
  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(x) - 127.0) / 10.0);

  return (a + b * (1 - a)) * x / 127.0;
}

/// @brief Scales a joystick input to drivetrain motor intensity according to a percentage
/// @param input The joystick input to be scaled
/// @param percentage The percentage of the drivetrain's `MAX_RPM` to scale to
/// @return The `input` scaled to the `MAX_RPM` of the drivetrain as per `percentage`
inline double ApplySpeed(const double& input, const double& percentage){
  return input * MAX_RPM * percentage;
}

void HDrive::drive(double leftX, double leftY, double rightX, double nothing) {
  const double forward = ApplySpeed(leftY , 0.6);
  const double horizontal = ApplySpeed(leftX, 0.6);
  const double turn = ApplySpeed(rightX, 0.4);
  
  driveWhileTurning(forward, turn);
  middleMotor.moveVelocity(horizontal);
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

    currVelocity = this->motionProfile.update(remainingDist, dt);
    this->motorsMid(sign * currVelocity);

    if (traveledDist >= dist) { break; }  // Overshoot prevention

    pros::delay(20);
  }

  currVelocity=0;
  this->motorsMid(0);
}

/// @brief Determines the linear speed of the robot given drivetrain motors' RPM
/// @param rpm The RPM for which to calculate the velocity (default current RPM)
/// @returns The speed in \b in/s at which the robot would move at the given RPM
/// @note Test the accuracy precision of the `getActualVelocity()` method,
/// @note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
/// @note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`.
inline double linearSpeed(const double &rpm = MAX_RPM){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double rps = rpm / 60;
  double speed = MOTOR_TO_DRIVE_RATIO * circumference * rps;
  return speed;
}

/// @brief Determines the rotational speed of the robot given drivetrain motors' RPM
/// @param rpm The RPM for which to calculate the velocity (default current RPM)
/// @return The speed in \b deg/s at which the robot would move at the given RPM
inline double rotationalSpeed(const double &rpm){
  const double drive_width = 10.5;
  const double drive_length = 8.25;
  const double ROBOT_RADIUS = hypot(drive_width, drive_length) / 2;
  double wheelCircumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double rps = rpm / 60;
  double tangentialSpeed = MOTOR_TO_DRIVE_RATIO * wheelCircumference * rps;
  return (tangentialSpeed * 180 / M_PI) / ROBOT_RADIUS;
}

/*
THINGS TO DO:
- change to Pose class, the x, y and theta
- test with PID
- check constant
- put the motion profile and pid as part of the class

*/
void HDrive::HolonomicMotion(
  double X, double Y, double T,
  double max_speed, double max_accel, double max_deccel,
  PID drivePID, PID turnPID
) {
  this->setX(0);
  this->setY(0);
  this->setTheta(0);
  // ---------- Constants ----------
  const double dt = 0.02; 
  const double d = DRIVE_WIDTH / 2; // inches
  const double drive_width = 17.5;
  const double drive_length = 17.5;
  const double ROBOT_RADIUS = hypot(drive_width, drive_length) / 2;
  const double circumference = M_TWOPI * ROBOT_RADIUS;

  // Declaration of motion profile (Change this into the tank class or H)
  MotionProfile xProfile(max_speed, max_accel, max_deccel, max_accel); // limit all movement because mid motor is slower than the rest of the base
  MotionProfile yProfile(max_speed * 0.8, max_accel * 0.8, max_deccel * 0.8, max_accel * 0.8);
  MotionProfile turnProfile(max_speed, max_accel * 0.3, max_deccel * 0.8, max_accel * 0.3);

  // Test if accuracy improve with PID
  drivePID.Reset();
  turnPID.Reset();

  double remainingX = abs(X - this->getX());
  double remainingY = abs(Y - this->getY());
  double remainingAngle = abs(T - this->getTheta());

  std::cout << "Reamining X: " << remainingX << "\n"; 
  std::cout << "Reamining Y: " << remainingY << "\n"; 
  std::cout << "Reamining T: " << remainingAngle << "\n"; 

  // double remainingX = abs(X - aon::odometry::GetX());
  // double remainingY = abs(Y - aon::odometry::GetY());
  // double remainingAngle = abs(T - aon::odometry::GetDegrees());

  double lastTime = pros::micros() / 1E6;

  double delay = 20;

  while (remainingX > 0.01 || remainingY > 0.01 || remainingAngle > 3) {
    double now = pros::micros() / 1E6;
    double dt_loop = now - lastTime;
    lastTime = now;

    // ---- Actual position ----
    double x = this->getX();
    double y = this->getY();
    double t = this->getTheta();

    std::cout << "current X: " << x << "\n"; 
    std::cout << "current Y: " << y << "\n"; 
    std::cout << "current T: " << t << "\n"; 

    // double x = aon::odometry::GetX();
    // double y = aon::odometry::GetY();
    // double t = aon::odometry::GetDegrees();

    // --- Remaining Distance ---
    remainingX = X - x;
    remainingY = Y - y;
    remainingAngle = T - t;

    constexpr double EPS_XY = 1e-4;   // inches
    constexpr double EPS_T  = 1e-2;   // degrees

    remainingX = fabs(remainingX) < EPS_XY ? 0 : remainingX;
    remainingY = fabs(remainingY) < EPS_XY ? 0 : remainingY;
    remainingAngle = fabs(remainingAngle) < EPS_T ? 0 : remainingAngle;

    std::cout << "Reamining X: " << remainingX << "\n"; 
    std::cout << "Reamining Y: " << remainingY << "\n"; 
    std::cout << "Reamining T: " << remainingAngle << "\n"; 

    // ----- Correct sign for motion profile ----
    int signX = (remainingX == 0) ? 1 : remainingX / abs(remainingX);
    int signY = (remainingY == 0) ? 1 : remainingY / abs(remainingY);
    int signT = (remainingAngle == 0) ? 1 : remainingAngle / abs(remainingAngle);


    remainingX = abs(remainingX);
    remainingY = abs(remainingY);
    remainingAngle = abs(remainingAngle);

    // --- Motion profile update ---
    double vx = xProfile.update(remainingX, dt_loop) * signX;
    double vy = yProfile.update(remainingY, dt_loop) * signY;
    double vT = turnProfile.update(circumference * (remainingAngle / 360.0), dt_loop) * signT;

    std::cout << "velocity X: " << vx << "\n"; 
    std::cout << "velocity Y: " << vy << "\n"; 
    std::cout << "velocity T: " << vT << "\n";
    
    // TEST IF MORE ACCURACY WITH PID
    // double vx_profile = xProfile.update(remainingX, dt_loop) * signX;
    // double vy_profile = yProfile.update(remainingY, dt_loop) * signY;
    // double vT_profile = turnProfile.update(circumference * (remainingAngle / 360.0), dt_loop) * signT;
    
    // double vx_pid = drivePID.OutputDt(X, x, dt_loop);
    // double vy_pid = drivePID.OutputDt(Y, y, dt_loop);
    // double vt_pid = drivePID.OutputDt(T, t, dt_loop);
    
    // double vx = vx_profile + vx_pid;
    // double vy = vx_profile + vy_pid;
    // double vT = vx_profile + vt_pid;
    
    
    // ============ FIELD → ROBOT TRANSFORM ============
    //
    // Rotate field-relative velocity into robot frame
    //
    // [ vx_r ]   [  cosθ   sinθ ] [ vx_f ]
    // [ vy_r ] = [ -sinθ   cosθ ] [ vy_f ]
    //
    // const double vx_robot =  vx * std::cos(aon::odometry::GetDegrees()) + vy * std::sin(aon::odometry::GetDegrees());
    // const double vy_robot = -vx * std::sin(aon::odometry::GetDegrees()) + vy * std::cos(aon::odometry::GetDegrees());

    double thetaRad = this->getTheta() * M_PI / 180.0;

    const double vx_robot = vx * std::cos(thetaRad) + vy * std::sin(thetaRad);
    const double vy_robot = -vx * std::sin(thetaRad) + vy * std::cos(thetaRad);

    
    std::cout << "velocity robot X: " << vx_robot << "\n"; 
    std::cout << "velocity robot Y: " << vy_robot << "\n"; 
    std::cout << "velocity T: " << vT << "\n";

    // ----- Move motors ------
    this->motorsLeft (vy_robot + vT);
    this->motorsRight(vy_robot - vT);
    this->motorsMid  (vx_robot);

    pros::delay(delay);

    std::cout << "X: " << this->getX() << "\n"; 
    std::cout << "X: " << this->getY() << "\n"; 
    std::cout << "X: " << this->getTheta() << "\n"; 

    this->setX(this->getX() + linearSpeed(vx_robot + vT) * (delay / 1000)); //# in case of odom failure
    this->setY(this->getY() + linearSpeed(vy_robot - vT) * (delay / 1000)); //# in case of odom failure
    this->setTheta(this->getTheta() + rotationalSpeed(vT) * delay / 1000); //# in case of odom failure
  }

  std::cout << "Stop because remaining is: " << remainingX << ", " <<  remainingY << ", " << remainingAngle << "\n";

  this->stop();
}

void HDrive::goToPose(double x, double y, double theta) {
  // if (x == 0 && y == 0) this->turn(theta);
  // const double timeout = std::max({x / MININUM_VELOCITY_LINEAR, y / MININUM_VELOCITY_LINEAR, theta / MININUM_VELOCITY_ANGULAR});
  HDrive::HolonomicMotion(x, y, theta);
}

} // aon namespace