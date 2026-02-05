#include "../include/aon/h-drive/h-drive.hpp"

namespace aon {
// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ 
//   | |) | '_| \ V / -_) '_|
//   |___/|_| |_|\_/\___|_|
//
// ============================================================================

void HDrive::drive(double leftX, double leftY, double rightX, double nothing) {
  const double forward = applySpeed(leftY , 0.6);
  const double horizontal = applySpeed(leftX, 0.6);
  const double turn = applySpeed(rightX, 0.4);
  
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

void HDrive::motorMid(const double &rpm) {
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
    this->motorMid(sign * currVelocity);

    if (traveledDist >= dist) { break; }  // Overshoot prevention

    pros::delay(20);
  }

  currVelocity=0;
  this->motorMid(0);
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
  PID drivePID, PID turnPID
) {
  this->setX(0);
  this->setY(0);
  this->setTheta(0);
  // ---------- Constants ----------
  const double delay = 20;
  const double d = DRIVE_WIDTH / 2; // inches
  // Maybe move this to constants
  const double drive_width = 17.5;
  const double drive_length = 17.5;
  const double ROBOT_RADIUS = hypot(drive_width, drive_length) / 2;
  const double circumference = M_TWOPI * ROBOT_RADIUS;

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

  while (remainingX > 0.05 || remainingY > 0.05 || remainingAngle > 0.05) {

    // ---- Actual position ---- Simplify this after testing PID
    double x = this->getX();
    double y = this->getY();
    double t = this->getTheta();
    double tRad = t * (M_PI / 180);

    std::cout << "current X: " << x << "\n"; 
    std::cout << "current Y: " << y << "\n"; 
    std::cout << "current T: " << t << "\n"; 

    // --- Remaining Distance ---
    remainingX = X - x;
    remainingY = Y - y;
    remainingAngle = T - t;

    constexpr double EPS_XY = 1e-4;   // inches
    constexpr double EPS_T  = 1e-2;   // degrees

    remainingX = fabs(remainingX) < EPS_XY ? 0 : remainingX;
    remainingY = fabs(remainingY) < EPS_XY ? 0 : remainingY;
    remainingAngle = fabs(remainingAngle) < EPS_T ? 0 : remainingAngle;

    
    // ----- Correct sign for motion profile ----
    int signX = (remainingX == 0) ? 1 : remainingX / abs(remainingX);
    int signY = (remainingY == 0) ? 1 : remainingY / abs(remainingY);
    int signT = (remainingAngle == 0) ? 1 : remainingAngle / abs(remainingAngle);
    
    remainingX = abs(remainingX);
    remainingY = abs(remainingY);
    remainingAngle = abs(remainingAngle);

    std::cout << "Reamining X: " << remainingX << ",Y: " << remainingY << ", T: " << remainingAngle << "\n"; 

    // ============ FIELD → ROBOT TRANSFORM ============
    //
    // Rotate field-relative velocity into robot frame
    //
    // [ vx_r ]   [  cosθ   sinθ ] [ vx_f ]
    // [ vy_r ] = [ -sinθ   cosθ ] [ vy_f ]

    double dx_r =  cos(tRad) * remainingX + sin(tRad) * remainingY;
    double dy_r = -sin(tRad) * remainingX + cos(tRad) * remainingY;

    // --- Motion profile update ---
    double vx = this->xProfile.update(remainingX) * signX;
    double vy = this->yProfile.update(remainingY) * signY;
    double vT = this->thetaProfile.update(circumference * (remainingAngle / 360.0)) * signT;

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

    // ----- Move motors ------
    this->motorsLeft (vy + vT);
    this->motorsRight(vy - vT);
    this->motorMid   (vx);

    pros::delay(delay);

    this->setX(this->getX() + math::linearSpeed(vx) * (delay / 1000)); //# in case of odom failure
    this->setY(this->getY() + math::linearSpeed(vy) * (delay / 1000)); //# in case of odom failure
    this->setTheta(this->getTheta() + math::rotationalSpeed(vT) * delay / 1000); //# in case of odom failure
  }

  std::cout << "Stop because remaining is: " << remainingX << ", " <<  remainingY << ", " << remainingAngle << "\n";

  this->stop();
}

// void HDrive::goToH(const double &x, const double &y, const double &theta) {
//   HDrive::HolonomicMotion(x, y, theta);
// }

void HDrive::goToPose(Pose &target) {
  HDrive::HolonomicMotion(target.x, target.y, target.theta);
}

} // aon namespace