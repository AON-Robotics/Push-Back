#include "../../../include/aon/drivetrain/x-drive.hpp"

namespace aon {

void XDrive::sideways(const double &rpm, const int& delay) {
  this->frontLeftMotors.moveVelocity(rpm);
  this->frontRightMotors.moveVelocity(-rpm);
  this->backLeftMotors.moveVelocity(-rpm);
  this->backRightMotors.moveVelocity(rpm);
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void XDrive::tank(const double &left, const double &right){
  this->frontLeftMotors.moveVelocity(left);
  this->backLeftMotors.moveVelocity(left);
  this->frontRightMotors.moveVelocity(right);
  this->backRightMotors.moveVelocity(right);
}

void XDrive::holonomic(const double &forward, const double &sideways, const double &turn){
  Vector direction = Vector().SetPosition(sideways, forward);
  Vector command = translateToMotorCommand(direction);
  double topRightDiag = command.GetX();
  double topLeftDiag = command.GetY();

  double frontLeft = topRightDiag + turn;
  double frontRight = topLeftDiag - turn;
  double backLeft = topLeftDiag + turn;
  double backRight = topRightDiag - turn;

  // Normalize if anything exceeds MAX_RPM
  double maxVal = std::max({std::abs(frontLeft), std::abs(frontRight), std::abs(backLeft), std::abs(backRight)});
  if(maxVal > MAX_RPM) {
    frontLeft = (frontLeft / maxVal) * MAX_RPM;
    frontRight = (frontRight / maxVal) * MAX_RPM;
    backLeft = (backLeft / maxVal) * MAX_RPM;
    backRight = (backRight / maxVal) * MAX_RPM;
  }

  this->frontLeftMotors.moveVelocity(frontLeft);
  this->frontRightMotors.moveVelocity(frontRight);
  this->backLeftMotors.moveVelocity(backLeft);
  this->backRightMotors.moveVelocity(backRight);
}

Vector XDrive::translateToMotorCommand(Vector direction){
  Vector result;
  result.SetX(direction.GetX() * 0.70710678118 + direction.GetY() * 0.70710678118);
  result.SetY(direction.GetX() * -0.70710678118 + direction.GetY() * 0.70710678118);
  return result;
}

void XDrive::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode){
  frontLeftMotors.setBrakeMode(brakeMode);
  frontRightMotors.setBrakeMode(brakeMode);
  backLeftMotors.setBrakeMode(brakeMode);
  backRightMotors.setBrakeMode(brakeMode);
}

void XDrive::setGearset(okapi::AbstractMotor::gearset gearset){
  frontLeftMotors.setGearing(gearset);
  frontRightMotors.setGearing(gearset);
  backLeftMotors.setGearing(gearset);
  backRightMotors.setGearing(gearset);
}

void XDrive::setEncoderUnits(okapi::AbstractMotor::encoderUnits units){
  frontLeftMotors.setEncoderUnits(units);
  frontLeftMotors.tarePosition();
  frontRightMotors.setEncoderUnits(units);
  frontRightMotors.tarePosition();
  backLeftMotors.setEncoderUnits(units);
  backLeftMotors.tarePosition();
  backRightMotors.setEncoderUnits(units);
  backRightMotors.tarePosition();
}

void XDrive::setSlewRate(double slew){
  frontLeftMotors.SetAcceleration(slew);
  frontRightMotors.SetAcceleration(slew);
  backLeftMotors.SetAcceleration(slew);
  backRightMotors.SetAcceleration(slew);
}

double XDrive::getRPM(){
  double frontLeft = abs(frontLeftMotors.getActualVelocity());
  double frontRight = abs(frontRightMotors.getActualVelocity());
  double backLeft = abs(backLeftMotors.getActualVelocity());
  double backRight = abs(backRightMotors.getActualVelocity());
  return (frontLeft + frontRight + backLeft + backRight) / 4;
}

// Using PID
// void XDrive::goToPose(const Pose& target){
//   const double delay = 20; // ms
//   const Pose initialPose = this->getPose();
//   Pose currPose = this->getPose();
//   PID xPID = PID(50, 10, 0, delay / 1000, 2, 50);
//   PID yPID = PID(50, 10, 0, delay / 1000, 2, 50);
//   PID thetaPID = PID(2.8, 1.25, 0, delay / 1000, 5, 50);

//   // while(!(withinError(this->getPose().x, target.x, 10) && withinError(this->getPose().y, target.y, 10) && withinError(this->getPose().theta, target.theta, 10))){
//   while(abs(this->getPose().x - target.x) > 0.5 || abs(this->getPose().y - target.y) > 0.5 || abs(this->getPose().theta - target.theta) > 2.5){
//     pros::lcd::print(1, "X: %.2f / %.2f", this->getPose().x, target.x);
//     pros::lcd::print(2, "Y: %.2f / %.2f", this->getPose().y, target.y);
//     pros::lcd::print(3, "Theta: %.2f / %.2f", this->getPose().theta, target.theta);
//     double x = xPID.Output(target.x, currPose.x);
//     double y = yPID.Output(target.y, currPose.y);
//     double theta = thetaPID.Output(target.theta, currPose.theta);

//     Vector direction = Vector().SetPosition(x, y);
//     direction.SetDegrees(direction.GetDegrees() + currPose.theta);// - initialPose.theta);
//     Vector command = translateToMotorCommand(direction);
//     double topRightDiag = command.GetX();
//     double topLeftDiag = command.GetY();
//     double turn = theta;

//     this->frontLeftMotors.moveVelocity(topRightDiag + turn);
//     this->frontRightMotors.moveVelocity(topLeftDiag - turn);
//     this->backLeftMotors.moveVelocity(topLeftDiag + turn);
//     this->backRightMotors.moveVelocity(topRightDiag - turn);
//     pros::delay(delay);
//     currPose.x += getSpeed(x) * delay / 1000; //# in case of odom failure
//     currPose.y += getSpeed(y) * delay / 1000; //# in case of odom failure
//     currPose.theta += rotationSpeed(theta) * delay / 1000; //# in case of odom failure
//     this->setPose(currPose);
//   }

//   pros::lcd::clear();
//   this->stop();
// }

// Using Motion Profile
void XDrive::goToPose(const Pose& target){
  const double delay = 20; // ms

  double remainingX = abs(target.x - this->getX());
  double remainingY = abs(target.y - this->getY());
  double remainingTheta = abs(target.theta - this->getTheta());

  const double drive_width = 10.5;
  const double drive_length = 8.25;
  const double ROBOT_RADIUS = hypot(drive_width, drive_length) / 2;
  const double circumference = M_TWOPI * ROBOT_RADIUS;

  // TODO: add timeouts for safety
  while(remainingX > 0.05 || remainingY > 0.05 || remainingTheta > 0.05){

    pros::lcd::print(0, "(x, y, theta): (%.2f, %.2f, %.2f)", this->getX(), this->getY(), this->getTheta());
    remainingX = target.x - this->getX();
    remainingY = target.y - this->getY();
    remainingTheta = target.theta - this->getTheta();

    double xSign = (remainingX > 0) - (remainingX < 0);
    double ySign = (remainingY > 0) - (remainingY < 0);
    double thetaSign = (remainingTheta > 0) - (remainingTheta < 0);

    remainingX = abs(remainingX);
    remainingY = abs(remainingY);
    remainingTheta = abs(remainingTheta);

    double x = this->xProfile->update(remainingX) * xSign;
    double y = this->yProfile->update(remainingY) * ySign;
    double theta = this->thetaProfile->update(circumference * (remainingTheta / 360.0)) * thetaSign;

    Vector direction = Vector().SetPosition(x, y);
    direction.SetDegrees(direction.GetDegrees() + this->getTheta());// - initialPose.theta);

    // TODO: test that this line correctly replaces the previous (commented out) behavior (it should)
    this->holonomic(direction.GetY(), direction.GetX(), theta);
    // Vector command = translateToMotorCommand(direction);
    
    // double topRightDiag = command.GetX();
    // double topLeftDiag = command.GetY();
    // double turn = theta;

    // this->frontLeftMotors.moveVelocity(topRightDiag + turn);
    // this->frontRightMotors.moveVelocity(topLeftDiag - turn);
    // this->backLeftMotors.moveVelocity(topLeftDiag + turn);
    // this->backRightMotors.moveVelocity(topRightDiag - turn);

    pros::delay(delay);

    // this->setX(this->getX() + math::linearSpeed(x) * delay / 1000); //# in case of odom failure
    // this->setY(this->getY() + math::linearSpeed(y) * delay / 1000); //# in case of odom failure
    // this->setTheta(this->getTheta() + math::rotationalSpeed(theta) * delay / 1000); //# in case of odom failure
  }

  pros::lcd::clear();
  this->stop();
}

void XDrive::follow(const std::vector<Pose>& path) {
  PurePursuit controller = PurePursuit(*this->yProfile, *this->thetaProfile, 5, 2.5, 2.5);

  std::pair<double, double> output = {-1, -1};

  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;

  // Generous timeout
  const uint32_t timeoutMs = (this->odometry->getPose().distanceTo(pose)) * 1E3;
  Timer timer;
  timer.start(timeoutMs);
  while (odometry->getPose().distanceTo(path.back()) > 2.0 && !timer.isCompleted()) {
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    output = controller.go(pose, this->odometry->getPose(), dt);
    lastTime = now;
    this->tank(output.first, output.second);

    pros::lcd::print(0, "Current: Pose(%.2f, %.2f, %.2f)", odometry->getX(), odometry->getY(), odometry->getDegrees());
    pros::lcd::print(1, "Target: Pose(%.2f, %.2f, %.2f)", pose.x, pose.y, pose.theta);
    pros::lcd::print(2, "Distance: %.2f", odometry->getPose().distanceTo(pose));
    pros::c::controller_print(pros::E_CONTROLLER_MASTER, 0, 0, "Distance: %.2f", odometry->getPose().distanceTo(pose));

    if (output.first == 0 && output.second == 0) { break; }

    pros::delay(10);
  }

  this->turnToHeading(path.back().theta);

  this->stop();
}

}  // namespace aon
