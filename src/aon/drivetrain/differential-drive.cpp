#include "../../../include/aon/drivetrain/differential-drive.hpp"

namespace aon {

void DifferentialDrive::tank(const double &left, const double &right){
  this->leftMotors.moveVelocity(left);
  this->rightMotors.moveVelocity(right);
}

void DifferentialDrive::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode){
  leftMotors.setBrakeMode(brakeMode);
  rightMotors.setBrakeMode(brakeMode);
}

void DifferentialDrive::setGearset(okapi::AbstractMotor::gearset gearset){
  leftMotors.setGearing(gearset);
  rightMotors.setGearing(gearset);
}

void DifferentialDrive::setEncoderUnits(okapi::AbstractMotor::encoderUnits units){
  leftMotors.setEncoderUnits(units);
  leftMotors.tarePosition();
  rightMotors.setEncoderUnits(units);
  rightMotors.tarePosition();
}

void DifferentialDrive::setSlewRate(double slew){
  leftMotors.SetAcceleration(slew);
  rightMotors.SetAcceleration(slew);
}

double DifferentialDrive::getRPM(){
  double left = leftMotors.getActualVelocity();
  double right = rightMotors.getActualVelocity();
  return (left + right) / 2;
}

void DifferentialDrive::goToPose(const Pose& pose) {
  PurePursuit controller = PurePursuit(*this->yProfile, *this->thetaProfile, 5, 2.5, 2.5);

  std::pair<double, double> output = {-1, -1};

  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;

  // Generous timeout
  const uint32_t timeoutMs = (this->odometry->getPose().distanceTo(pose)) * 1E3;
  Timer timer;
  timer.start(timeoutMs);
  while (odometry->getPose().distanceTo(pose) > 2.0 && !timer.isCompleted()){
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

  this->turnToHeading(pose.theta);

  this->stop();
}

}  // namespace aon
