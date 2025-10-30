#include "../include/aon/h-drive/h-drive.hpp"

namespace aon {
// ============================================================================
//    ___      _
//   |   \ _ _(_)_ _____ _ _ ___
//   | |) | '_| \ V / -_) '_(_-<
//   |___/|_| |_|\_/\___|_| /__/
//
// ============================================================================

// void HDrive::opcontrol() {
//   //////////// DRIVE ////////////
//   const double vertical = aon::operator_control::AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0, SENSITIVITY);
//   const double horizontal = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0, SENSITIVITY);
//   const double turn = AnalogInputScaling(mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0, SENSITIVITY) * .8;
  
//   leftMotors.moveVelocity(MAX_RPM * std::clamp(vertical + turn, -1.0, 1.0));
//   rightMotors.moveVelocity(MAX_RPM * std::clamp(vertical - turn, -1.0, 1.0));
//   middleMotor.moveVelocity(MAX_RPM * std::clamp(vertical - turn, -1.0, 1.0) * 0.5); // make sure not kill the motor
// }

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

void HDrive::move2D(double x, double y, double t = aon::odometry::GetDegrees()) {
  aon::holonomic_motionH::MoveTrapezoidH(x, y, t);
}

void HDrive::moveHorizontalPID(PID pid, double dist, const double &MAX_REVS) {
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

void HDrive::moveHorizontalProfiled(double dist) {
  if (dist == 0) { return; }
  const int sign = dist / abs(dist);  // Getting the direction of the movement
  dist = abs(dist);                   // Setting the magnitude to positive
  
  double dt = 0.02;                   // (s)
  double currVelocity = 0;
  double traveledDist = 0;
  Vector startPos = aon::odometry::GetPosition();
  
  double now = pros::micros() / 1E6;
  double lastTime = now;
  
  this->motionProfile.setVelocity(this->getRPM());
  
  while (traveledDist < dist) {
    traveledDist = (aon::odometry::GetPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    lastTime = now;
    
    currVelocity = motionProfile.update(remainingDist, dt);
    this->motors(sign * currVelocity);

    if (traveledDist >= dist) { break; }  // Overshoot prevention

    pros::delay(20);
  }

  this->stop();
}

} // aon namespace