#include "../include/aon/x-drive/x-drive.hpp"

#include <algorithm> // std::max, std::min, std::clamp
#include <cmath>     // std::abs

namespace aon {

// =========================
// Local helpers (no headers)
// =========================

// Convert robot-centric (vx, vy) into X-drive diagonal normalized commands.
// Returns Vector(x = topRightDiag, y = topLeftDiag), each roughly in [-2, 2] before normalization.
static Vector xyToDiagonals(double vx, double vy) {
  // Keep inputs sane
  vx = std::clamp(vx, -1.0, 1.0);
  vy = std::clamp(vy, -1.0, 1.0);

  // 45° transform for X-drive diagonals:
  // diagTR responds to (vx + vy)
  // diagTL responds to (vy - vx)
  const double diagTR = vx + vy;
  const double diagTL = vy - vx;

  // Normalize so neither diagonal exceeds [-1, 1]
  const double maxMag = std::max(1.0, std::max(std::abs(diagTR), std::abs(diagTL)));
  return Vector().SetPosition(diagTR / maxMag, diagTL / maxMag);
}

// Scale a normalized command [-1,1] to RPM using MAX_RPM and a [0,1] percentage.
static double toRPM(double normalized, double percentage) {
  normalized = std::clamp(normalized, -1.0, 1.0);
  percentage = std::clamp(percentage, 0.0, 1.0);
  return normalized * MAX_RPM * percentage;
}

void XDrive::motors(const double &rpm) {
  this->frontLeftMotors.moveVelocity(rpm);
  this->frontRightMotors.moveVelocity(rpm);
  this->backLeftMotors.moveVelocity(rpm);
  this->backRightMotors.moveVelocity(rpm);
}

void XDrive::rotate(const double &rpm) {
  this->frontLeftMotors.moveVelocity(rpm);
  this->backLeftMotors.moveVelocity(rpm);
  this->frontRightMotors.moveVelocity(-rpm);
  this->backRightMotors.moveVelocity(-rpm);
}

void XDrive::driveWhileTurning(const double &forward, const double &turn){
  this->frontLeftMotors.moveVelocity(forward + turn);
  this->backLeftMotors.moveVelocity(forward + turn);
  this->frontRightMotors.moveVelocity(forward - turn);
  this->backRightMotors.moveVelocity(forward - turn);
}

void XDrive::drive(double leftX, double leftY, double rightX, double rightY) {
  (void)rightY; // unused
  // Driver control default speed scaling:
  // 1.0 = full MAX_RPM, 0.5 = half speed
  const double pct = 0.5;
  driveRobotCentric(leftX, leftY, rightX, pct);
}

void XDrive::driveRobotCentric(const double vx, const double vy, const double omega, const double percentage) {
  const double clampedVX    = std::clamp(vx, -1.0, 1.0);
  const double clampedVY    = std::clamp(vy, -1.0, 1.0);
  const double clampedOmega = std::clamp(omega, -1.0, 1.0);

  // Convert XY to diagonal commands (normalized)
  Vector diags = xyToDiagonals(clampedVX, clampedVY);

  // Convert to RPM
  const double topRightDiagRPM = toRPM(diags.GetX(), percentage);
  const double topLeftDiagRPM  = toRPM(diags.GetY(), percentage);
  const double turnRPM         = toRPM(clampedOmega, percentage);

  // Motor mixing matches your earlier pattern
  this->frontLeftMotors.moveVelocity(topRightDiagRPM + turnRPM);
  this->frontRightMotors.moveVelocity(topLeftDiagRPM - turnRPM);
  this->backLeftMotors.moveVelocity(topLeftDiagRPM + turnRPM);
  this->backRightMotors.moveVelocity(topRightDiagRPM - turnRPM);
}

void XDrive::stop() { this->motors(0); }

void XDrive::configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset){
  frontLeftMotors.setBrakeMode(brakeMode);
  frontLeftMotors.setGearing(gearset);
  frontLeftMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  frontLeftMotors.tarePosition();

  frontRightMotors.setBrakeMode(brakeMode);
  frontRightMotors.setGearing(gearset);
  frontRightMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  frontRightMotors.tarePosition();
  
  backLeftMotors.setBrakeMode(brakeMode);
  backLeftMotors.setGearing(gearset);
  backLeftMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  backLeftMotors.tarePosition();

  backRightMotors.setBrakeMode(brakeMode);
  backRightMotors.setGearing(gearset);
  backRightMotors.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  backRightMotors.tarePosition();
}

double XDrive::getRPM(){
  // TODO: test and validate
  double frontLeft = frontLeftMotors.getActualVelocity();
  double frontRight = frontRightMotors.getActualVelocity();
  double backLeft = backLeftMotors.getActualVelocity();
  double backRight = backRightMotors.getActualVelocity();
  return ((frontLeft + frontRight + backLeft + backRight) / 4)/ 2;
}

void XDrive::drivePID(PID pid, double dist, const double &MAX_REVS) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive
  pid.Reset();

  Vector initialPos = odometry::GetPosition();

  const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time // every time the variable is called it is recalculated automatically

  while((odometry::GetPosition() - initialPos).GetMagnitude() < dist){

    double currentDisplacement = (odometry::GetPosition() - initialPos).GetMagnitude();

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

void XDrive::turnPID(PID pid, double angle, const double &MAX_REVS){
  const int sign = angle / abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  odometry::gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = odometry::GetDegrees(); // Angle relative to the start
  
  double timeLimit = math::getTimetoTurnDeg(angle);

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < timeLimit){

    double traveledAngle = abs(odometry::GetDegrees() - startAngle);

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

void XDrive::driveProfiled(double dist){
  if(dist == 0) { return; }
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive

  double dt = 0.02; // (s)
  double currVelocity = 0;
  double traveledDist = 0;
  Vector startPos = odometry::GetPosition();

  double now = pros::micros() / 1E6;
  double lastTime = now;

  this->motionProfile.setVelocity(this->getRPM());

  while(traveledDist < dist){
    traveledDist = (odometry::GetPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt =  now - lastTime;
    lastTime = now;

    currVelocity = this->motionProfile.update(remainingDist, dt);

    this->motors(sign * currVelocity);

    if(traveledDist >= dist) { break; } // Overshoot prevention

    pros::delay(20);
  }

  this->stop();
}

void XDrive::turnProfiled(double angle){
  if(angle == 0) { return; }
  const int sign = angle / abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  
  const double circumference = DRIVE_LENGTH * M_PI; // Of the robot's rotation, used in the condition to calculate the length of arc remaining
  const double MAX_VELOCITY = MAX_RPM; // (RPM)
  const double MAX_JERK = MAX_ACCEL; // (RPM/s^2)
  double dt = 0.02; // (s)
  double currVelocity = 0;
  double currAccel = 0;
  double traveledAngle = 0;
  double startAngle = odometry::GetDegrees();

  double now;
  double lastTime = pros::micros() / 1E6;
  
  while(traveledAngle < angle){
    traveledAngle = abs(odometry::GetDegrees() - startAngle);
    double remainingAngle = angle - traveledAngle;
    now = pros::micros() / 1E6;
    dt =  now - lastTime;
    lastTime = now;

    // Debugging output to brain
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
    pros::lcd::print(2, "RPM: %.2f, Accel: %.2f", currVelocity, currAccel);
    pros::lcd::print(3, "Remaining: %.2f", remainingAngle);
    pros::lcd::print(4, "Calculated Velocity: %.2f", getSpeed(currVelocity));
    pros::lcd::print(5, "Max Velocity: %.2f", getSpeed(MAX_VELOCITY));

    // Acceleration
    // For the condition, consider half the deceleration for accuracy (there is an error of half an inch almost constant when not used, I have to investigate a bit further on that part but if works fine like this)
    if(circumference * (remainingAngle / 360.0) <= getSpeed(currVelocity) * getSpeed(currVelocity) / (2.0 * getSpeed(MAX_DECEL * 0.5))){
      currAccel = - MAX_DECEL;
    } else {
      currAccel = std::min(currAccel + (MAX_JERK * dt), MAX_ACCEL);
    }

    currVelocity += currAccel * dt;
    currVelocity = std::min(currVelocity,  MAX_VELOCITY);

    frontLeftMotors.moveVelocity(sign * currVelocity);
    backLeftMotors.moveVelocity(sign * currVelocity);
    frontRightMotors.moveVelocity(-sign * currVelocity);
    backRightMotors.moveVelocity(-sign * currVelocity);

    if(traveledAngle >= angle) { break; } // Overshoot prevention

    pros::delay(20);
  }
  this->stop();
}

void XDrive::move(const double &dist){
  driveProfiled(dist);
}

void XDrive::turn(const double &angle){
  turnProfiled(angle);
}

void XDrive::setMaxVelocity(const double &rpm){
  motionProfile.setMaxVelocity(rpm);
}

double XDrive::updateProfile(const double &distance, const double &dt){
  return motionProfile.update(distance, dt);
}

void XDrive::driveInArc(double radius, const double &midSpeed) {
  if(radius == 0) return;
  const bool clockwise = radius > 0.0;
  radius = std::abs(radius);

  // Calculate wheel speeds based on center speed and arc geometry
  const double outerRatio =  (radius + (DRIVE_WIDTH / 2)) / radius;
  const double innerRatio = (radius - (DRIVE_WIDTH / 2)) / radius;
  const double outerSpeed = midSpeed * outerRatio;
  const double innerSpeed = midSpeed * innerRatio;

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

  frontLeftMotors.moveVelocity(leftSpeed); 
  backLeftMotors.moveVelocity(leftSpeed); 
  frontRightMotors.moveVelocity(rightSpeed);
  backRightMotors.moveVelocity(rightSpeed);
}

void XDrive::driveAngleOfArc(const double &radius, const double &angle) {
  if(angle == 0) { return; }
  if(radius == 0) {
    turn(angle);
    return;
  }
  const short sign = angle / std::abs(angle);
  const double distance = std::abs((2 * radius * M_PI) * (angle / 360));
  double midSpeed;
  double traveledDist = 0, remainingDist = distance;
  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;
  const double rightEncStartPos = odometry::encoderRight.get_position(); //! Temporary
  const double leftEncStartPos = odometry::encoderLeft.get_position(); //! Temporary
  // const double startDist = odometry::getTraveledDistance();
  while(traveledDist < distance){
    // traveledDist = odometry::getTraveledDistance() - startDist;
    const double rightEncDist = (std::abs(odometry::encoderRight.get_position() - rightEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    const double leftEncDist = (std::abs(odometry::encoderLeft.get_position() - leftEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    traveledDist = (rightEncDist + leftEncDist) / 2; //! Temporary
    remainingDist = distance - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    midSpeed = motionProfile.update(remainingDist, dt);
    lastTime = now;

    this->driveInArc(radius, sign * midSpeed);

    pros::delay(20);
  }

  this->stop();
}

void XDrive::driveInArcTo(const double &x, const double &y){
  // Get the current pose
  Vector position = odometry::GetPosition();
  position.SetPosition(math::inchesToMeters(position.GetX()), math::inchesToMeters(position.GetY()));
  double heading = odometry::GetDegrees(); //? should this come in the same format as the GPS heading?
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

/**
 * \brief Determines the angle needed to be turned in order to face a specific point in the field
 *
 * \param target The point we wish to face
 * \param current Where the robot is now
 *
 * \returns The angle the robot needs to turn in order to face the target location
 *
 * \note The result must be passed into functions such as `turn()` and `drivetrain.turnPID()` as negative because of the GPS convention
 */
double calculateTurn(Vector target, Vector current) {
  // Get and change the heading to the common cartesian plane
  double heading = 90 - odometry::gps.get_heading();

  // Limiting the heading to the 0-360 range
  if (heading < 0) heading += 360;
  else if (heading > 360) heading -= 360;
 
  // This number is in respect to the common cartesian plane if odometry position is used
  double toTarget = (target - current).GetDegrees();
 
  // Limiting the the target to the 0-360 range
  if (toTarget < 0) toTarget += 360;
  else if (toTarget >= 360) toTarget -= 360;

  double angle = toTarget - heading; // Calculate the angle to turn
 
  // Limiting the heading to the -180-180 range
  if (angle > 180) angle -= 360;
  else if (angle < -180) angle += 360;

  return angle;
}

void XDrive::turnTo(const double &x, const double &y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = odometry::gpsPosition();

  // Do the movement
  turn(-calculateTurn(target, current));
}

void XDrive::goTo(const double &x, const double &y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = odometry::gpsPosition();

  // Do the movement
  turn(-calculateTurn(target, current));
  move(math::findDistance(target, current));
}

}  // namespace aon
