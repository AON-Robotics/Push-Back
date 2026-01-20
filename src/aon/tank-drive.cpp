#include "../include/aon/tank-drive/tank-drive.hpp"

namespace aon {

void TankDrive::motors(const double &rpm) {
  this->leftMotors.moveVelocity(rpm);
  this->rightMotors.moveVelocity(rpm);
}

void TankDrive::motorsLeft(const double &rpm) {
  this->leftMotors.moveVelocity(rpm);
}

void TankDrive::motorsRight(const double &rpm) {
  this->rightMotors.moveVelocity(rpm);
}

void TankDrive::rotate(const double &rpm) {
  this->leftMotors.moveVelocity(rpm);
  this->rightMotors.moveVelocity(-rpm);
}

void TankDrive::driveWhileTurning(const double &forward, const double &turn){
  this->leftMotors.moveVelocity(forward + turn);
  this->rightMotors.moveVelocity(forward - turn);
}

void TankDrive::drive(double leftX, double leftY, double rightX, double rightY) {
    // TODO: implement
}

void TankDrive::stop() { this->motors(0); }

void TankDrive::configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset){
  this->setBrakeMode(brakeMode);
  this->setGearset(gearset);
  this->setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);

  if(brakeMode == okapi::AbstractMotor::brakeMode::hold){
    this->setSlewRate(0);
  } else {
    this->setSlewRate(MAX_ACCEL);
  }
}

void TankDrive::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode){
  leftMotors.setBrakeMode(brakeMode);
  rightMotors.setBrakeMode(brakeMode);
}

void TankDrive::setGearset(okapi::AbstractMotor::gearset gearset){
  leftMotors.setGearing(gearset);
  rightMotors.setGearing(gearset);
}

void TankDrive::setEncoderUnits(okapi::AbstractMotor::encoderUnits units){
  leftMotors.setEncoderUnits(units);
  leftMotors.tarePosition();
  rightMotors.setEncoderUnits(units);
  rightMotors.tarePosition();
}

void TankDrive::setSlewRate(double slew){
  leftMotors.SetAcceleration(slew);
  rightMotors.SetAcceleration(slew);
}

double TankDrive::getRPM(){
  double left = leftMotors.getActualVelocity();
  double right = rightMotors.getActualVelocity();
  return (left + right) / 2;
}

void TankDrive::drivePID(PID pid, double dist, const double &MAX_REVS) {
  const int sign = dist / abs(dist);  // Getting the direction of the movement
  dist = abs(dist);                   // Setting the magnitude to positive
  pid.Reset();
  
  Vector initialPos = odometry::GetPosition();

  const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time  // every time the variable is called it is recalculated automatically

  while ((aon::odometry::GetPosition() - initialPos).GetMagnitude() < dist) {
    double currentDisplacement =
        (aon::odometry::GetPosition() - initialPos).GetMagnitude();
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

void TankDrive::turnPID(PID pid, double angle, const double &MAX_REVS) {
  const int sign = angle / abs(angle);  // Getting the direction of the movement
  angle = abs(angle);                   // Setting the magnitude to positive
  pid.Reset();
  odometry::gyroscope.tare();  // .tare() or .reset(true) depending on the time issue
  const double startAngle = odometry::GetDegrees();  // Angle relative to the start
  
  double timeLimit = math::getTimetoTurnDeg(angle);
  
  if (sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if (sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while (time < 3 * timeLimit) {

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

void TankDrive::driveProfiled(double dist) {
  if (dist == 0) { return; }
  const int sign = dist / abs(dist);  // Direction of the movement
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

    pros::lcd::print(0, "Trav %.2f", traveledDist);
    
    currVelocity = motionProfile.update(remainingDist, dt);
    this->motors(sign * currVelocity);

    if (remainingDist <= 0) { break; }  // Overshoot prevention

    pros::delay(20);
  }

  this->stop();
  std::cout << "Robot move to front\n";
  pros::delay(5000);
}

void TankDrive::turnProfiled(double angle) {
  MotionProfile turningProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
  if (angle == 0) { return; }
  const int sign = angle / abs(angle);  // Getting the direction of the movement
  angle = abs(angle);                   // Setting the magnitude to positive
  
  const double circumference = DRIVE_LENGTH * M_PI;  // Of the robot's rotation, used in the condition to calculate the length of arc remaining
  double dt = 0.02;                     // (s)
  double currVelocity = 0;
  double currAngle;
  double traveledAngle = 0;
  double startAngle = aon::odometry::gyroscope.get_rotation(); // TODO: add a function for this in the future odom class
  // double startAngle = aon::odometry::GetDegrees();  //! this means we need an equivalent for the odometer but for gyro

  double now;
  double lastTime = pros::micros() / 1E6;

  while (traveledAngle < angle) {
    currAngle = aon::odometry::gyroscope.get_rotation();
    traveledAngle = abs(currAngle - startAngle);
    // traveledAngle = abs(aon::odometry::GetDegrees() - startAngle);
    double remainingAngle = angle - traveledAngle;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    lastTime = now;
    
    // Debugging output to brain
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
    pros::lcd::print(2, "RPM: %.2f", currVelocity);
    pros::lcd::print(3, "Remaining: %.2f", remainingAngle);
    pros::lcd::print(4, "Calculated Velocity: %.2f", getSpeed(currVelocity));

    currVelocity = turningProfile.update(circumference * (remainingAngle / 360.0), dt);
    this->rotate(sign * currVelocity);

    if (traveledAngle >= angle) { break; }  // Overshoot prevention

    pros::delay(20);
  }
  
  this->stop();
  std::cout << "Turn do\n";
  pros::delay(5000);
}


void TankDrive::move(const double &dist) {
  driveProfiled(dist);
}

void TankDrive::turn(const double &angle) {
  turnProfiled(angle);
}

void TankDrive::setMaxVelocity(const double &rpm){
  motionProfile.setMaxVelocity(rpm);
}

double TankDrive::updateProfile(const double &distance, const double &dt){
  return motionProfile.update(distance, dt);
}

void TankDrive::driveInArc(double radius, const double &midSpeed) {
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

  leftMotors.moveVelocity(leftSpeed); 
  rightMotors.moveVelocity(rightSpeed);
}

void TankDrive::driveAngleOfArc(const double &radius, const double &angle) {
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

void TankDrive::driveInArcTo(const double &x, const double &y){
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

inline double calculateTurn(Vector target, Vector current) {
  // Get and change the heading to the common cartesian plane
  double heading = 90 - odometry::gps.get_heading();
  // Limiting the heading to the 0-360 range
  if (heading < 0)
    heading += 360;
  else if (heading > 360)
    heading -= 360;
  // This number is in respect to the common cartesian plane if odometry
  // position is used
  double toTarget = (target - current).GetDegrees();
  // Limiting the the target to the 0-360 range
  if (toTarget < 0)
    toTarget += 360;
  else if (toTarget >= 360)
    toTarget -= 360;
  double angle = toTarget - heading;  // Calculate the angle to turn
  // Limiting the heading to the -180-180 range
  if (angle > 180)
    angle -= 360;
  else if (angle < -180)
    angle += 360;
  return angle;
}

void TankDrive::turnTo(const double &x, const double &y) {
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = odometry::gpsPosition();
  // Do the movement
  turn(-calculateTurn(target, current));
}

void TankDrive::goTo(const double &x, const double &y) {
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = odometry::gpsPosition();
  // Do the movement
  turn(-calculateTurn(target, current));
  move(math::findDistance(target, current));
}

}  // namespace aon
