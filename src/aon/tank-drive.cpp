#include "../include/aon/tank-drive/tank-drive.hpp"


namespace aon {


void TankDrive::initialize(){
  this->odometry->initialize();
}

void TankDrive::motors(const double &rpm, const int& delay) {
  this->leftMotors.moveVelocity(rpm);
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
  const double vertical = applySpeed(leftY, this->isTurbo() ? 1 : 0.6);
  const double turn = applySpeed(rightX, this->isTurbo() ? 1 : 0.4);

  this->driveWhileTurning(vertical, turn);
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
  
  Vector initialPos = odometry->getPosition();

  const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time  // every time the variable is called it is recalculated automatically

  while ((odometry->getPosition() - initialPos).GetMagnitude() < dist) {
    double currentDisplacement =
        (odometry->getPosition() - initialPos).GetMagnitude();
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

void TankDrive::driveProfiled(double dist, bool settle) {
  if (dist == 0) { return; }
  const int sign = dist / abs(dist);  // Direction of the movement
  dist = abs(dist);                   // Setting the magnitude to positive
  
  // Timeout determined experimentally
  const uint32_t estimatedTime = (dist / 3.0) * 1E3;
  const uint32_t timeout = pros::millis() + estimatedTime;
  
  double dt = 0.02;                   // (s)
  double currVelocity = 0;
  double traveledDist = 0;
  Vector startPos = odometry->getPosition();
  
  double now = pros::micros() / 1E6;
  double lastTime = now;
  
  this->motionProfile.setVelocity(this->getRPM());
  this->motionProfile.setFinalVelocity(settle ? 0 : 100);

  while (traveledDist < dist && timeout > pros::millis()) {
    traveledDist = (odometry->getPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    lastTime = now;

    // Debugging output
    pros::lcd::print(1, "Traveled %.2f / %.2f", traveledDist, dist);
    pros::c::controller_print(pros::controller_id_e_t::E_CONTROLLER_MASTER, 0, 0, "Trav %.2f / %.2f", traveledDist, dist);

    currVelocity = this->motionProfile.update(remainingDist, dt);
    this->motors(sign * currVelocity);

    if (remainingDist <= 0) { break; }  // Overshoot prevention

    pros::delay(20);
  }

  if (settle) { this->stop(); }
}

void TankDrive::turnProfiled(double angle, bool settle) {
  if (angle == 0) { return; }
  const int sign = angle / abs(angle);  // Getting the direction of the movement
  angle = abs(angle);                   // Setting the magnitude to positive

  // Timeout determined experimentally
  const uint32_t estimatedTime = (std::sqrt(angle / 2)) * 1E3;
  const uint32_t timeout = pros::millis() + estimatedTime;

  const double circumference = DRIVE_WIDTH * M_PI;  // Of the robot's rotation, used in the condition to calculate the length of arc remaining
  double dt = 0.02;                     // (s)
  double currVelocity = 0;
  double currAngle;
  double traveledAngle = 0;

  double startAngle = odometry->gyroscope.get_rotation(); // TODO: add a function for this in the future odom class
  // double startAngle = aon::odometry::GetDegrees();  //! this means we need an equivalent for the odometer but for gyro

  double now;
  double lastTime = pros::micros() / 1E6;

  this->turningProfile.setFinalVelocity(settle ? 0 : 50);

  while (traveledAngle < angle && timeout > pros::millis()) {
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

    currVelocity = this->turningProfile.update(circumference * (remainingAngle / 360.0), dt);
    this->rotate(sign * currVelocity);

    if (traveledAngle >= angle) { break; }  // Overshoot prevention

    pros::delay(20);
  }
  if (settle) this->stop();
}


void TankDrive::move(const double &dist, bool settle) {
  driveProfiled(dist, settle);
}

void TankDrive::turn(const double &angle, bool settle) {
  turnProfiled(angle, settle);
}

void TankDrive::setMaxVelocity(const double &rpm){
  this->motionProfile.setMaxVelocity(rpm);
}

double TankDrive::updateProfile(const double &distance, const double &dt){
  return this->motionProfile.update(distance, dt);
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

void TankDrive::driveAngleOfArc(const double &radius, const double &angle, bool settle) {
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
  this->motionProfile.setVelocity(this->getRPM());
  this->motionProfile.setFinalVelocity(settle ? 0 : 100);
  // const double startDist = odometry::getTraveledDistance();

  // Timeout determined experimentally
  const uint32_t estimatedTime = (distance / 3.0) * 1E3;
  const uint32_t timeout = pros::millis() + estimatedTime;

  while(traveledDist < distance && timeout > pros::millis()){
    // traveledDist = odometry::getTraveledDistance() - startDist;
    const double rightEncDist = (std::abs(odometry->encoderRight.get_position() - rightEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    const double leftEncDist = (std::abs(odometry->encoderLeft.get_position() - leftEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    traveledDist = (rightEncDist + leftEncDist) / 2; //! Temporary
    remainingDist = distance - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    midSpeed = this->motionProfile.update(remainingDist, dt);
    lastTime = now;

    this->driveInArc(radius, sign * midSpeed);

    pros::delay(20);
  }

  if(settle) this->stop();
}

void TankDrive::driveInArcTo(const double &x, const double &y){
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

void TankDrive::turnTo(const double &x, const double &y) {
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Pose current = odometry->getPose();
  // Do the movement
  turn(-math::calculateTurn(target, current));
}

// TODO: refactor so it uses `Pose()`
void TankDrive::goTo(const double &x, const double &y) {
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = odometry->gpsPosition();
  // Do the movement
  turn(-math::calculateTurn(target, odometry->getPose()));
  move(math::findDistance(target, current));
}

// TODO: replace with "Pure Pursuit" implementation
void TankDrive::goToPose(const Pose& pose){
  
}

}  // namespace aon
