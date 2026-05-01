#include "../include/aon/h-drive/h-drive.hpp"
#include "../include/aon/math/timer.hpp"


namespace aon {

void HDrive::initialize() {
  this->odometry->initialize();
}

void HDrive::motors(const double &rpm, const int& delay) {
  this->leftMotors.moveVelocity(rpm);
  this->rightMotors.moveVelocity(rpm);
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void HDrive::sideways(const double &rpm) {
  this->midMotors.moveVelocity(rpm);
}

void HDrive::rotate(const double &rpm) {
  this->leftMotors.moveVelocity(rpm);
  this->rightMotors.moveVelocity(-rpm);
}

void HDrive::driveWhileTurning(const double &forward, const double &turn){
  this->leftMotors.moveVelocity(forward + turn);
  this->rightMotors.moveVelocity(forward - turn);
}

void HDrive::drive(double leftX, double leftY, double rightX, double rightY) {
  double forward = applySpeed(leftY, this->isTurbo() ? 1 : 0.5);
  double sideways = applySpeed(leftX, this->isTurbo() ? 1 : 0.75);
  double turn = applySpeed(rightX, this->isTurbo() ? 1 : 0.5);

  this->driveWhileTurning(forward, turn);
  this->sideways(sideways);
}

void HDrive::setBrakeMode(okapi::AbstractMotor::brakeMode brakeMode){
  leftMotors.setBrakeMode(brakeMode);
  rightMotors.setBrakeMode(brakeMode);
  midMotors.setBrakeMode(brakeMode);
}

void HDrive::setGearset(okapi::AbstractMotor::gearset gearset){
  leftMotors.setGearing(gearset);
  rightMotors.setGearing(gearset);
  midMotors.setGearing(gearset);
}

void HDrive::setEncoderUnits(okapi::AbstractMotor::encoderUnits units){
  leftMotors.setEncoderUnits(units);
  leftMotors.tarePosition();
  rightMotors.setEncoderUnits(units);
  rightMotors.tarePosition();
  midMotors.setEncoderUnits(units);
  midMotors.tarePosition();
}

void HDrive::setSlewRate(double slew){
  leftMotors.SetAcceleration(slew);
  rightMotors.SetAcceleration(slew);
  midMotors.SetAcceleration(slew);
}



double HDrive::getRPM(){
  double left = leftMotors.getActualVelocity();
  double right = rightMotors.getActualVelocity();
  return (left + right) / 2;
}

void HDrive::drivePID(PID pid, double dist, const double &MAX_REVS) {
  const int sign = dist / abs(dist); // Getting the direction of the movement
  dist = abs(dist); // Setting the magnitude to positive
  pid.Reset();

  Vector initialPos = odometry->getPosition();

  const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
  const double start_time = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - start_time // every time the variable is called it is recalculated automatically

  while((odometry->getPosition() - initialPos).GetMagnitude() < dist){

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

void HDrive::turnPID(PID pid, double angle, const double &MAX_REVS){
  const int sign = angle / abs(angle); // Getting the direction of the movement
  angle = abs(angle); // Setting the magnitude to positive
  pid.Reset();
  odometry->gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
  const double startAngle = odometry->getDegrees(); // Angle relative to the start
  
  double timeLimit = math::getTimetoTurnDeg(angle);

  if(sign == -1) { angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET; }
  if(sign == 1) { angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET; }

  const double startTime = pros::micros() / 1E6;
  #define time (pros::micros() / 1E6) - startTime

  while(time < timeLimit){

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

void HDrive::driveProfiled(double dist, bool settle){
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

  this->yProfile.setVelocity(this->getRPM());
  this->yProfile.setFinalVelocity(settle ? 0 : 100);

  while(traveledDist < dist && !timer.isCompleted()){
    traveledDist = (odometry->getPosition() - startPos).GetMagnitude();
    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt =  now - lastTime;
    lastTime = now;

    currVelocity = this->yProfile.update(remainingDist, dt);
    this->motors(sign * currVelocity);

    if(remainingDist <= 0) { break; } // Overshoot prevention

    pros::delay(20);
  }

  if(settle) this->stop();
}

void HDrive::strafeProfiled(double dist, bool settle){
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
  // Vector startPos = odometry->getPosition();
  const double backStartPos = odometry->encoderBack.get_position(); //! Temporary

  double now = pros::micros() / 1E6;
  double lastTime = now;

  this->xProfile.setVelocity(this->getRPM());
  this->xProfile.setFinalVelocity(settle ? 0 : 100);

  while(traveledDist < dist && !timer.isCompleted()){
    // traveledDist = (odometry->getPosition() - startPos).GetMagnitude();
    traveledDist = (std::abs(odometry->encoderBack.get_position() - backStartPos) / 100) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    // traveledDist += getSpeed(this->getRPM()) * dt; //# in case of odom failure

    double remainingDist = dist - traveledDist;
    now = pros::micros() / 1E6;
    dt =  now - lastTime;
    lastTime = now;

    currVelocity = this->xProfile.update(remainingDist, dt);
    this->sideways(sign * currVelocity);

    if(remainingDist <= 0) { break; } // Overshoot prevention

    pros::delay(20);
  }

  if (settle) this->stop();
}

void HDrive::turnProfiled(double angle, bool settle){
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
  double startAngle = odometry->getDegrees();

  double now;
  double lastTime = pros::micros() / 1E6;

  this->thetaProfile.setFinalVelocity(settle ? 0 : 100);
  
  while(traveledAngle < angle && !timer.isCompleted()){
    traveledAngle = abs(odometry->getDegrees() - startAngle);
    double remainingAngle = angle - traveledAngle;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    lastTime = now;
    
    // Debugging output
    pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
    // pros::c::controller_print(pros::controller_id_e_t::E_CONTROLLER_MASTER, 0, 0, "Trav %.2f / %.2f", traveledAngle, angle);

    currVelocity = this->thetaProfile.update(circumference * (remainingAngle / 360.0), dt);
    this->rotate(sign * currVelocity);

    if (traveledAngle >= angle) { break; }  // Overshoot prevention

    pros::delay(20);
  }
  if(settle) this->stop();
}

void HDrive::stop(){
  this->motors(0);
  this->sideways(0);
}

void HDrive::move(const double &dist, bool settle){
  driveProfiled(dist, settle);
}

void HDrive::strafe(const double &dist, bool settle){
  strafeProfiled(dist, settle);
}

void HDrive::turn(const double &angle, bool settle){
  turnProfiled(angle, settle);
}

void HDrive::setMaxVelocity(const double &rpm){
  this->yProfile.setMaxVelocity(rpm);
}

double HDrive::updateProfile(const double &distance, const double &dt){
  return this->yProfile.update(distance, dt);
}

void HDrive::driveInArc(double radius, const double &midSpeed) {
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

void HDrive::driveAngleOfArc(const double &radius, const double &angle, bool settle) {
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
  this->yProfile.setVelocity(this->getRPM());
  this->yProfile.setFinalVelocity(settle ? 0 : 100);

  // Timeout determined experimentally
  const uint32_t timeoutMs = (distance / 3.0) * 1E3;
  Timer timer;
  timer.start(timeoutMs);

  // const double startDist = odometry::getTraveledDistance();
  while(traveledDist < distance && !timer.isCompleted()){
    // traveledDist = odometry::getTraveledDistance() - startDist;
    const double rightEncDist = (std::abs(odometry->encoderRight.get_position() - rightEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    const double leftEncDist = (std::abs(odometry->encoderLeft.get_position() - leftEncStartPos) / 100 ) * M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION; //! Temporary
    traveledDist = (rightEncDist + leftEncDist) / 2; //! Temporary
    remainingDist = distance - traveledDist;
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    midSpeed = this->yProfile.update(remainingDist, dt);
    lastTime = now;

    this->driveInArc(radius, sign * midSpeed);

    pros::delay(20);
  }

  if (settle) this->stop();
}

void HDrive::driveInArcTo(const double &x, const double &y){
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

void HDrive::turnTo(const double &x, const double &y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Pose current = odometry->getPose();

  // Do the movement
  turn(-math::calculateTurn(target, current));
}

// TODO: replace with with the `goToPose()`
void HDrive::goTo(const double &x, const double &y){
  Vector target = Vector().SetPosition(x, y);
  // Determine current position
  Vector current = odometry->gpsPosition();

  // Do the movement
  turn(-math::calculateTurn(target, odometry->getPose()));
  move(math::findDistance(target, current));
}

// Using PID
// void HDrive::goToPose(const Pose& target){
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

//     this->leftMotors.moveVelocity(topRightDiag + turn);
//     this->rightMotors.moveVelocity(topLeftDiag - turn);
//     this->midMotors.moveVelocity(topLeftDiag + turn);
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
void HDrive::goToPose(const Pose& target){
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

    double xSign = remainingX / abs(remainingX);
    if(remainingX == 0) xSign = 1;
    double ySign = remainingY / abs(remainingY);
    if(remainingY == 0) ySign = 1;
    double thetaSign = remainingTheta / abs(remainingTheta);
    if(remainingTheta == 0) thetaSign = 1;

    remainingX = abs(remainingX);
    remainingY = abs(remainingY);
    remainingTheta = abs(remainingTheta);

    double x = this->xProfile.update(remainingX) * xSign;
    double y = this->yProfile.update(remainingY) * ySign;
    double theta = this->thetaProfile.update(circumference * (remainingTheta / 360.0)) * thetaSign;

    Vector direction = Vector().SetPosition(x, y);
    direction.SetDegrees(direction.GetDegrees() + this->getTheta());// - initialPose.theta);
    

    this->driveWhileTurning(direction.GetY(), theta);
    this->sideways(direction.GetX());

    pros::delay(delay);

    // this->setX(this->getX() + math::linearSpeed(x) * delay / 1000); //# in case of odom failure
    // this->setY(this->getY() + math::linearSpeed(y) * delay / 1000); //# in case of odom failure
    // this->setTheta(this->getTheta() + math::rotationalSpeed(theta) * delay / 1000); //# in case of odom failure
  }

  pros::lcd::clear();
  this->stop();
}

void HDrive::follow(const std::vector<Pose>& path) {
  PurePursuit controller = PurePursuit(this->yProfile, this->thetaProfile, 5, 0.5, 2.0);

  std::pair<double, double> output = {-1, -1};

  double dt = 0.02;
  double now = pros::micros() / 1E6;
  double lastTime = now;

  // Generous timeout
  const uint32_t timeoutMs = (math::length(path)) * 1E3;
  Timer timer;
  timer.start(timeoutMs);

  while (odometry->getPose().distanceTo(path.back()) > 2.0 && std::abs(odometry->getDegrees() - path.back().theta) > 5.0 && !timer.isCompleted()) {
    now = pros::micros() / 1E6;
    dt = now - lastTime;
    output = controller.follow(path, this->odometry->getPose(), dt);
    lastTime = now;
    this->leftMotors.moveVelocity(output.first);
    this->rightMotors.moveVelocity(output.second);

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

}  // namespace aon
