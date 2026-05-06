#include "../../include/aon/orbit/orbit.hpp"
#include "../../include/aon/tools/general.hpp"
namespace aon {

// Constructor

Orbit::Orbit(int rotationPort, bool reversedEncoder, int visionPort, int port)
    : encoder(rotationPort, reversedEncoder),
      vision_sensor(visionPort),
      motor(port) {}

// Functions :

/// @brief Adds the colors to the vision sensor
void Orbit::configure() {
  vision_sensor.set_signature(RED, &RED_SIG);
  vision_sensor.set_signature(BLUE, &BLUE_SIG);
  vision_sensor.set_signature(STAKE, &STAKE_SIG);

  motor.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  motor.setGearing(okapi::AbstractMotor::gearset::green);
  motor.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  motor.tarePosition();
}

void Orbit::stop() {
  motor.moveVelocity(0);
}

/// @brief Begins ORBIT following cycle
void Orbit::activateFollow() {
  following = true;
  braking = true;
  scanning = false;
}

/// @brief Ends ORBIT following cycle
void Orbit::deactivateFollow() { following = false; }

/// @brief Begins ORBIT scanning cycle
void Orbit::activateScan() {
  following = false;
  braking = false;
  scanning = true;
}

/// @brief Ends ORBIT scanning cycle
void Orbit::deactivateScan() { scanning = false; }

/// @brief Sets the ORBIT to brake if not scanning
void Orbit::brake() { braking = true; }

/// @brief Releases the ORBIT from braking to allow other functions to use it
void Orbit::release() { braking = false; }

/// @brief Async task to align ORBIT only to the item with the globally set
/// `COLOR` signature
void Orbit::follow() {
  const int TOLERANCE = 10;
  const int VISION_FIELD_CENTER = 315 / 2;
  int OBJ_CENTER;
  double position;

  while (true) {
    if (following) {
      auto object = vision_sensor.get_by_sig(0, COLOR);
      OBJ_CENTER = object.x_middle_coord;
      double SPEED = PID.Output(0, VISION_FIELD_CENTER - OBJ_CENTER);
      position = this->getAngle();

      if (object.signature == COLOR) {
        if (abs(OBJ_CENTER - VISION_FIELD_CENTER) <= TOLERANCE) {
          this->stop();
        }
        // Limiting to protect hardware
        else if (limited && (leftLimit >= position && position >= rightLimit)) {
          rotateAbsolute(nearest(
              position, std::make_pair(leftLimit + 10, rightLimit - 10)));
        } else {  // Turn Towards Object
          motor.moveVelocity(SPEED);
        }
      }
      // Dont move if nothing is there
      else {
        activateScan();
      }
    } else if (braking) {
      this->stop();
    }
    pros::delay(10);
  }
  this->stop();
}

void Orbit::rotateRelative(const double &givenAngle) {
  const double TOLERANCE = 5;
  double currentAngle;
  double initialAngle = encoder.get_position() / 100.0;
  double targetAngle = initialAngle + givenAngle;
  do {
    currentAngle = encoder.get_position() / 100.0;
    double output = PID.Output(targetAngle, currentAngle);
    motor.moveVelocity(output);
    pros::delay(10);
  } while (abs(currentAngle - targetAngle) > TOLERANCE);
  this->stop();
}

/// @brief ORBIT async task scanning test function
void Orbit::scan() {
  // To scan, make the ORBIT go from one side of its maximum FOV to the other,
  // if the ORBIT is not limited, make it go from 175° to 185° (going the long
  // way) if at any point the ORBIT detects an object, start following it and
  // stop scanning
  bool goingLeft = true;
  while (true) {
    if (isScanning() && !following) {
      deactivateFollow();  // redundant but ensures no fight for the vision
                           // sensor
      pros::vision_object object = vision_sensor.get_by_sig(0, COLOR);

      if (object.signature == COLOR) {
        // stop scanning and start following if we find something
        activateFollow();
      } else {
        double position = encoder.get_angle() / 100;
        // scan if we find nothing
        // Limiting to protect hardware (even if the rotation is 360°, we dont
        // want to twist the cable)
        if (leftLimit >= position && position >= rightLimit) {
          goingLeft = !goingLeft;
          // Make the ORBIT go to the nearest limit and keep rotating from there
          rotateAbsolute(nearest(
              position, std::make_pair(leftLimit + 20, rightLimit - 20)));
        }
        motor.moveVelocity(40 * (goingLeft ? -1 : 1));
      }
    } else if (isFollowing()) {
      deactivateScan();  // dont scan if the ORBIT following was activated
                         // elsewhere for some reason
    }  // an else would be redundant for our purposes
    pros::delay(20);
  }
}

/// @brief Rotates the ORBIT to a given angle, with respect to 0 degrees facing
/// forward. (Absolute Rotation)
/// @param targetAngle Angle in degrees we wish to rotate ORBIT. within [-180,
/// 180] or [0, 360]
/// @details `turretEncoder.get_angle()` is divided by 100 for scaling purposes.
void Orbit::rotateAbsolute(double targetAngle) {
  const double TOLERANCE = 5;
  if (targetAngle > 180) targetAngle -= 360;
  double currentAngle;
  do {
    currentAngle = encoder.get_angle() / 100.0;
    if (currentAngle > 180) currentAngle -= 360;
    double output = PID.Output(targetAngle, currentAngle);
    motor.moveVelocity(output);
    pros::delay(10);
  } while (abs(currentAngle - targetAngle) > TOLERANCE);
  this->stop();
}

bool Orbit::isAligned(const double &tolerance) {
  return abs(this->getAngle()) <= tolerance;
}

double Orbit::difference() {
  return (this->getAngle() < 180) ? this->getAngle() : (this->getAngle() - 360);
}

/// @brief Calculates the distance the robot would have to travel to get to an
/// object
/// @param pixels The pixels reported by the vision sensor viewing an object
/// (preferably width of that object)
/// @return The distance in \b inches that the robot is from the object,
/// probably to pass into the `move()` function
double Orbit::groundDistanceToDisk(const double &pixels){
  const double distance = widthToDistance(pixels);
  if (distance < this->getHeight()) {
    return distance;
  }  // avoid √(-1) issues if the ring is detected to be bigger than it should
     // be for some reason
  // pythagorean theorem: a^2 + b^2 = c^2
  // a = √(c^2 - b^2)
  return std::sqrt((distance * distance) -
                   (this->getHeight() * this->getHeight()));
}

/// @brief Converts the amount of `pixels` seen from the vision sensor, to the
/// corresponding \b inches
/// @param pixels The pixels reported by the vision sensor viewing an object
/// (preferably width of that object)
/// @return The corresponding amount of \b inches
double Orbit::pixelsToInches(const double &pixels) {
  const double CONSTANT =
      0.000208333;  // found experimentally be measuring the distance from the
                    // vision sensor to the object and using algebra to tune the
                    // value until consistent/realistic results were returned
  return pixels * CONSTANT;
}

/// @brief Calculates the distance in \b inches of an object based on its width
/// in \b pixels from the vision sensor
/// @param width The width of the object detected by the sensor in \b pixels
/// @return The distance from the vision sensor to the object in \b inches
/// @note This function assumes the entire object is in view, this may be
/// changed later
/// @details The funcion uses `pixelsToInches()` as a crucial part of the
/// calculations
/// @details The math is explained inside and the formulas are from optical
/// geometry
double Orbit::widthToDistance(const double &width) {
  // m = -i/d
  // m = w_i / w_o
  // d = |i| * (w_o / w_i)
  // w_i = pixels * CONSTANT
  // since i, w_o and CONSTANT are constants
  // then the formula technically is:
  // d = K / pixels
  // where K is a constant K = |i| * (w_o / CONSTANT)
  const double REAL_WIDTH = 3.25;
  const double DISTANCE_OF_IMAGE = 0.0625;  // estimated/experimental
  const double imageWidthInInches =
      pixelsToInches(width);  // also somewhat estimated/experimental
  const double distance = DISTANCE_OF_IMAGE * (REAL_WIDTH / imageWidthInInches);
  return distance;
}

/// @brief Calculates the distance to a ring of the specified `color` using a EKF
/// @param color The color of the ring we wish to track
/// @return The filtered distance to that ring
/// @note Takes half a second (0.5s) to complete
double Orbit::getDistanceToRing(Colors color){
  color = this->getColor();
  this->setColor(this->getColor());
  okapi::EKFFilter ekf;

  double distance;

  // Filter the distance for half a second using 100 measurements (1 every 5 milliseconds)
  for(int i = 0; i < 100; i++){
    distance = ekf.filter(groundDistanceToDisk((this->getLargestObject()).width));
    pros::delay(5);
  }

  return distance;
}

}  // namespace aon
