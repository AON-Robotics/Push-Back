#include "../include/aon/odometry/odometry.hpp"
#include <cstdio>    // fopen, fprintf, fclose
#include <cmath>     // sin, cos, fmod, abs
#include <random>    // mt19937, normal_distribution
#include <vector>    // vector

namespace aon {

// ─── Constructors ─────────────────────────────────────────────────────────────

Odometry::Odometry(short left, short right, short back, short gps, short gyro)
    : conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
      encoderLeft(abs(left), (left / abs(left) != 1)),
      encoderRight(abs(right), (right / abs(right) != 1)),
#if BACK_ENCODER_ENABLED
      encoderBack(abs(back), (back / abs(back) != 1)),
#endif
      gps(gps, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET,
          GPS_Y_OFFSET)
#if GYRO_ENABLED
      , gyroscope(gyro)
#endif
{}

Odometry::Odometry(const Odometry& other)
    : conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
      encoderLeft(other.encoderLeft),
      encoderRight(other.encoderRight),
#if BACK_ENCODER_ENABLED
      encoderBack(other.encoderBack),
#endif
      gps(other.gps)
#if GYRO_ENABLED
      , gyroscope(other.gyroscope)
#endif
{}


/// @brief Get current X position in inches
double Odometry::getX() {
  p_mutex.take(1);
  double currentX = position.GetX();
  p_mutex.give();
  return currentX;
}

/// @brief Get current Y position in inches
double Odometry::getY() {
  p_mutex.take(1);
  double currentY = position.GetY();
  p_mutex.give();
  return currentY;
}

/// @brief Get a vector with the current position
Vector Odometry::getPosition() {
  p_mutex.take(1);
  Vector pos = position;
  p_mutex.give();
  return pos;
}

/// @brief Set position in inches
void Odometry::setPosition(double x, double y) {
  p_mutex.take(1);
  position.SetPosition(x, y);
  p_mutex.give();
}

/// @brief Get current heading in degrees
double Odometry::getDegrees() {
  orientation_mutex.take(1);
  double currentDegrees = orientation.GetDegrees();
  orientation_mutex.give();
  return currentDegrees;
}

/// @brief Set current heading in degrees
void Odometry::setDegrees(double degrees) {
  orientation_mutex.take(1);
  orientation.SetDegrees(degrees);
  orientation_mutex.give();
  deltaTheta = 0.0;
}

/// @brief Get current heading in radians
double Odometry::getRadians() {
  orientation_mutex.take(1);
  const double currentRadian = orientation.GetRadians();
  orientation_mutex.give();
  return currentRadian;
}

/// @brief Set current heading in radians
void Odometry::setRadians(double radians) {
  orientation_mutex.take(1);
  orientation.SetRadians(radians);
  orientation_mutex.give();
}

/// @brief Get current pose (x, y, heading_deg)
Pose Odometry::getPose() { return Pose(getX(), getY(), getDegrees()); }

/// @brief Reset to INITIAL_ODOMETRY_X/Y/THETA constants
void Odometry::resetInitial() {
  resetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

/// @brief Initialization — zeroes encoders, resets pose, then loops update()
void Odometry::initialize() {
  encoderLeft.reset();
  encoderRight.reset();

  encoderLeft.set_position(0);
  encoderRight.set_position(0);

  #if BACK_ENCODER_ENABLED
  encoderBack.reset();
  encoderBack.set_position(0);
#endif

#if GYRO_ENABLED
  while (gyroscope.is_calibrating()) {
    pros::delay(20);
  }
  pros::delay(250); 
#endif
  
  resetInitial();


  while (true) {
    update();
    pros::delay(20);
  }
}

/**
 * @brief Core odometry function — reads sensors, calculates position,
 *        logs one CSV row to /usd/odom_log.csv on every call.
 */
void Odometry::update() {

  // ── Read encoder values (centidegrees → degrees) ─────────────────────────
  encoderRight_data.currentValue = encoderRight.get_position() / 100.0;
  encoderLeft_data.currentValue  = encoderLeft.get_position()  / 100.0;

  // ── Convert to distances ─────────────────────────────────────────────────
  encoderRight_data.currentDistance = encoderRight_data.currentValue * conversionFactor;
  encoderLeft_data.currentDistance  = encoderLeft_data.currentValue  * conversionFactor;

  // ── Calculate deltas ─────────────────────────────────────────────────────
  encoderRight_data.delta = encoderRight_data.currentValue - encoderRight_data.prevValue;
  encoderLeft_data.delta  = encoderLeft_data.currentValue  - encoderLeft_data.prevValue;

  encoderRight_data.deltaDistance = encoderRight_data.currentDistance
                                    - encoderRight_data.previousDistance;
  encoderLeft_data.deltaDistance  = encoderLeft_data.currentDistance
                                    - encoderLeft_data.previousDistance;

#if BACK_ENCODER_ENABLED
  // ── Back encoder ─────────────────────────────────────────────────────────
  encoderBack_data.currentValue    = encoderBack.get_position() / 100.0;
  encoderBack_data.currentDistance = encoderBack_data.currentValue * conversionFactor;
  encoderBack_data.delta           = encoderBack_data.currentValue
                                     - encoderBack_data.prevValue;
  encoderBack_data.deltaDistance   = encoderBack_data.currentDistance
                                     - encoderBack_data.previousDistance;
#endif

  // ── Encoder-only deltaTheta (used directly when GYRO_ENABLED = 0) ────────
  double deltaTheta = (encoderRight_data.deltaDistance - encoderLeft_data.deltaDistance)
                      / (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER
                         + DISTANCE_LEFT_TRACKING_WHEEL_CENTER);

#if GYRO_ENABLED
  double rawHeading = gyroscope.get_heading();

  // Guard: PROS returns PROS_ERR_F (infinity) when the IMU is not ready.
  // Skip the gyro entirely this step and fall back to encoder-only deltaTheta.
  if (!std::isinf(rawHeading) && !std::isnan(rawHeading)) {
    // ── Read gyro; negate so CCW is positive ─────────────────────────────────
    gyro_data.currentDegrees = -gyroscope.get_heading();
    gyro_data.currentRadians  = gyro_data.currentDegrees * (M_PI / 180.0);

    // ── Normalise heading to (-180, 180] ─────────────────────────────────────
    if (gyro_data.currentDegrees >  180.0) gyro_data.currentDegrees -= 360.0;
    if (gyro_data.currentDegrees <= -180.0) gyro_data.currentDegrees += 360.0;


    // ── Gyro delta then unwrap across the ±180° boundary ─────────────────────
    gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
    if (gyro_data.deltaDegrees >  180.0) gyro_data.deltaDegrees -= 360.0;
    if (gyro_data.deltaDegrees < -180.0) gyro_data.deltaDegrees += 360.0;
    // [FIX 3] Radians computed AFTER the unwrap (was before it in original)
    gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);

    gyro_data.prevDegrees = gyro_data.currentDegrees;

    // ── Fuse encoder and gyro (GYRO_CONFIDENCE ∈ [0,1]) ──────────────────────
    deltaTheta = (1.0 - GYRO_CONFIDENCE) * deltaTheta
                + GYRO_CONFIDENCE * gyro_data.deltaRadians;
  }
#endif
  // Midpoint of the arc swept this step. The original formula
  // (getRadians() + deltaTheta)/2 was only correct when starting at 0°.
  double thetaMid = getRadians() + deltaTheta / 2.0;
  setRadians(getRadians() + deltaTheta);

  // If we are turning on our axis, dont update anything
  if (this->turn) {
    // ── Save previous values for next call ───────────────────────────────────
    encoderLeft_data.prevValue          = encoderLeft_data.currentValue;
    encoderRight_data.prevValue         = encoderRight_data.currentValue;
    encoderLeft_data.previousDistance   = encoderLeft_data.currentDistance;
    encoderRight_data.previousDistance  = encoderRight_data.currentDistance;
    #if BACK_ENCODER_ENABLED
      encoderBack_data.prevValue          = encoderBack_data.currentValue;
      encoderBack_data.previousDistance   = encoderBack_data.currentDistance;
    #endif
    return;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // deltaDlocal — arc-math approach
  //   Branch on motion type, compute local-frame displacement, rotate global.
  // ─────────────────────────────────────────────────────────────────────────
  if (std::abs(deltaTheta) > TURNING_THRESHOLD) {
    // ── Arc motion ───────────────────────────────────────────────────────────
    double sign       = (deltaTheta > 0.0) ? 1.0 : -1.0;
    double radiusLeft = encoderLeft_data.deltaDistance  / deltaTheta
                        + sign * DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
    double radiusRight= encoderRight_data.deltaDistance / deltaTheta
                        - sign * DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
    double averageR   = (radiusLeft + radiusRight) / 2.0;

    deltaDlocal.SetPosition(averageR * std::sin(deltaTheta),
                            averageR * (1.0 - std::cos(deltaTheta)));
#if BACK_ENCODER_ENABLED
    // Back encoder gives direct lateral displacement
    deltaDlocal.SetPosition(averageR * std::sin(deltaTheta),
        encoderBack_data.deltaDistance
        - DISTANCE_BACK_TRACKING_WHEEL_CENTER * deltaTheta);
#endif

  } else {
    // ── Straight line (rotation below threshold) ─────────────────────────────
    double dL_corr = encoderLeft_data.deltaDistance
                   - DISTANCE_LEFT_TRACKING_WHEEL_CENTER  * deltaTheta;
    double dR_corr = encoderRight_data.deltaDistance
                   - DISTANCE_RIGHT_TRACKING_WHEEL_CENTER * deltaTheta;
    double deltaD = (dL_corr + dR_corr) / 2.0;
    deltaDlocal.SetPosition(deltaD, 0.0);
#if BACK_ENCODER_ENABLED
    double deltaY = encoderBack_data.deltaDistance
                    - DISTANCE_BACK_TRACKING_WHEEL_CENTER * deltaTheta;
    deltaDlocal.SetPosition(deltaD, deltaY);
#endif
  }

  // ── Rotate local displacement to global frame and update position ─────────
  position.SetPosition(
      getX() + deltaDlocal.GetX() * std::cos(thetaMid)
             - deltaDlocal.GetY() * std::sin(thetaMid),
      getY() + deltaDlocal.GetX() * std::sin(thetaMid)
             + deltaDlocal.GetY() * std::cos(thetaMid));


  // ── Save previous values for next call ───────────────────────────────────
  encoderLeft_data.prevValue          = encoderLeft_data.currentValue;
  encoderRight_data.prevValue         = encoderRight_data.currentValue;
  encoderLeft_data.previousDistance   = encoderLeft_data.currentDistance;
  encoderRight_data.previousDistance  = encoderRight_data.currentDistance;
#if BACK_ENCODER_ENABLED
  encoderBack_data.prevValue          = encoderBack_data.currentValue;
  encoderBack_data.previousDistance   = encoderBack_data.currentDistance;
#endif
}

void Odometry::resetCurrent(double x, double y, double theta) {
  const double currentAngleRight = encoderRight.get_position() / 100.0;
  const double currentAngleLeft  = encoderLeft.get_position()  / 100.0;
#if BACK_ENCODER_ENABLED
  const double currentAngleBack  = encoderBack.get_position()  / 100.0;
#endif
  const double currentAngleGyro  = gyroscope.get_heading();

  encoderRight_data = {
      currentAngleRight, currentAngleRight, 0,
      currentAngleRight * conversionFactor,
      currentAngleRight * conversionFactor, 0.0};

  encoderLeft_data = {
      currentAngleLeft, currentAngleLeft, 0,
      currentAngleLeft * conversionFactor,
      currentAngleLeft * conversionFactor, 0.0};

#if BACK_ENCODER_ENABLED
  encoderBack_data = {
      currentAngleBack, currentAngleBack, 0,
      currentAngleBack * conversionFactor,
      currentAngleBack * conversionFactor, 0.0};
#endif

  gyro_data  = {0, 0, 0, 0.0, 0.0};
  deltaTheta = 0.0;
  deltaDlocal.SetPosition(0.0, 0.0);

  setDegrees(theta);
  setPosition(x, y);

#if GYRO_ENABLED
  gyroscope.reset();
  gyroscope.tare();
  pros::delay(3000);
#endif
}

// ─── gpsPosition ──────────────────────────────────────────────────────────────

/// @brief Returns GPS position as a Vector
Vector Odometry::gpsPosition() {
  pros::delay(2000);
  pros::c::gps_status_s_t status = gps.get_status();
  Vector current = Vector().SetPosition(status.x, status.y);
  return current;
}

// ─── debug ────────────────────────────────────────────────────────────────────

void Odometry::debug() {
  while (true) {
    pros::lcd::print(0, "Left : %0.3f, %0.3f, %0.3f",
                     encoderLeft_data.currentDistance,
                     encoderLeft_data.previousDistance,
                     encoderLeft_data.deltaDistance);
    pros::lcd::print(1, "Right: %0.3f, %0.3f, %0.3f",
                     encoderRight_data.currentDistance,
                     encoderRight_data.previousDistance,
                     encoderRight_data.deltaDistance);
#if BACK_ENCODER_ENABLED
    pros::lcd::print(2, "Back: %0.3f, %0.3f, %0.3f",
                     encoderBack_data.currentDistance,
                     encoderBack_data.previousDistance,
                     encoderBack_data.deltaDistance);
#endif
    pros::lcd::print(3, "Heading: %0.3f", getDegrees());
    pros::lcd::print(4, "arc: X: %0.3f | Y: %0.3f", getX(), getY());
    update();
    pros::delay(20);
  }
}

}  // namespace aon