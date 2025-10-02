#include "../include/aon/controls/TankDrive/tankDrive.hpp"
#include "../include/aon/globals.hpp"
#include "../include/aon/sensing/odometry.hpp"
#include "../include/aon/controls/s-curve-profile.hpp"

namespace aon {
// Global forwardProfile instance
MotionProfile forwardProfile(MAX_RPM, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
}

namespace aon {
// Global TankDrive instance
TankDrive tankDrive;

TankDrive::TankDrive(const std::initializer_list<okapi::Motor> &leftMotors, const std::initializer_list<okapi::Motor> &rightMotors, const std::initializer_list<okapi::Motor> &fullMotors):
    driveLeft(leftMotors),
    driveRight(rightMotors),
    driveFull(fullMotors),
    drivePID(TANK_DRIVE_PID_KP, TANK_DRIVE_PID_KI, TANK_DRIVE_PID_KD),
    turnPID(TANK_TURN_PID_KP, TANK_TURN_PID_KI, TANK_TURN_PID_KD) {

    // Initialize motor groups for drivetrain
    driveLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    driveLeft.setGearing(okapi::AbstractMotor::gearset::blue);
    driveLeft.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    driveLeft.tarePosition();
    
    driveRight.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    driveRight.setGearing(okapi::AbstractMotor::gearset::blue);
    driveRight.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    driveRight.tarePosition();
    
    driveFull.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    driveFull.setGearing(okapi::AbstractMotor::gearset::blue);
    driveFull.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
    driveFull.tarePosition();
}

void TankDrive::moveDrivePID(double dist, const double &MAX_REVS) {
    MoveDrivePID(drivePID, dist, MAX_REVS);
}

void TankDrive::moveTurnPID(double angle, const double &MAX_REVS) {
    MoveTurnPID(turnPID, angle, MAX_REVS);
}

// ------------------------------------------------------------ PID Functions ------------------------------------------------------------
void TankDrive::MoveDrivePID(PID pid, double dist, const double &MAX_REVS) {
    const int sign = dist / abs(dist); // Getting the direction of the movement
    dist = abs(dist); // Setting the magnitude to positive
    pid.Reset();
    aon::Vector initialPos = aon::odometry::GetPosition();
    const double timeLimit = getTimetoTarget(dist, MAX_REVS);
    const double start_time = pros::micros() / 1E6;
#define time \
    (pros::micros() / 1E6) - start_time // every time the variable is called it is
                                         // recalculated automatically
    // while (time < timeLimit) {
    while ((aon::odometry::GetPosition() - initialPos).GetMagnitude() < dist) {
        double currentDisplacement = (aon::odometry::GetPosition() - initialPos).GetMagnitude();
        double output = pid.Output(dist, currentDisplacement);
        pros::lcd::print(0, "Time Limit %.2f", timeLimit);
        pros::lcd::print(1, "Time: %.2f", time);
        pros::lcd::print(2, "Odometry Displacement %.2f", currentDisplacement);
        driveFull.moveVelocity(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
        pros::delay(10);
    }
    // Stop the motors
    driveFull.moveVelocity(0);
#undef time
}

void TankDrive::MoveTurnPID(PID pid, double angle, const double &MAX_REVS) {
    const int sign = angle / abs(angle); // Getting the direction of the movement
    angle = abs(angle); // Setting the magnitude to positive
    pid.Reset();
    gyroscope.tare(); // .tare() or .reset(true) depending on the time issue
    const double startAngle = odometry::GetDegrees(); // Angle relative to the start
    double timeLimit = getTimetoTurnDeg(angle);
    if (sign == -1) {
        angle = 360.0 - angle + CLOCKWISE_ROTATION_DEGREES_OFFSET;
    }
    if (sign == 1) {
        angle -= CLOCKWISE_ROTATION_DEGREES_OFFSET;
    }
    const double startTime = pros::micros() / 1E6;
#define time (pros::micros() / 1E6) - startTime
    while (time < 3 * timeLimit) {
        double traveledAngle = abs(odometry::GetDegrees() - startAngle);
        double output = pid.Output(angle, traveledAngle);
        pros::lcd::print(0, "Time Limit %.2f", timeLimit);
        pros::lcd::print(1, "Time: %.2f", time);
        pros::lcd::print(2, "Gyroscope Displacement %.2f", traveledAngle);
        // Taking clockwise rotation as positive (to change this just flip the
        // negative on the sign below)
        driveLeft.moveVelocity(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
        driveRight.moveVelocity(-sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
        pros::delay(10);
    }
    driveLeft.moveVelocity(0);
    driveRight.moveVelocity(0);
#undef time
}

void TankDrive::turnToTarget(double x, double y) {
    Vector target = Vector().SetPosition(x, y);
    // Determine current position
    Vector current = position();
    // Do the movement
    turn(-calculateTurn(target, current));
}

void TankDrive::goToTarget(double x, double y) {
    Vector target = Vector().SetPosition(x, y);
    // Determine current position
    Vector current = position();
    // Do the movement
    turn(-calculateTurn(target, current));
    move(findDistance(target, current));
}

// ------------------------------------------------------------ Motion Profile Functions ------------------------------------------------------------
void TankDrive::motionProfile(double dist) {
    if (dist == 0) {
        return;
    }
    const int sign = dist / abs(dist); // Getting the direction of the movement
    dist = abs(dist); // Setting the magnitude to positive
    double dt = 0.02; // (s)
    double currVelocity = 0;
    double traveledDist = 0;
    Vector startPos = aon::odometry::GetPosition();
    double now = pros::micros() / 1E6;
    double lastTime = now;
    forwardProfile.setVelocity(driveFull.getActualVelocity());
    while (traveledDist < dist) {
        traveledDist = (aon::odometry::GetPosition() - startPos).GetMagnitude();
        double remainingDist = dist - traveledDist;
        now = pros::micros() / 1E6;
        dt = now - lastTime;
        lastTime = now;
        currVelocity = forwardProfile.update(remainingDist, dt);
        driveFull.moveVelocity(sign * currVelocity);
        if (traveledDist >= dist) {
            break;
        } // Overshoot prevention
        pros::delay(20);
    }
    driveFull.moveVelocity(0);
}

void TankDrive::turnProfile(double angle) {
    if (angle == 0) {
        return;
    }
    const int sign = angle / abs(angle); // Getting the direction of the movement
    angle = abs(angle); // Setting the magnitude to positive
    const double circumference = DRIVE_LENGTH * M_PI; // Of the robot's rotation, used in the condition to
                                                     // calculate the length of arc remaining
    const double MAX_VELOCITY = MAX_RPM; // (RPM)
    const double MAX_JERK = MAX_ACCEL; // (RPM/s^2)
    double dt = 0.02; // (s)
    double currVelocity = 0;
    double currAccel = 0;
    double traveledAngle = 0;
    double startAngle = aon::odometry::GetDegrees();
    double now;
    double lastTime = pros::micros() / 1E6;
    while (traveledAngle < angle) {
        traveledAngle = abs(aon::odometry::GetDegrees() - startAngle);
        double remainingAngle = angle - traveledAngle;
        now = pros::micros() / 1E6;
        dt = now - lastTime;
        lastTime = now;
        // Debugging output to brain
        pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
        pros::lcd::print(2, "RPM: %.2f, Accel: %.2f", currVelocity, currAccel);
        pros::lcd::print(3, "Remaining: %.2f", remainingAngle);
        pros::lcd::print(4, "Calculated Velocity: %.2f", getSpeed(currVelocity));
        pros::lcd::print(5, "Max Velocity: %.2f", getSpeed(MAX_VELOCITY));
        // Acceleration
        // For the condition, consider half the deceleration for accuracy (there is
        // an error of half an inch almost constant when not used, I have to
        // investigate a bit further on that part but if works fine like this)
        if (circumference * (remainingAngle / 360.0) <= getSpeed(currVelocity) * getSpeed(currVelocity) / (2.0 * getSpeed(MAX_DECEL * 0.5))) {
            currAccel = -MAX_DECEL;
        } else {
            currAccel = std::min(currAccel + (MAX_JERK * dt), MAX_ACCEL);
        }
        currVelocity += currAccel * dt;
        currVelocity = std::min(currVelocity, MAX_VELOCITY);
        driveLeft.moveVelocity(sign * currVelocity);
        driveRight.moveVelocity(-sign * currVelocity);
        if (traveledAngle >= angle) {
            break;
        } // Overshoot prevention
        pros::delay(20);
    }
    driveFull.moveVelocity(0);
}

// ------------------------------------------------------------ Tile Movement Functions ------------------------------------------------------------
void TankDrive::moveTilesStraight(double amt) {
    move(TILE_WIDTH * amt);
}

void TankDrive::moveHalfTiles(int amt) {
    move((TILE_WIDTH / 2) * amt);
}

void TankDrive::moveTilesDiag(int amt) {
    move(TILE_DIAG_LENGTH * amt);
}

void TankDrive::moveHalfDiagTiles(int amt) {
    move((TILE_DIAG_LENGTH / 2) * amt);
}

void TankDrive::turn90(int amt) {
    turn(90 * amt);
}

// ------------------------------------------------------------ Basic Movement Functions ------------------------------------------------------------
int TankDrive::move(const double &dist) {
    motionProfile(dist);
    return 1;
}

int TankDrive::turn(const double &angle) {
    turnProfile(angle);
    return 1;
}

void TankDrive::moveAndTurn(double forward, double turn) {
    // Calculate differential speeds for tank drive
    double leftSpeed = forward + turn;
    double rightSpeed = forward - turn;
    // Apply the speeds to the motor groups
    driveLeft.moveVelocity(leftSpeed);
    driveRight.moveVelocity(rightSpeed);
}

//////////////////////////////////////// Nota: Posiblemente necesite relocalización ///////////////////////////////////////////////
/////////////////////////////////////////////////////// HELPER FUNCTIONS //////////////////////////////////////////////////////////
double TankDrive::findDistance(Vector target, Vector current) {
    double distInMeters = (target - current).GetMagnitude();
    return metersToInches(distInMeters);
}

double TankDrive::calculateTurn(Vector target, Vector current) {
    // Get and change the heading to the common cartesian plane
    double heading = 90 - gps.get_heading();
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
    double angle = toTarget - heading; // Calculate the angle to turn
    // Limiting the heading to the -180-180 range
    if (angle > 180)
        angle -= 360;
    else if (angle < -180)
        angle += 360;
    return angle;
}

double TankDrive::metersToInches(const double &meters) {
    return meters * 39.3701;
}

double TankDrive::inchesToMeters(const double &inches) {
    return inches / 39.3701;
}

void TankDrive::STOP() {
    driveFull.moveVelocity(0);
    driveLeft.moveVelocity(0);
    driveRight.moveVelocity(0);
}

Vector TankDrive::position() {
    STOP();
    pros::delay(2000);
    pros::c::gps_status_s_t status = gps.get_status();
    Vector current = Vector().SetPosition(status.x, status.y);
    return current;
}

double TankDrive::getTimetoTarget(const double &distance, const double &RPM) {
    double time = 4 * distance / getSpeed(RPM);
    return time;
}

double TankDrive::getTimetoTurnDeg(const double &degrees) {
    return getTimetoTurnRad(degrees * M_PI / 180, MAX_RPM / 4);
}

double TankDrive::getTimetoTurnRad(const double &radians, const double &RPM) {
    double arcLength = radians * AVG_DRIVETRAIN_RADIUS; // Of the turn (inches)
    double time = 2 * arcLength / getSpeed(RPM); // Calculated time (seconds)
    return time;
}
} // namespace aon