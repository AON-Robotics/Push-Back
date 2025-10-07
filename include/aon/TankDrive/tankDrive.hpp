#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"
#include "../constants.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../math/misc/misc.hpp"

// Forward declaration to avoid circular dependency

namespace aon {
class TankDrive {
    
private:

    okapi::MotorGroup driveLeft;
    okapi::MotorGroup driveRight;
    okapi::MotorGroup driveFull;
    aon::PID drivePID;
    aon::PID turnPID;
    MotionProfile forwardProfile;

public:

    TankDrive(const std::initializer_list<okapi::Motor> &leftMotors = {-20, 19, -18},
              const std::initializer_list<okapi::Motor> &rightMotors = {9, -8, 7},
              const std::initializer_list<okapi::Motor> &fullMotors = {-20, 19, -18, 9, -8, 7});
    /**
     * \brief Moves the robot a given distance using PID control
     * \param dist The distance to be moved in inches
     * \param MAX_REVS The maximum RPM to send to the movement
     */
    void moveDrivePID(double dist = TILE_WIDTH, const double &MAX_REVS = 100.0);

    /**
     * \brief Turns the robot by a given angle using PID control
     * \param angle The angle to make the robot turn in degrees
     * \param MAX_REVS The maximum RPM to send to the movement
     */     
    void moveTurnPID(double angle = 90, const double &MAX_REVS = 50.0);

    /**
     * \brief Turns the robot towards a specific direction
     * \param x The x component of the point we wish to face
     * \param y The y component of the point we wish to face
     * \note Uses coordinate system from GPS in meters
     */
    void turnToTarget(double x, double y);

    /**
     * \brief Goes to the target point
     * \param x The x component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
     * \param y The y component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
     * \note Uses coordinate system from GPS in meters
     */
    void goToTarget(double x, double y);

    /**
     * \brief S-graph motion profile for linear movement
     * \param dist The distance to be moved in inches, positive values will move forward and negative values backwards
     */
    void motionProfile(double dist = TILE_WIDTH);

    /**
     * \brief S-graph motion profile for rotations
     * \param angle The angle in degrees we wish to rotate the robot, positive is clockwise and negative is counterclockwise
     */
    void turnProfile(double angle = 90);

    /**
     * \brief Moves the robot straight across a given amount of tiles
     * \param amt The amount of tiles to be driven
     * \attention To move in reverse make the amount negative
     * \warning Robot should be aligned properly to achieve desired result
     */
    void moveTilesStraight(double amt = 1);

    /**
     * \brief Moves the robot straight across a given amount of half-tiles
     * \param amt The amount of half-tiles to be driven
     * \attention To move in reverse make the amount negative
     * \warning Robot should be aligned properly to achieve desired result
     */
    void moveHalfTiles(int amt = 1);

    /**
     * \brief Moves the robot diagonally across a given amount of tiles
     * \param amt The amount of tiles to be driven
     * \attention To move in reverse make the amount negative
     * \warning Robot should be aligned properly to achieve desired result
     */
    void moveTilesDiag(int amt = 1);

    /**
     * \brief Moves the robot diagonally across a given amount of half-tiles
     * \param amt The amount of half-tiles to be driven
     * \attention To move in reverse make the amount negative
     * \warning Robot should be aligned properly to achieve desired result
     */
    void moveHalfDiagTiles(int amt = 1);

    /**
     * \brief Turns the robot clockwise by 90 degrees
     * \param amt The amount of 90 degree turns to make
     * \attention To turn counterclockwise make the amount negative
     */
    void turn90(int amt = 1);

    /**
     * \brief Moves the robot a given distance
     * \param dist The distance to move in inches
     * \details A positive dist makes the robot go forward while a negative dist makes the robot go backwards
     * \returns 1 on completion
     */
    int move(const double &dist = TILE_WIDTH);

    /**
     * \brief Turn the robot a given angle
     * \param angle The angle to turn in degrees
     * \details Clockwise is positive and counterclockwise is negative
     * \returns 1 on completion
     */
    int turn(const double &angle = 90);

    /**
     * \brief Moves the robot forward/backward while turning simultaneously
     * \param forward The forward velocity in RPM (positive = forward, negative = backward)
     * \param turn The turn velocity in RPM (positive = turn right, negative = turn left)
     */
    void moveAndTurn(double forward, double turn);

    /**
     * \brief Stops movement from robot
     */
    void STOP();

    /**
     * \brief Internal PID drive function
     * \param pid The PID used for the driving
     * \param dist The distance to be moved in inches
     * \param MAX_REVS The maximum RPM to send to the movement
     */
    void MoveDrivePID(PID pid, double dist, const double &MAX_REVS);

    /**
     * \brief Internal PID turn function
     * \param pid The PID to be used for the turn
     * \param angle The angle to make the robot turn in degrees
     * \param MAX_REVS The maximum RPM to send to the movement
     */
    void MoveTurnPID(PID pid, double angle, const double &MAX_REVS);

    /**
     * \brief Calculates the angle needed to be turned in order to face a specific point in the field
     * \param target The point we wish to face
     * \param current Where the robot is now
     * \returns The angle the robot needs to turn in order to face the target location
     */
    double calculateTurn(Vector target, Vector current);


};

// Global instance
extern TankDrive tankDrive;
} // namespace aon