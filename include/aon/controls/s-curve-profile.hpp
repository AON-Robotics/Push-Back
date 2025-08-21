/**
 * @file s-curve-profile.hpp
 * @author Kevin Javier Gomez Guzman @kevgom018
 * @brief S-Curve motion profiling class for smooth and precise robot movements based on odometry.
 * @version 1.0
 * @date 2025-06-19
 * 
 * This class calculates target velocities using an S-curve motion profile to ensure smooth acceleration and deceleration.
 * It improves movement accuracy and reduces overshoot by carefully controlling jerk and acceleration phases.
 */

#pragma once

#include <cmath>
#include "../constants.hpp"
#include "../globals.hpp"

inline double getSpeed(const double &RPM);

class MotionProfile {

    double MAX_VELOCITY, MAX_ACCELERATION, MAX_DECELERATION, JERK;
    double currVelocity = 0;
    double currAccel = 0;

    public:

        MotionProfile(double MAX_VELOCITY, double MAX_ACCELERATION, double MAX_DECELERATION, double JERK) {
            this->MAX_VELOCITY = std::abs(MAX_VELOCITY); // RPM
            this->MAX_ACCELERATION = std::abs(MAX_ACCELERATION); // RPM/s
            this->MAX_DECELERATION = std::abs(MAX_DECELERATION); 
            this->JERK = std::abs(JERK); // RPM/(s^2)
        }

        /// @brief Calculates the target velocity to send to the motors for smooth and precise movements using an S-curve profile.
        /// @param remainingDist The remaining distance to the target in \b inches.
        /// @param dt The time elapsed since the last function call in \b seconds.
        /// @return The updated velocity in \b RPM.
        double update(const double &remainingDist, const double &dt = 0.02) {

            // Deceleration
            // Decelerate early using half the deceleration for better accuracy.
            // Note: Without this, the system consistently overshoots by about half an inch. Needs further investigation.
            if(remainingDist <= getSpeed(this->currVelocity) * getSpeed(this->currVelocity) / (2 * getSpeed(this->MAX_DECELERATION * .5))){
                this->currAccel = - this->MAX_DECELERATION;
            }
            // Decelerate if the current velocity exceeds the updated `MAX_VELOCITY`.
            else if (this->currVelocity > this->MAX_VELOCITY){
                this->currAccel = std::min(this->currAccel - (this->JERK * dt), this->MAX_DECELERATION);
            }
            // Maintain constant velocity (no acceleration needed).
            else if (this->currVelocity == this->MAX_VELOCITY) {
                this->currAccel = 0;
            }
            // Stop the profile when the target is reached.
            else if(remainingDist <= 0) {
                this->currAccel = 0;
                this->currVelocity = 0;
            }
            // Increase acceleration up to the maximum allowed value.
            else {
                this->currAccel = std::min(this->currAccel + (this->JERK * dt), this->MAX_ACCELERATION);
            }

            this->currVelocity += this->currAccel * dt;
            this->currVelocity = std::min(this->currVelocity,  this->MAX_VELOCITY); // Redundancy for safety
            return this->currVelocity;
        }

        /// @brief Resets the velocity and acceleraton for reusability
        void reset(){
            this->currVelocity = 0;
            this->MAX_VELOCITY = MAX_RPM;
            this->currAccel = 0;
        }

        /// @brief Sets the velocity in case profile is not started from rest
        /// @param velocity The current velocity of the profile
        void setVelocity(const double &velocity = 0) {
            this->currVelocity = velocity;
        }

        /// @brief Sets the max velocity for the profile
        /// @param max_velocity The new max velocity for the profile
        void setMaxVelocity(const double &max_velocity = MAX_RPM) {
            this->MAX_VELOCITY = max_velocity;
        }

        /// @brief Sets the acceleration in case profile is not started from rest
        /// @param accel The current acceleration of the profile
        void setAccel(const double &accel = 0) {
            this->currAccel = accel;
        }
};

/// @brief Determines the speed of the robot given drivetrain motors' `RPM`
/// @param RPM The RPM for which to calculate the velocity (default current RPM)
/// @return The speed in \b in/s at which the robot would move at the given RPM
/// @note Test the accuracy precision of the `getActualVelocity()` method which is used as a default value,
/// @note it may be possible to need to use `get_velocity()` from `pros::Rotation` which uses \b centidegrees.
/// @note The distance units depend on the units used for measuring `DRIVE_WHEEL_DIAMETER`.
inline double getSpeed(const double &RPM = (int)driveFull.getActualVelocity()){
  double circumference = DRIVE_WHEEL_DIAMETER * M_PI;
  double RPS = RPM / 60;
  double speed = MOTOR_TO_DRIVE_RATIO * circumference * RPS;
  return speed;
}