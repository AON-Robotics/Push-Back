#pragma once

#ifndef AON_SENSING_ODOMETRY_HPP_
#define AON_SENSING_ODOMETRY_HPP_

#include <cmath>
#include <iostream>
#include "../constants.hpp"
#include "../../api.h"
#if GYRO_ENABLED
#include "../../okapi/api.hpp"
#endif
#include "../tools/vector.hpp"
#include "../math/pose.hpp"

#include <cstdio>


namespace aon {

    /**
     * \struct  ENCODER 
     *
     * \brief Store encoder data from current and previous odometry
     * iterations.
     * */
    struct ENCODER {
        double currentValue;
        double prevValue;
        double delta;
        double currentDistance;
        double previousDistance;
        double deltaDistance;

    };
    /**
     * \struct GYRO
     *
     * \brief Store gyro data from current and previous odometry
     * iterations.
     * */
    struct GYRO {
        double currentDegrees;
        double prevDegrees;
        double currentRadians;
        double deltaRadians;
        double deltaDegrees;

    };

    class Odometry {

    private:
        double deltaTheta;
        Vector deltaDlocal;
        Angle orientation;
        Vector position;

        Vector changeMine; // testing
        
        const double conversionFactor;

        ENCODER encoderBack_data;
        ENCODER encoderRight_data;
        ENCODER encoderLeft_data;
        GYRO gyro_data;


        pros::Mutex p_mutex;
        pros::Mutex orientation_mutex;

    public: 
        Odometry(short left, short right, short back, short gps, short gyro);
        Odometry(const Odometry& other);

        double getX();
        double getY();
        void setX(double x);
        void setY(double y);

        FILE* _logFile = nullptr;

        Vector getPosition();
        void setPosition(double x, double y);

        double getDegrees();
        void setDegrees(double degrees);

        double getRadians();
        void setRadians(double radians);

        // MAIN functions
        void resetInitial();
        void initialize();
        void update();
        void resetCurrent(double x, double y, double theta);
        Vector gpsPosition();
        Pose getPose();


        //Threading
        void sense();

        //Debugging/Testing
        void debug();

        pros::Rotation encoderRight;
        pros::Rotation encoderLeft;//was in private 
        #if ENCODER_BACK_ENABLED
        pros::Rotation encoderBack;
        #endif  
        pros::Gps gps;

        #if GYRO_ENABLED
        pros::Imu gyroscope;
        #endif

    

    };
}
#endif
