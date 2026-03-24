#include "../include/aon/odometry/odometry.hpp"

namespace aon{

    Odometry::Odometry() 
    :
    conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
    encoderRight(15, true),
    encoderLeft(14, false),
    encoderBack(16, true),
    gps(20, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING, GPS_X_OFFSET, GPS_Y_OFFSET)
    #if GYRO_ENABLED
    , gyroscope(1)
    #endif
    {}

    //GETTER & SETTERS


    /**
     * \brief Get current X position in \b inches
     *
     * \returns Returns current X position in \b inches
     */
    double Odometry::getX(){
        p_mutex.take(1);
        double currentX = position.GetX();
        p_mutex.give();
        return currentX;
    }

    /**
     * \brief Set current X position in \b inches
     */
    void Odometry::setX(double x){
        p_mutex.take(1);
        position.SetPosition(x, getY());
        p_mutex.give();
    }

    /**
     * \brief Get current Y position in \b inches
     *
     * \returns Returns current Y position in \b inches
     */
    double Odometry::getY(){
        p_mutex.take(1);
        double currentY = position.GetY();
        p_mutex.give();
        return currentY;
    }

    /**
     * \brief Set current Y position in \b inches
     */
    void Odometry::setY(double y){
        p_mutex.take(1);
        position.SetPosition(getX(), y);
        p_mutex.give();
    }

    /**
     * \brief Get a vector with the current position
     *
     * \return Returns new vector with current position
     */
    Vector Odometry::getPosition(){
        p_mutex.take(1);
        Vector pos = position;
        p_mutex.give();
        return pos;
    }


    /**
     * \brief Set Y position in \b inches
     *
     * \param x Value on X to set new position
     * \param y Value on Y to set new position
     */
    void Odometry::setPosition(double x, double y){
        p_mutex.take(1);
        position.SetPosition(x, y);
        p_mutex.give();
    }


    /**
     * \brief Get current pose's angle in \b degrees
     *
     * \returns Returns current pose's angle in \b degrees
     */
    double Odometry::getDegrees(){
        orientation_mutex.take(1);
        double currentDegrees = orientation.GetDegrees();
        orientation_mutex.give();
        return currentDegrees;
    }


    /**
     * \brief Set current pose's angle in \b degrees
     *
     * \param degrees Input value to set the current angle to
     */
    void Odometry::setDegrees(double degrees){
        orientation_mutex.take(1);
        orientation.SetDegrees(degrees);
        orientation_mutex.give();
        deltaTheta = 0.0;
    }


    /**
     * \brief Get current pose's angle in \b radians
     *
     * \returns Returns current pose's angle in \b radians
     */
    double Odometry::getRadians(){
        orientation_mutex.take(1);
        const double currentRadian = orientation.GetRadians();
        orientation_mutex.give();
        return currentRadian;
    }


    /// @brief Set current pose's angle in \b radians
    /// @param radians Input value to set the current angle to
    /// @warning Sets angles in units of \b radians. INPUT MUST BE IN \b RADIANS
    void Odometry::setRadians(double radians){
        orientation_mutex.take(1);
        orientation.SetRadians(radians);
        orientation_mutex.give();
    }


    /// @brief Get current position in the X-axis, Y-axis, and  angle in \b degrees
    Pose Odometry::getPose(){
        return Pose(getX(),getY(), getDegrees());
    }


    //MAIN FUNCTIONS


    /**
     * \brief Resets the Odometry values with `INITIAL_ODOMETRY_X`,Y and T constants.
     */
    void Odometry::resetInitial(){
        /*
        ATTENTION
        We need to know where the robot start (coordinates), for the odometry knows where the
        robot is at all times. Maybe using gps or a const variable
        */
        // Normal initial
        resetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
    }


    /**
     * \brief Initialization function to put everything to 0
     */
    void Odometry::initialize(){
        encoderLeft.set_position(0);
        encoderRight.set_position(0);
        encoderBack.set_position(0);


        encoderLeft.reset();
        encoderRight.reset();
        encoderBack.reset();


        // Set initial position with gps (need test with field)
        // INITIAL_ODOMETRY_X = gps.get_x_position();
        // INITIAL_ODOMETRY_Y = gps.get_y_position();
        resetInitial();
    }

    static double max = 0;
    static double maxRight = 0;
    static double maxLeft = 0;
    static double maxResta = 0;
    static double minResta = 100000;

    /**
     * @brief Fundamental function for Odometry, use encoders and gyro to calculate position and orientation.
     *
     * @details Uses changes in encoder (right and left) and gyro to calculate position
     * 
     * */
    void Odometry::update() {
        // Read encoder values, divided by 100 to convert centidegrees to degrees
        encoderRight_data.currentValue = encoderRight.get_position() / 100.0; 
        encoderLeft_data.currentValue = encoderLeft.get_position() / 100.0; 
        
        // Convert to distances
        encoderRight_data.currentDistance = encoderRight_data.currentValue * conversionFactor;
        encoderLeft_data.currentDistance = encoderLeft_data.currentValue * conversionFactor;
        
        // Calculate deltas
        encoderRight_data.delta = encoderRight_data.currentValue - encoderRight_data.prevValue;
        encoderLeft_data.delta = encoderLeft_data.currentValue - encoderLeft_data.prevValue;
        
        encoderRight_data.deltaDistance = encoderRight_data.currentDistance - encoderRight_data.previousDistance;
        encoderLeft_data.deltaDistance = encoderLeft_data.currentDistance - encoderLeft_data.previousDistance;
        
        // Take information from the back encoder
        #if USING_BIG_ROBOT
        encoderBack_data.currentValue = encoderBack.get_position() / 100.0;
        encoderBack_data.currentDistance = encoderBack_data.currentValue * conversionFactor;
        encoderBack_data.delta = encoderBack_data.currentValue - encoderBack_data.prevValue;
        encoderBack_data.deltaDistance = encoderBack_data.currentDistance - encoderBack_data.previousDistance;
        #endif

        // Calculate delta theta if we dont have gyro
        double deltaTheta = (encoderLeft_data.deltaDistance - encoderRight_data.deltaDistance) / (DISTANCE_RIGHT_TRACKING_WHEEL_CENTER + DISTANCE_LEFT_TRACKING_WHEEL_CENTER);

        // If we have gyro, get value and calculate delta
        #if GYRO_ENABLED 
        // Read gyro value
        gyro_data.currentDegrees = gyroscope.get_heading(); // CCW positive
        gyro_data.currentRadians = gyro_data.currentDegrees * (M_PI / 180);

        
        // Normalize angle to prevent overshoot when using turn function
        if (gyro_data.currentDegrees > 180) {
            gyro_data.currentDegrees -= 360;
        }
        else if (gyro_data.currentDegrees <= -180) {
            gyro_data.currentDegrees += 360;
        }

        // Calculate delta
        gyro_data.deltaDegrees = gyro_data.currentDegrees - gyro_data.prevDegrees;
        gyro_data.deltaRadians = gyro_data.deltaDegrees * (M_PI / 180.0);

        // std::cout << "Current distance | Prev value | Delta\n";
        // std::cout << encoderLeft_data.currentDistance << " | " << encoderLeft_data.previousDistance << " | " << encoderLeft_data.deltaDistance << "\n";   
        // std::cout << encoderRight_data.currentDistance << " | " << encoderRight_data.previousDistance << " | " << encoderRight_data.deltaDistance << "\n";   
        // std::cout << encoderBack_data.currentDistance << " | " << encoderBack_data.previousDistance << " | " << encoderBack_data.deltaDistance << "\n";   
        
        // Save current data for future calculations
        gyro_data.prevDegrees = gyro_data.currentDegrees;
        
        // Right now, confidence gyro 1.0, encoder confidence 0 (must sum 1) 
        deltaTheta = (1 - GYRO_CONFIDENCE) * deltaTheta + GYRO_CONFIDENCE * gyro_data.deltaRadians;
        #endif
        if (deltaTheta * (180/M_PI) > max) max = deltaTheta * (180/M_PI);
        pros::lcd::print(3, "max: %0.3f", max);

        if (std::abs(encoderRight_data.deltaDistance) > std::abs(maxRight)) maxRight = std::abs(encoderRight_data.deltaDistance);
        pros::lcd::print(4, "max Right: %0.3f", maxRight);

        if (std::abs(encoderLeft_data.deltaDistance) > std::abs(maxLeft)) maxLeft = std::abs(encoderLeft_data.deltaDistance);
        pros::lcd::print(5, "max Left: %0.3f", maxLeft);

        if (std::abs(encoderLeft_data.deltaDistance) - std::abs(encoderRight_data.deltaDistance) < minResta) 
            minResta = std::abs(encoderLeft_data.deltaDistance) - std::abs(encoderRight_data.deltaDistance);
        if (std::abs(encoderLeft_data.deltaDistance) - std::abs(encoderRight_data.deltaDistance) > maxResta) 
            maxResta = std::abs(encoderLeft_data.deltaDistance) - std::abs(encoderRight_data.deltaDistance);

        pros::lcd::print(6, "max resta: %0.3f", minResta);
        pros::lcd::print(7, "min resta: %0.3f", maxResta);
        

        // Updating angle
        double thetaMid = (getRadians() + deltaTheta) / 2; // Theta where change happen
        setRadians(getRadians() + deltaTheta);
        
        std::cout << encoderBack_data.deltaDistance
          << " vs "
          << deltaTheta * DISTANCE_BACK_TRACKING_WHEEL_CENTER
          << "\n";

        /* ------------------ no math ------------------ */
        #if USING_BIG_ROBOT
        double dx = ((encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / 2) - (OFFSET_TRACKING_WHEEL * deltaTheta);
        double dy = encoderBack_data.deltaDistance - (deltaTheta * DISTANCE_BACK_TRACKING_WHEEL_CENTER); // delete the contribution from the back encoder in the turning
        #else 
        changeMine.SetPosition(encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance / 2, 
                                0.0);
        #endif

        changeMine.SetPosition(changeMine.GetX() + (dx * cos(thetaMid) - dy * sin(thetaMid)), 
                               changeMine.GetY() + (dx * sin(thetaMid) + dy * cos(thetaMid)));
        
        pros::lcd::print(2, "no math: X: %0.3f | Y: %0.3f | H: %0.3f", changeMine.GetX(), changeMine.GetY(), changeMine.GetDegrees());

        // Calculations simple trigonometry
        // If we are rotating in the same place
        // if (std::abs(std::abs(encoderLeft_data.deltaDistance) - std::abs(encoderRight_data.deltaDistance)) > 0.05 && 
        //     encoderLeft_data.deltaDistance * encoderRight_data.deltaDistance <= 0) {
        //     deltaDlocal.SetPosition(0.0, 0.0);
        // }
        // Else if we are rotating in a arc
        if (std::abs(deltaTheta) > TURNING_THRESHOLD) {
            // Calculate change as an arc
            // Calculate the radius of rotation for each wheel
            double sign = (deltaTheta > 0) ? 1 : -1; 
            double radiusLeft  = (encoderLeft_data.deltaDistance / deltaTheta)  + sign * DISTANCE_LEFT_TRACKING_WHEEL_CENTER;
            double radiusRight = (encoderRight_data.deltaDistance / deltaTheta) - sign * DISTANCE_RIGHT_TRACKING_WHEEL_CENTER;
            double radiusBack = (encoderBack_data.deltaDistance / deltaTheta) - sign * DISTANCE_BACK_TRACKING_WHEEL_CENTER;

            std::cout << "Radius Right: " << radiusRight << ", Left: " << radiusLeft << ", Back: " << radiusBack << "\n";
            // Calculate radius
            double averageR = (radiusLeft + radiusRight + radiusBack) / 3;
            pros::lcd::print(6, "Radius: %0.3f", averageR);
                
            // Update position using trigonometry
            deltaDlocal.SetPosition(averageR * std::sin(thetaMid), averageR * (1 - std::cos(thetaMid))); 
        }
        // Else if the robot is moving straight forward or backward or sideways, average encoder values for distance    
        else {
            // std::cout << "Not turning\n";
            double deltaD = ((encoderLeft_data.deltaDistance + encoderRight_data.deltaDistance) / 2.0) - (OFFSET_TRACKING_WHEEL * deltaTheta); // movement in X axis
            std::cout << "Delta D: " << deltaD << "      supose to be close to 0\n";
            deltaDlocal.SetPosition(deltaD, 0);
            
            // If we have encoder back
            #if USING_BIG_ROBOT 
            double deltaY = encoderBack_data.deltaDistance - deltaTheta * DISTANCE_BACK_TRACKING_WHEEL_CENTER;
            deltaDlocal.SetPosition(deltaD, deltaY);
            #endif
        }

        // Updating global position using 2D matrix transformation (previous way to update to global coordinates)
        position.SetPosition(getX() + deltaDlocal.GetX() * std::cos(thetaMid) - deltaDlocal.GetY() * std::sin(thetaMid), 
                             getY() + deltaDlocal.GetX() * std::sin(thetaMid) + deltaDlocal.GetY() * std::cos(thetaMid));

        std::cout << "X: " << getX() << ", Y: " << getY() << ", T: " << getDegrees() << "\n";
        pros::lcd::print(1, "X: %0.3f | Y: %0.3f | H: %0.3f", getX(), getY(), getDegrees());

        // Save current values as previous for future updates
        encoderLeft_data.prevValue = encoderLeft_data.currentValue;
        encoderRight_data.prevValue = encoderRight_data.currentValue;

        encoderRight_data.previousDistance = encoderRight_data.currentDistance;
        encoderLeft_data.previousDistance = encoderLeft_data.currentDistance;
        #if USING_BIG_ROBOT
        encoderBack_data.previousDistance = encoderBack_data.currentDistance;
        #endif
    }


    /**
     * \brief resets Odometry values using the particular parameters
     *
     * \param x X position in \b inches
     * \param y Y position in \b inches
     * \param theta Angular position in \b degrees
     */
    void Odometry::resetCurrent(double x , double y, double theta){

        const double currentAngleRight = encoderRight.get_position() / 100.0;
        const double currentAngleLeft = encoderLeft.get_position() / 100.0;
        const double currentAngleBack = encoderBack.get_position() / 100.0;
        const double currentAngleGyro = gyroscope.get_heading();


        // Reset encoder's struct variables
        encoderRight_data = {currentAngleRight,                     // current position in degrees
                            currentAngleRight,                     // previous position in degrees
                            0,                                     // delta in degrees
                            currentAngleRight * conversionFactor,  // current position in inches 
                            currentAngleRight * conversionFactor,  // previous position in inches
                            0.0};                                  // delta in inches

        encoderLeft_data = {currentAngleLeft,                       // current position in degrees
                            currentAngleLeft,                       // previous position in degrees
                            0,                                      // delta in degrees
                            currentAngleLeft * conversionFactor,    // current position in inches 
                            currentAngleLeft * conversionFactor,    // previous position in inches
                            0.0};                                   // delta in inches

        encoderBack_data = {currentAngleBack,                       // current position in degrees
                            currentAngleBack,                       // previous position in degrees
                            0,                                      // delta in degrees
                            currentAngleBack * conversionFactor,    // current position in inches 
                            currentAngleBack * conversionFactor,    // previous position in inches
                            0.0};                                   // delta in inches

        gyro_data = {0,                                             // current value degrees
                    0,                                             // previous value degrees
                    0,                                             // current radians
                    0.0,                                           // delta degrees
                    0.0};                                          // delta radians

        // Preset odometry values
        deltaTheta = 0.0;
        deltaDlocal.SetPosition(0.0, 0.0);

        // Other odometry we could use, less calculations
        changeWeb.SetPosition(0.0, 0.0);
        changeMine.SetPosition(0.0, 0.0);
        changeEasy.SetPosition(0.0, 0.0);

        setDegrees(theta);
        position.SetPosition(x, y);
        #if GYRO_ENABLED
        gyroscope.tare();
        pros::delay(3000);
        #endif
    }


    /// @brief Returns position of the robot in the field
    /// @returns The GPS coordinates as a `Vector`
    Vector Odometry::gpsPosition(){
      pros::delay(2000);
        pros::c::gps_status_s_t status = gps.get_status();
        Vector current = Vector().SetPosition(status.x, status.y);

        return current;
    }

    //Threading

    /**
     * \brief Function for odometry thread
     */
    void Odometry::sense(){
        while(true){
            update();
            pros::delay(20);
        }
    }

    //testing 


    /**
     * \brief Simple debug function that prints odometry values
     *
     * \details Blocking function that helps check if there are any issues with
     * odometry
     *
     * \note Requires initialize pros::lcd and calling the odometry::Initialize
     *       function
     * */
    void Odometry::debug(){
        while (true) {
        // pros::lcd::print(1, "X: %0.3f", GetX());
        // pros::lcd::print(2, "Y: %0.3f", GetY());
        // pros::lcd::print(0, "X: %0.3f, Y: %0.3f", GetX(), GetY());
        pros::lcd::print(0, "Left : %0.3f, %0.3f, %0.3f", encoderLeft_data.currentDistance, encoderLeft_data.previousDistance, encoderLeft_data.deltaDistance);
        pros::lcd::print(1, "Right: %0.3f, %0.3f, %0.3f", encoderRight_data.currentDistance, encoderRight_data.previousDistance, encoderRight_data.deltaDistance);
        pros::lcd::print(2, "Back: %0.3f, %0.3f, %0.3f", encoderBack_data.currentDistance, encoderBack_data.previousDistance, encoderBack_data.deltaDistance);
        pros::lcd::print(3, "Heading: %0.3f", getDegrees());
        pros::lcd::print(4, "Delta X: %0.3f, Y: %0.3f\n", deltaDlocal.GetX(), deltaDlocal.GetY());
        pros::lcd::print(5, "X: %0.3f | Y: %0.3f", getX(), getY());
        // pros::lcd::print(5, "Web:        X: %0.3f | Y: %0.3f", changeWeb.GetX(), changeWeb.GetY()); 
        // pros::lcd::print(6, "No Math:    X: %0.3f | Y: %0.3f", changeMine.GetX(), changeMine.GetY()); 
        // pros::lcd::print(7, "Video:       X: %0.3f | Y: %0.3f", changeVideo.GetX(), changeVideo.GetY()); 

        update();
        pros::delay(20);
        }

    }

}

