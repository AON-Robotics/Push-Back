#include "../okapi/api.hpp"
#include "./constants.hpp"
#include "./controls/pid/pid.hpp"
#include "../tools/vector.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../sensing/odometry.hpp" 

class HDrive {
    private:
        // Motors 
        okapi::Motor middleMotor;

        // Controller
        pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);

        // Shared flags
        std::atomic_bool xDone = false;
        std::atomic_bool yDone = false;
        
        // Odometry

    public:
        // Constructor
        HDrive();

        // Helper functions 
        inline double AnalogInputScaling(const double x, const double t);

        // Movement functions
        void opcontrol();
        void drive2D();
        void move2D();
        
        // Control
};