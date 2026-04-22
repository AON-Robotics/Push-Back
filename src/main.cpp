#include "main.hpp"
#include "aon/globals.hpp"
#include "aon/autonomy/routine.hpp"
#include "aon/autonomy/goto_pose.hpp"
#include "../include/aon/intake/intake.hpp"

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Boot: calibrating IMU");

  imu.reset(true);
  while (imu.is_calibrating()) {
    pros::delay(10);
  }

  aon::Configure(true);
  sensorFeeder.applyEkfDefaults(ekf);
  sensorFeeder.reset(&ekf);
  drivetrainFeeder2.applyDriveDefaults(drivetrainTank2, hppCfg);

  pros::lcd::set_text(0, "Init done: goToPose test");
}

void autonomous() {
  aon::autonomy::setGoToPoseCsvDebugEnabled(true);

  aon::autonomy::Routine routine;
  // routine.goToDeg(46.5, 0.0, -90, 10000)
        //  .goToDeg(46.5, -12, -90, 1800)
        //  .goToDeg(46.5, 0, -90, 5000)
        //  .goToDeg(46.5, 0, -225, 5000)
        //  .goToDeg(22, 24, -225,10000);
         
        // .goToDeg(0, 0, 180,10000);

      // right loader (top) 
        routine
        .goToDeg(42, 0.0, 90, 4000)       // Go top between and middle of loader a long a and rotate left
        .goToDeg(42, 15, 90, 8000, [] { intake.pickUp(8000); })  // Approach loader a + start pickup simultaneously
        .goToDeg(42, -15, 90, 5000)       // Do not rotate, reverse to reach long a
        .doAction([] {intake.score(aon::Intake::TOP, 4000);})
        .goToDeg(42, 15, 90, 2500, [] { intake.pickUp(3000); }) 
        .goToDeg(42, 1.5, 90, 2500)      // Reverse
        .goToDeg(42, 0, 45, 3000)       // Heading to middle in reverse
        .goToDeg(5, -35, 45, 8500)    // Reached center
        .doAction([] {intake.score(aon::Intake::MIDDLE, 4000);})
        .goToDeg(18.5, -22, -90, 5000)        // Go front (we are facing the back to the middle )
        .goToDeg(18.5, -39.0, -90, 5000)  // Go to a coordinate near the long_a enemy side 
        .goToDeg(42, -115, -90, 8000)    // try middle of long a in enemy side
           
        ; // go to middle
  routine.run();

}

void opcontrol() {
autonomous();
}

void disabled() {}

void competition_initialize() {}

