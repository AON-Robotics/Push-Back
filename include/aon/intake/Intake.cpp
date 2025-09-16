#include "Intake.hpp"
#include "../constants.hpp"
#include "../globals.hpp"
#include "../okapi/api.hpp"

namespace aon{
//Singleton
Intake& Intake::instance() {
  static Intake s_instance;
  return s_instance;
}
//Construction
Intake::Intake()
  : intake(intake_)
  , rail(rail_)
  , gate(gate_) {
}

//High Stakes Intake Functions:
//Public API

void Intake::startScan() { intakeScanning = true; }
void Intake::stopScan()  { intakeScanning = false; }

// /**
//  * \brief This small subroutine moves the intake such that a ring is scored on the mobile goal being carried
//  *
//  * \param delay The time in \b milliseconds to leave the intake running
//  */
void Intake::pickUp(int delayMs) {
  intake.moveVelocity(INTAKE_VELOCITY / 0.8); // run a bit faster than our default
  pros::delay(delayMs);
  intake.moveVelocity(0);                     // stop after the delay
}

//  * \brief This small subroutine moves the rail such that a ring is scored on the mobile goal being carried
//  *
//  * \param delay The time in \b milliseconds to leave the intake running
//  */
void Intake::score(int delayMs) {
  rail.moveVelocity(INTAKE_VELOCITY);
  pros::delay(delayMs);
  rail.moveVelocity(0);
}

// /**
//  * \brief Discards disk at beginning of match
//  *
//  * \note This function is really meant for routines that will focus on enemy rings
//  */
void Intake::discard() {
  intake.moveVelocity(-INTAKE_VELOCITY);
  pros::delay(1000);
  intake.moveVelocity(0);
}

/// @brief Drops the gate from starting position so the robot can grab stuff
void Intake::openGate(int ms) {
  gate.moveVelocity(-100);
  pros::delay(ms);
  gate.moveVelocity(0);
}

void Intake::move(int velocity){
  intake.moveVelocity(velocity);
}

/// @brief Runs a background loop to auto-pick rings when scanning is active.
//For this function, the drivetrain logic can be accessed via a bool in autonomous so that
// So instead of using drivetrain logic here just use a bool to not have drivetrain logic here. 
void Intake::scan(){
  while(true){
    if (intakeScanning && distanceSensor.get() <= DISTANCE) {
      intakePickupDetected = true;
      pickUp();
      intake.moveVelocity(0);
    }
    pros::delay(20);
  }
}
}