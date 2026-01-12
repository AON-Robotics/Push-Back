#pragma once

#include "../constants.hpp"
#include "../../api.h"
#include "../../okapi/api.hpp"
#include "../tools/general.hpp"

namespace aon {

class Intake {
 public:
  enum Action {
    ACCEPT,
    REJECT,
  };

  enum Height {
    TOP,
    MIDDLE,
    BOTTOM,
  };

#if USING_BIG_ROBOT
 private:
  okapi::MotorGroup intakeMG;
  okapi::MotorGroup frontElevatorMG;
  okapi::MotorGroup hoarderMG;
  okapi::MotorGroup backElevatorMG;
  okapi::MotorGroup scorerMG;
  okapi::MotorGroup shotbeltMG;
  okapi::MotorGroup shooterMG;
  pros::ADIDigitalOut shrimpPistons;
  pros::Distance distanceSensor;
  pros::Optical colorSensor;

  volatile bool scanning = false;

 public:
  Intake(const std::initializer_list<okapi::Motor>& allMotorPorts,
         const std::initializer_list<okapi::Motor>& frontElevatorPorts,
         const std::initializer_list<okapi::Motor>& hoarderPorts,
         const std::initializer_list<okapi::Motor>& backElevatorPorts,
         const std::initializer_list<okapi::Motor>& scorerPorts,
         const std::initializer_list<okapi::Motor>& shotbeltPorts,
         const std::initializer_list<okapi::Motor>& shooterPorts,
         char shrimpPistonsPort, int distanceSensorPort, int colorSensorPort);

  /// @brief Moves only the frontElevator at the given `rpm`
  /// @param rpm The rpm at which to set the frontElevator
  void frontElevator(const int& rpm = INTAKE_VELOCITY);

  /// @brief Moves only the backElevator at the given `rpm`
  /// @param rpm The rpm at which to set the backElevator
  void backElevator(const int& rpm = INTAKE_VELOCITY);

  /// @brief Moves only the hoarder at the given `rpm`
  /// @param rpm The rpm at which to set the hoarder
  void hoarder(const int& rpm = INTAKE_VELOCITY);

  /// @brief Moves only the shotbelt at the given `rpm`
  /// @param rpm The rpm at which to set the shotbelt
  void shotbelt(const int& rpm = INTAKE_VELOCITY);

  /// @brief Moves only the shooter at the given `rpm`
  /// @param rpm The rpm at which to set the shooter
  void shooter(const int& rpm = 200);

  /// @brief Discards blocks through the back of the robot
  void hoard(const int& delay = 0);

  /// @brief Drops the shrimp by activating its pistons
  void dropShrimp();

  /// @brief Raises the shrimp by deactivating its pistons
  void raiseShrimp();

  /// @brief This small subroutine moves the intake such that a block is scored
  /// on a goal.
  /// @param to The location out of which we will score the balls (HIGHER,
  /// MIDDLE, LOWER)
  /// @param from The location of the balls to be scored (HIGHER, LOWER)
  /// @param delay The time in \b milliseconds to leave the scorer running.
  /// @note A delay of 0 will never stop moving the intake.
  void score(const Height& to = TOP, const Height& from = BOTTOM,
             const int& delay = 0);

#else
 private:
  okapi::MotorGroup intakeMG;
  okapi::MotorGroup elevatorMG;
  okapi::MotorGroup judgeMG;
  okapi::MotorGroup scorerMG;
  pros::ADIDigitalOut scorerPiston;
  pros::ADIDigitalOut cartPiston;
  pros::ADIDigitalOut trapdoorPiston;
  pros::Distance distanceSensor;
  pros::Optical colorSensor;

  volatile bool scanning = true;

 public:
  Intake(const std::initializer_list<okapi::Motor>& allMotorPorts,
         const std::initializer_list<okapi::Motor>& elevatorPorts,
         const std::initializer_list<okapi::Motor>& judgePorts,
         const std::initializer_list<okapi::Motor>& scorerPorts,
         char scorerPistonPort, char cartPistonPort, char trapdoorPistonPort,
         int distanceSensorPort, int colorSensorPort);

  /// @brief Moves only the elevator at the given `rpm`
  /// @param rpm The rpm at which to set the elevator
  void elevator(const int& rpm = INTAKE_VELOCITY);

  /// @brief Moves only the judge at the given `rpm`
  /// @param rpm The rpm at which to set the judge
  void judge(const int& rpm = INTAKE_VELOCITY);

  /// @brief This small subroutine moves the intake such that a block is scored
  /// on a goal.
  /// @param height Whether to score out of the top or bottom of the robot.
  /// @param delay The time in \b milliseconds to leave the scorer running.
  /// @note A delay of 0 will never stop moving the intake.
  void score(const Height& height = TOP, const int& delay = 0);

  /// @brief Discards blocks through the back of the robot
  void reject(const int& delay = 0);

  /// @brief Moves the piston of the scorer to set its desired state (`HIGH` or
  /// `LOW` only)
  /// @param height The next height of the scorer in `{HIGH, LOW}`
  void setScorerHeight(const short& height);

  /// @brief Drops the cart by activating its pistons
  void dropCart();

  /// @brief Raises the cart by deactivating its pistons
  void raiseCart();

  /// @brief Opens the trapdoor by activating its pistons
  void openTrapdoor();

  /// @brief Closes the trapdoor by deactivating its pistons
  void closeTrapdoor();

#endif

  /// @brief Configures the subsytems of the intake
  /// @param brakeModeThe braking paradigm we will use, usually `coast`
  /// @param gearset The gearbox the physical motors contain
  void configure(okapi::AbstractMotor::brakeMode brakeMode,
                 okapi::AbstractMotor::gearset gearset);

  /// @brief Moves the entire intake system at the same `rpm`
  /// @param rpm The rpm to set to the motors
  void move(const int& rpm = INTAKE_VELOCITY);

  /// @brief Moves only the scorer at the given `rpm`
  /// @param rpm The rpm at which to set the scorer
  void scorer(const int& rpm = INTAKE_VELOCITY);

  /// @brief Stops all motors
  void stop();

  /// @brief Gets the distance from the distance sensor
  /// @return The distance from the sensor to whatever it is detecting
  double distance();

  /// @brief Getter for internal boolean
  /// @return Whether or not there is a donut as determined by the distance
  /// sensor
  bool isObjectDetected();

  /// @brief Runs a background loop to auto-pick-up blocks when scanning is
  /// active.
  /// @details For this function, the drivetrain logic can be accessed via a
  /// bool in autonomous so that instead of using drivetrain logic here just
  /// use a bool to not have drivetrain logic here.
  void scan();

  /// @brief Runs a background loop to color sort blocks when scanning is
  /// active.
  void sort();

  /// @brief Sets the flag for the scanning async task to start/resume runnning
  void activateScan();

  /// @brief Sets the flag for the scanning async task to stop runnning
  void stopScan();

  /// @brief This small subroutine moves the elevator such that a block is
  /// picked up
  /// @param delay The time in \b milliseconds to leave the elevator running
  void pickUp(const int& delay = 0);

  /// @brief This small subroutine moves the intake such that a
  /// block is stored for scoring.
  /// @param delay The time in \b milliseconds to leave the intake running.
  /// @note A delay of 0 will never stop moving the intake.
  void store(const int& delay = 0);

  /// @brief Makes the intake go slightly back to get it unstuck
  void kickBack();

  /// @brief Gets the hue from the color sensor
  /// @return The color of whatever is in front of the sensor
  double hue();

  /// @brief Determines whether the given `hue` corresponds to the color red
  /// @param hue The hue to be judged
  /// @return True if the hue corresponds to red, false otherwise
  static bool isRed(const double& hue);

  /// @brief Determines whether the given `hue` corresponds to the color blue
  /// @param hue The hue to be judged
  /// @return True if the hue corresponds to blue, false otherwise
  static bool isBlue(const double& hue);
};

}  // namespace aon
