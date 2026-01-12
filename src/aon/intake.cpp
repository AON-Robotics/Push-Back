#include "../include/aon/intake/intake.hpp"

namespace aon {

#if USING_BIG_ROBOT

Intake::Intake(const std::initializer_list<okapi::Motor>& allMotorPorts,
               const std::initializer_list<okapi::Motor>& frontElevatorPorts,
               const std::initializer_list<okapi::Motor>& hoarderPorts,
               const std::initializer_list<okapi::Motor>& backElevatorPorts,
               const std::initializer_list<okapi::Motor>& scorerPorts,
               const std::initializer_list<okapi::Motor>& shotbeltPorts,
               const std::initializer_list<okapi::Motor>& shooterPorts,
               char shrimpPistonsPort, int distanceSensorPort,
               int colorSensorPort)
    : intakeMG(allMotorPorts),
      frontElevatorMG(frontElevatorPorts),
      hoarderMG(hoarderPorts),
      backElevatorMG(backElevatorPorts),
      scorerMG(scorerPorts),
      shotbeltMG(shotbeltPorts),
      shooterMG(shooterPorts),
      shrimpPistons(shrimpPistonsPort),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort) {}

void Intake::configure(okapi::AbstractMotor::brakeMode brakeMode,
                       okapi::AbstractMotor::gearset gearset) {
  intakeMG.setBrakeMode(brakeMode);
  intakeMG.setGearing(gearset);
  intakeMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intakeMG.tarePosition();

  frontElevatorMG.setBrakeMode(brakeMode);
  frontElevatorMG.setGearing(gearset);
  frontElevatorMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  frontElevatorMG.tarePosition();

  hoarderMG.setBrakeMode(brakeMode);
  hoarderMG.setGearing(gearset);
  hoarderMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  hoarderMG.tarePosition();

  backElevatorMG.setBrakeMode(brakeMode);
  backElevatorMG.setGearing(gearset);
  backElevatorMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  backElevatorMG.tarePosition();

  scorerMG.setBrakeMode(brakeMode);
  scorerMG.setGearing(gearset);
  scorerMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  scorerMG.tarePosition();

  shotbeltMG.setBrakeMode(brakeMode);
  shotbeltMG.setGearing(okapi::AbstractMotor::gearset::blue);
  shotbeltMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  shotbeltMG.tarePosition();

  shooterMG.setBrakeMode(brakeMode);
  shooterMG.setGearing(gearset);
  shooterMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  shooterMG.tarePosition();
}

void Intake::frontElevator(const int& rpm) { frontElevatorMG.moveVelocity(rpm); }


void Intake::hoarder(const int& rpm) { hoarderMG.moveVelocity(rpm); }

void Intake::backElevator(const int& rpm) {
  backElevatorMG.moveVelocity(rpm);
}

void Intake::shotbelt(const int& rpm) { shotbeltMG.moveVelocity(rpm); }

void Intake::shooter(const int& rpm) { shooterMG.moveVelocity(rpm); }

void Intake::scan() {
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 2600;  // ms
  while (true) {
    if (scanning) {
      if (this->isObjectDetected()) {
        this->pickUp();
        stopTime = pros::millis() + DELAY_PER_BALL;
        pros::delay(500);  // this is to avoid counting the same block multiple times
      }
      if (pros::millis() >= stopTime) {
        this->frontElevator(0);
        this->backElevator(0);
        this->scorer(0);
        stopTime = INT_MAX;
      }
    }
    pros::delay(50);
  }
}

void Intake::sort() {
  size_t checkTime = 0;
  size_t actionTime = UINT32_MAX;
  size_t stopTime = UINT32_MAX;
  const short ACTION_DELAY = 5;        // ms
  const short CHECK_DELAY = 200;       // ms
  const short ACCEPTANCE_DELAY = 700;  // ms
  const short REJECTION_DELAY = 450;   // ms
  Action action = NONE;
  while (true) {
    if (scanning) {
      colorSensor.set_led_pwm(100); // For consistent illumination in the elevator
      const double hue = this->hue();
      const bool red = isRed(hue), blue = isBlue(hue);

      if (pros::millis() >= checkTime && (red || blue)) {
        // Schedule Corresponding Action
        if ((blue && BLUE_ALLIANCE) || (red && RED_ALLIANCE)) {
          action = ACCEPT;
          actionTime = pros::millis() + ACTION_DELAY;
        } else if ((red && BLUE_ALLIANCE) || (blue && RED_ALLIANCE)) {
          action = REJECT;
          actionTime = pros::millis() + 0;
        }
        checkTime = pros::millis() + CHECK_DELAY;
      }

      if (pros::millis() >= actionTime) {
        // Execute Scheduled Action
        if (action == ACCEPT) {
          this->hoarder();
          this->scorer(-INTAKE_VELOCITY);
          this->shotbelt();
          stopTime = pros::millis() + ACCEPTANCE_DELAY;
        } else if (action == REJECT) {
          this->hoarder(-INTAKE_VELOCITY);
          this->scorer(-INTAKE_VELOCITY);
          stopTime = pros::millis() + REJECTION_DELAY;
        }
        actionTime = UINT32_MAX;
      }

      if (pros::millis() >= stopTime) {
        // Stop Scheduled Action after the given delay
        this->hoarder(0);
        this->scorer(0);
        this->shotbelt(0);
        stopTime = UINT32_MAX;
      }
    }
    else {
      colorSensor.set_led_pwm(0);
    }
    pros::delay(25);
  }
}

void Intake::pickUp(const int& delay) {
  this->frontElevator();
  this->backElevator();
  this->scorer(-INTAKE_VELOCITY);
  if (delay == 0) return;
  pros::delay(delay);
  this->scorer(0);
  this->backElevator(0);
  this->frontElevator(0);
}

void Intake::store(const int& delay) {
  this->frontElevator();
  this->scorer(-INTAKE_VELOCITY);
  this->hoarder();
  this->backElevator();
  this->shotbelt();
  if (delay == 0) return;
  pros::delay(delay);
  this->frontElevator(0);
  this->scorer(0);
  this->hoarder(0);
  this->backElevator(0);
  this->shotbelt(0);
}

void Intake::hoard(const int& delay) {
  this->frontElevator();
  this->scorer(-INTAKE_VELOCITY);
  this->hoarder(-INTAKE_VELOCITY);
  this->backElevator();
  if (delay == 0) return;
  pros::delay(delay);
  this->frontElevator(0);
  this->scorer(0);
  this->hoarder(0);
  this->backElevator(0);
}

void Intake::score(const Height& to, const Height& from, const int& delay) {
  if (to == TOP) {
    this->move();
  } else if (to == BOTTOM) {
    this->frontElevator(-INTAKE_VELOCITY);
    this->scorer();
    this->hoarder(-INTAKE_VELOCITY);
    this->backElevator(-INTAKE_VELOCITY);
    this->shotbelt(-INTAKE_VELOCITY);
    this->shooter(-200);
  } else if (to == MIDDLE) {
    if (from == TOP) {
      this->frontElevator();
      this->scorer();
      this->hoarder(-INTAKE_VELOCITY);
      this->backElevator(-INTAKE_VELOCITY);
      this->shotbelt(-INTAKE_VELOCITY);
      this->shooter(-200);
    } else if (from == BOTTOM) {
      this->frontElevator();
      this->scorer();
      this->hoarder();
      this->backElevator();
    }
  }
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::dropShrimp() { shrimpPistons.set_value(HIGH); }

void Intake::raiseShrimp() { shrimpPistons.set_value(LOW); }

#else

Intake::Intake(const std::initializer_list<okapi::Motor>& allMotorPorts,
               const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               const std::initializer_list<okapi::Motor>& scorerPorts,
               char scorerPistonPort, char cartPistonPort, char trapdoorPistonPort,
               int distanceSensorPort, int colorSensorPort)
    : intakeMG(allMotorPorts),
      elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      scorerMG(scorerPorts),
      scorerPiston(scorerPistonPort),
      cartPiston(cartPistonPort),
      trapdoorPiston(trapdoorPistonPort),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort) {}

void Intake::configure(okapi::AbstractMotor::brakeMode brakeMode,
                       okapi::AbstractMotor::gearset gearset) {
  intakeMG.setBrakeMode(brakeMode);
  intakeMG.setGearing(gearset);
  intakeMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  intakeMG.tarePosition();

  elevatorMG.setBrakeMode(brakeMode);
  elevatorMG.setGearing(gearset);
  elevatorMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  elevatorMG.tarePosition();

  judgeMG.setBrakeMode(brakeMode);
  judgeMG.setGearing(gearset);
  judgeMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  judgeMG.tarePosition();

  scorerMG.setBrakeMode(brakeMode);
  scorerMG.setGearing(gearset);
  scorerMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  scorerMG.tarePosition();
}

void Intake::elevator(const int& rpm) { elevatorMG.moveVelocity(rpm); }

void Intake::judge(const int& rpm) { judgeMG.moveVelocity(rpm); }

void Intake::scan() {
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 2450;  // ms
  while (true) {
    if (scanning) {
      if (this->isObjectDetected()) {
        this->pickUp();
        stopTime = pros::millis() + DELAY_PER_BALL;
        pros::delay(
            500);  // this is to avoid counting the same block many times
      }

      if (pros::millis() >= stopTime) {
        this->elevator(0);
        stopTime = INT_MAX;
      }
    }

    pros::delay(50);
  }
}

void Intake::sort() {
  size_t checkTime = 0;
  size_t actionTime = UINT32_MAX;
  size_t stopTime = UINT32_MAX;
  const short ACTION_DELAY = 20;       // ms
  const short CHECK_DELAY = 100;       // ms
  const short ACCEPTANCE_DELAY = 800;  // ms
  const short REJECTION_DELAY = 400;   // ms
  Action action;
  while (true) {
    if (scanning) {
      colorSensor.set_led_pwm(100); // For consistent illumination in the elevator
      const double hue = this->hue();
      const bool red = isRed(hue), blue = isBlue(hue);

      if (pros::millis() >= checkTime && (red || blue)) {
        // Schedule Corresponding Action
        if ((blue && BLUE_ALLIANCE) || (red && RED_ALLIANCE)) {
          action = ACCEPT;
        } else if ((red && BLUE_ALLIANCE) || (blue && RED_ALLIANCE)) {
          action = REJECT;
        }
        actionTime = pros::millis() + ACTION_DELAY;
        checkTime = pros::millis() + CHECK_DELAY;
      }

      if (pros::millis() >= actionTime) {
        // Execute Scheduled Action
        if (action == ACCEPT) {
          this->judge();
          this->scorer();
          stopTime = pros::millis() + ACCEPTANCE_DELAY;
        } else if (action == REJECT) {
          this->judge(-INTAKE_VELOCITY);
          this->scorer();
          stopTime = pros::millis() + REJECTION_DELAY;
        }
        actionTime = UINT32_MAX;
      }

      if (pros::millis() >= stopTime) {
        // Stop Scheduled Action after the given delay
        this->judge(0);
        this->scorer(0);
        stopTime = UINT32_MAX;
      }
    }
    else {
      colorSensor.set_led_pwm(0);
    }
    pros::delay(25);
  }
}

void Intake::pickUp(const int& delay) {
  this->elevator();
  if (delay == 0) return;
  pros::delay(delay);
  this->elevator(0);
}

void Intake::store(const int& delay) {
  this->elevator();
  this->judge();
  if (delay == 0) return;
  pros::delay(delay);
  this->elevator(0);
  this->judge(0);
}

void Intake::reject(const int& delay) {
  this->elevator();
  this->judge(-INTAKE_VELOCITY);
  if (delay == 0) return;
  pros::delay(delay);
  this->elevator(0);
  this->judge(0);
}

void Intake::score(const Height& height, const int& delay) {
  if (height == TOP) {
    this->move();
  } else if (height == BOTTOM) {
    this->move(-INTAKE_VELOCITY);
  } else
    return;
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::setScorerHeight(const short& height) {
  scorerPiston.set_value(height);
}

void Intake::dropCart() { cartPiston.set_value(HIGH); }

void Intake::raiseCart() { cartPiston.set_value(LOW); }

void Intake::openTrapdoor() { trapdoorPiston.set_value(HIGH); }

void Intake::closeTrapdoor() { trapdoorPiston.set_value(LOW); }

#endif

void Intake::move(const int& rpm) { intakeMG.moveVelocity(rpm); }

void Intake::scorer(const int& rpm) { scorerMG.moveVelocity(rpm); }

void Intake::stop() { intakeMG.moveVelocity(0); }

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= DISTANCE; }

void Intake::activateScan() { scanning = true; }

void Intake::stopScan() { scanning = false; }

void Intake::kickBack() {
  this->move(-100);
  pros::delay(150);
  this->stop();
}

double Intake::hue() { return colorSensor.get_hue(); }

bool Intake::isRed(const double& hue) { return 0 <= hue && hue <= 35; }

bool Intake::isBlue(const double& hue) { return 185 <= hue && hue <= 215; }

}  // namespace aon
