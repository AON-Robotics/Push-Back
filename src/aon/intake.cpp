#include "../include/aon/intake/intake.hpp"

namespace aon {

#if USING_BIG_ROBOT

Intake::Intake(const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               char shrimpPistonsPort, int distanceSensorPort,
               int colorSensorPort)
    : elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      shrimpPistons(shrimpPistonsPort),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort),
      proximitySensor(proximitySensorPort),
      acceptSensor(acceptSensorPort) {}

void Intake::configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset) {

  elevatorMG.setBrakeMode(brakeMode);
  elevatorMG.setGearing(gearset);
  elevatorMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  elevatorMG.tarePosition();

  judgeMG.setBrakeMode(brakeMode);
  judgeMG.setGearing(gearset);
  judgeMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  judgeMG.tarePosition();
}

void Intake::move(const int& rpm) {
  this->elevator(rpm);
  this->judge(rpm);
}

void Intake::elevator(const int& rpm) { elevatorMG.moveVelocity(rpm); }


void Intake::judge(const int& rpm) { judgeMG.moveVelocity(rpm); }

void Intake::scan() {
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 1500;  // ms
  while (true) {
    if (scanning) {
      if (this->isObjectDetected()) {
        this->pickUp();
        stopTime = pros::millis() + DELAY_PER_BALL;
        pros::delay(500);  // this is to avoid counting the same block multiple times
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
  // NOTE: sensor values are inverted — HIGH means blocked, LOW means clear
  // LOW  = waiting (nothing blocking sensor)
  // HIGH = block is here (blocking sensor)
  // LOW  = block is gone (sensor cleared)
  enum SortState {
    IDLE,
    EJECT_WAITING,   // motors reversed, waiting for block to reach proximitySensor (sensor=LOW → HIGH when block arrives)
    EJECT_HERE,      // block at proximitySensor (HIGH), waiting for it to clear (LOW, debounced)
    ACCEPT_WAITING,  // motors forward, waiting for block to reach acceptSensor (sensor=LOW → HIGH when block arrives)
    ACCEPT_HERE,     // block at acceptSensor (HIGH), waiting for it to clear (LOW, debounced)
  };
  SortState sortState = IDLE;
  int clearedCount = 0;  // debounce counter shared between eject and accept paths
  const int CLEARED_THRESHOLD = 5;
  const uint32_t SENSOR_TIMEOUT_MS = 3000;
  uint32_t stateDeadline = 0;  // absolute time after which a stuck state resets to IDLE

  while (true) {
    if (scanning) {
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
          this->judge();
          stopTime = pros::millis() + ACCEPTANCE_DELAY;
        } else if (action == REJECT) {
          this->judge(-INTAKE_VELOCITY);
          stopTime = pros::millis() + REJECTION_DELAY;
        }
        actionTime = UINT32_MAX;
      }

      if (pros::millis() >= stopTime) {
        // Stop Scheduled Action after the given delay
        this->judge(0);
        stopTime = UINT32_MAX;
      }
    }
    pros::delay(5);
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
  if (delay == 0) return;
  pros::delay(delay);
  this->elevator(0);
}

void Intake::score(const Height& to, const int& delay) {
  if (to == TOP) {
    this->move();
  } else if (to == MIDDLE) {
    this->elevator();
    this->judge(-INTAKE_VELOCITY);
  } else if (to == BOTTOM) {
    this->elevator(-INTAKE_VELOCITY);
    this->judge(-INTAKE_VELOCITY);
  }
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::dropShrimp() { shrimpPistons.set_value(HIGH); }

void Intake::raiseShrimp() { shrimpPistons.set_value(LOW); }

#else

Intake::Intake(const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               const std::initializer_list<okapi::Motor>& scorerPorts,
               char scorerPistonPort, char cartPistonPort,
               int distanceSensorPort, int colorSensorPort,
               char proximitySensorPort, char acceptSensorPort)
    : elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      scorerMG(scorerPorts),
      scorerPiston(scorerPistonPort),
      cartPiston(cartPistonPort),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort),
      proximitySensor(proximitySensorPort),
      acceptSensor(acceptSensorPort) {}

void Intake::configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset) {
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

void Intake::move(const int& rpm) {
  this->elevator(rpm);
  this->judge(rpm);
  this->scorer(rpm);
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
        stopTime = UINT32_MAX;
      }
    }

    pros::delay(50);
  }
}

void Intake::sort() {
  while (true) {
    if (scanning) {
      const double hue = this->hue();
      const bool red = isRed(hue), blue = isBlue(hue);

      if (red || blue) {
        if ((red && BLUE_ALLIANCE) || (blue && RED_ALLIANCE)) {
          // Wrong alliance color detected — reverse motor to eject
          // NOTE: If the ejection motor is mounted inverted, negate the velocity below
          this->judge(-INTAKE_VELOCITY);

          // Wait for proximity sensor: LOW (clear) → HIGH (block arrives) → LOW (block cleared)
          // 3-second timeout in case the sensor never triggers
          const uint32_t SENSOR_TIMEOUT_MS = 3000;
          uint32_t deadline = pros::millis() + SENSOR_TIMEOUT_MS;
          while (scanning && proximitySensor.get_value() == HIGH && pros::millis() < deadline) pros::delay(5);
          deadline = pros::millis() + SENSOR_TIMEOUT_MS;
          while (scanning && proximitySensor.get_value() == LOW && pros::millis() < deadline) pros::delay(5);

          // Block confirmed removed (or scanning stopped / timed out) — stop rejection motor
          this->judge(0);
        } else if ((red && RED_ALLIANCE) || (blue && BLUE_ALLIANCE)) {
          const uint32_t SENSOR_TIMEOUT_MS = 3000;
          if (scoreDown) {
            // scoreDown enabled — send correct-color block down (reverse path,
            // tracked by proximitySensor same as ejection)
            this->judge(-INTAKE_VELOCITY);

            // NOTE: sensor inverted — HIGH = block present, LOW = clear
            uint32_t deadline = pros::millis() + SENSOR_TIMEOUT_MS;
            while (scanning && proximitySensor.get_value() == HIGH && pros::millis() < deadline) pros::delay(5);
            deadline = pros::millis() + SENSOR_TIMEOUT_MS;
            while (scanning && proximitySensor.get_value() == LOW && pros::millis() < deadline) pros::delay(5);

            this->judge(0);
          } else {
            // scoreDown disabled — send block forward, tracked by acceptSensor
            this->judge(INTAKE_VELOCITY);

            // NOTE: sensor inverted — HIGH = block present, LOW = clear
            uint32_t deadline = pros::millis() + SENSOR_TIMEOUT_MS;
            while (scanning && acceptSensor.get_value() == HIGH && pros::millis() < deadline) pros::delay(5);
            deadline = pros::millis() + SENSOR_TIMEOUT_MS;
            while (scanning && acceptSensor.get_value() == LOW && pros::millis() < deadline) pros::delay(5);

            this->judge(0);
          }
        }
      }
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

void Intake::scorer(const int& rpm) { scorerMG.moveVelocity(rpm); }

#endif

void Intake::stop() { this->move(0); }

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= INTAKE_ACTIVATION_DISTANCE; }

bool Intake::isScanning(){ return this->scanning; }

void Intake::activateScan() {
  scanning = true;
  colorSensor.set_led_pwm(100);
}

void Intake::stopScan() {
  scanning = false;
  colorSensor.set_led_pwm(0);
}

void Intake::kickBack() {
  this->move(-100);
  pros::delay(150);
  this->stop();
}

double Intake::hue() { return colorSensor.get_hue(); }

void Intake::setScoreDown(bool down) { scoreDown = down; }

bool Intake::isRed(const double& hue) { return 0 <= hue && hue <= 25; }

bool Intake::isBlue(const double& hue) { return 185 <= hue && hue <= 230; }

}  // namespace aon
