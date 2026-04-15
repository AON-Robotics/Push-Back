#include "../include/aon/intake/intake.hpp"

namespace aon {

#if USING_BIG_ROBOT

Intake::Intake(const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               char cartPistonsPort, int distanceSensorPort,
               int colorSensorPort, 
               char acceptSensorPort, char rejectSensorPort)
    : elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      cartPistons(cartPistonsPort),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort),
      acceptSensor(acceptSensorPort),
      rejectSensor(rejectSensorPort) {}

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
  const short DELAY_PER_BALL = 2200;  // ms
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
  enum State { IDLE, WAITING_ARRIVAL, WAITING_PASS };
  std::queue<bool> colorQueue;
  State state = IDLE;
  bool currentCorrect = false;
  pros::ADIDigitalIn* currentSensor = nullptr;

  // Distance sensor block detection (32–37mm range)
  bool lastInRange = false;
  int distCount = 0;

  // Color sensor detection
  bool lastColorSeen = false;
  int colorCount = 0;
  bool lastColorCorrect = false; // last color the sensor actually identified

  while (true) {
    if (scanning) {
      // --- Distance sensor: count blocks passing through 32–37mm window ---
      const double dist = this->distance();
      const bool inRange = (dist >= 32 && dist <= 37);
      if (inRange && !lastInRange) {
        distCount++;
      }
      lastInRange = inRange;

      // --- Color sensor: count detected blocks and record color ---
      const double hue = this->hue();
      const bool red = isRed(hue), blue = isBlue(hue);
      const bool colorSeen = red || blue;
      if (colorSeen && !lastColorSeen) {
        colorCount++;
        lastColorCorrect = (red && RED_ALLIANCE) || (blue && BLUE_ALLIANCE);
        colorQueue.push(lastColorCorrect);
      }
      lastColorSeen = colorSeen;

      // --- Reconciliation: distance saw a block the color sensor missed ---
      if (distCount > colorCount) {
        // Unknown color — eject (rejectHeight)
        colorQueue.push(false);
        colorCount++; // balance the counts
      }

      // --- State machine: only runs after release() is called ---
      if (releasing) {
        switch (state) {
        case IDLE:
          if (!colorQueue.empty()) {
            currentCorrect = colorQueue.front();
            colorQueue.pop();
            const Height height = currentCorrect ? this->acceptHeight : this->rejectHeight;
            currentSensor = (height == TOP) ? &acceptSensor : &rejectSensor;
            ejecting = true;
            this->judge(height == TOP ? INTAKE_VELOCITY : -INTAKE_VELOCITY);
            state = WAITING_ARRIVAL;
          }
          break;

        case WAITING_ARRIVAL:
          if (currentSensor->get_value() == HIGH) {
            state = WAITING_PASS;
          }
          break;

        case WAITING_PASS:
          if (currentSensor->get_value() == LOW) {
            this->judge(0);
            ejecting = false;
            currentSensor = nullptr;
            state = IDLE;
          }
          break;
        }
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

void Intake::dropCart() { cartPistons.set_value(HIGH); }

void Intake::raiseCart() { cartPistons.set_value(LOW); }

void Intake::setSortHeights(Height accept, Height reject) {
  acceptHeight = accept;
  rejectHeight = reject;
}

void Intake::release() { releasing = true; }

#else

Intake::Intake(const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               const std::initializer_list<okapi::Motor>& scorerPorts,
               char scorerPistonPort, char cartPistonPort,
               int distanceSensorPort, int colorSensorPort)
    : elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      scorerMG(scorerPorts),
      scorerPiston(scorerPistonPort),
      cartPiston(cartPistonPort),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort){}
       

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

  leverController->tarePosition();
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
          this->judge(-INTAKE_VELOCITY);
          pros::delay(500);
          this->judge(0);
        } else if ((red && RED_ALLIANCE) || (blue && BLUE_ALLIANCE)) {
          if (scoreDown) {
            this->judge(-INTAKE_VELOCITY);
            pros::delay(500);
            this->judge(0);
          } else {
            this->judge(INTAKE_VELOCITY);
            pros::delay(500);
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

void Intake::scorer(const int& rpm) {
  // TODO: modify logic for the lever
  scorerMG.moveVelocity(rpm);
}

#endif

void Intake::stop() { this->move(0); }

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= INTAKE_ACTIVATION_DISTANCE; }

bool Intake::isScanning(){ return this->scanning; }

void Intake::activateScan() {
  scanning = true;
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
