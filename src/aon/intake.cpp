#include "../../include/aon/intake/intake.hpp"

namespace aon {

#if USING_BIG_ROBOT

Intake::Intake(const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               char cartPistonsPort, int distanceSensorPort,
               int colorSensorPort, 
               char acceptSensorPort, char rejectSensorPort)
    : elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      cart(cartPistonsPort, Piston::RETRACTED),
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
      // Only stop the elevator if sort isn't actively controlling it
      if (!releasing && pros::millis() >= stopTime) {
        this->elevator(0);
        stopTime = INT_MAX;
      }
    }
    pros::delay(50);
  }
}

void Intake::sort() {
  sortState = INIT;
  bool lastReleasing = false;
  bool armed = true;
  bool pendingCorrect = false;
  uint32_t timerEnd = 0;

  while (true) {
    const double hue = this->hue();
    const bool red = isRed(hue), blue = isBlue(hue);

    if (releasing) {
        if (!lastReleasing) {
          // Rising edge — start init reverse non-blocking
          this->score(Intake::BOTTOM);
          this->judge(-INTAKE_VELOCITY);
          timerEnd = pros::millis() + 350;
          sortState = INIT;
          armed = true;
          lastReleasing = true;
        }

        switch (sortState) {
        case INIT:
          // Wait for init reverse to finish, then go to IDLE
          if (pros::millis() >= timerEnd) {
            this->elevator(0);
            this->judge(0);
            sortState = IDLE;
          }
          break;

        case IDLE:
          this->elevator(INTAKE_VELOCITY * 2 / 3);
          if (armed && (red || blue)) {
            armed = false;
            pendingCorrect = (ALLIANCE == Alliance::Skills) ||
                             (red && ALLIANCE == Alliance::Red) || (blue && ALLIANCE == Alliance::Blue);
            const Height height = pendingCorrect ? acceptHeight : (acceptHeight == TOP ? MIDDLE : TOP);
            if (height != TOP) {
              // Kickback — reverse briefly before routing
              this->elevator(-INTAKE_VELOCITY);
              timerEnd = pros::millis() + 265;
              sortState = KICKBACK;
            } else {
              this->elevator(INTAKE_VELOCITY * 2 / 3);
              this->judge(INTAKE_VELOCITY);
              sortState = WAIT_ACCEPT;
            }
          }
          break;

        case KICKBACK:
          // Wait for kickback to finish, then spin judge
          if (pros::millis() >= timerEnd) {
            const Height height = pendingCorrect ? acceptHeight : (acceptHeight == TOP ? MIDDLE : TOP);
            this->elevator(INTAKE_VELOCITY * 2 / 3);
            this->judge(height == TOP ? INTAKE_VELOCITY : -INTAKE_VELOCITY);
            sortState = pendingCorrect ? WAIT_ACCEPT : WAIT_REJECT;
          }
          break;

        case WAIT_ACCEPT: {
          auto& sensor = (acceptHeight == TOP) ? acceptSensor : rejectSensor;
          if (sensor.isDetecting()) sortState = CONFIRM_ACCEPT;
          break;
        }

        case CONFIRM_ACCEPT: {
          auto& sensor = (acceptHeight == TOP) ? acceptSensor : rejectSensor;
          if (!sensor.isDetecting()) {
            this->judge(0);
            timerEnd = pros::millis() + 105;
            sortState = SETTLE;
          }
          break;
        }

        case WAIT_REJECT: {
          auto& sensor = (acceptHeight != TOP) ? acceptSensor : rejectSensor;
          if (sensor.isDetecting()) sortState = CONFIRM_REJECT;
          break;
        }

        case CONFIRM_REJECT: {
          auto& sensor = (acceptHeight != TOP) ? acceptSensor : rejectSensor;
          if (!sensor.isDetecting()) {
            this->judge(0);
            timerEnd = pros::millis() + 105;
            sortState = SETTLE;
          }
          break;
        }

        case SETTLE:
          // Brief pause after block clears before re-arming
          if (pros::millis() >= timerEnd) {
            sortState = IDLE;
            armed = true;
          }
          break;
        }
      } else {
        if (lastReleasing) {
          // Falling edge — stop motors and reset for next press
          this->elevator(0);
          this->judge(0);
          sortState = INIT;
          timerEnd = UINT32_MAX; // prevent stale timer from skipping init on next press
        }
        lastReleasing = false;
      }
    pros::delay(10);
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

void Intake::toggleCart() { cart.toggle(); }

void Intake::dropCart() { cart.activate(); }

void Intake::raiseCart() { cart.deactivate(); }

void Intake::activateScan() { scanning = true; }

void Intake::stopScan() { scanning = false; }

void Intake::setSortHeights(Height accept) { // TODO: rename
  acceptHeight = accept;
}

void Intake::startReleasing() {
  releasing = false;  // force falling edge so sort task fully resets
  pros::delay(10);    // give sort task one cycle to see the false
  releasing = true;
}

void Intake::stopReleasing() { releasing = false; }

Intake::SortState Intake::getSortingState() const { return sortState; }
  
#else

Intake::Intake(const std::initializer_list<okapi::Motor>& corridorPorts,
               const std::initializer_list<okapi::Motor>& elevatorPorts,
               const std::initializer_list<okapi::Motor>& judgePorts,
               const std::initializer_list<okapi::Motor>& scorerPorts,
               char scorerPistonPort, char cartPistonPort, char trapdoorPistonPort,
               int distanceSensorPort, int colorSensorPort)
    : corridorMG(corridorPorts),
      elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      scorerMG(scorerPorts),
      scorerPiston(scorerPistonPort, Piston::RETRACTED),
      cart(cartPistonPort, Piston::RETRACTED),
      trapdoor(trapdoorPistonPort, Piston::RETRACTED),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort) {
        this->leverController = okapi::AsyncPosControllerBuilder().withMotor(scorerPorts).build();
      }

void Intake::configure(okapi::AbstractMotor::brakeMode brakeMode, okapi::AbstractMotor::gearset gearset) {
  corridorMG.setBrakeMode(brakeMode);
  corridorMG.setGearing(gearset);
  corridorMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  corridorMG.tarePosition();

  elevatorMG.setBrakeMode(brakeMode);
  elevatorMG.setGearing(gearset);
  elevatorMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  elevatorMG.tarePosition();

  judgeMG.setBrakeMode(brakeMode);
  judgeMG.setGearing(gearset);
  judgeMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  judgeMG.tarePosition();

  scorerMG.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  scorerMG.setGearing(okapi::AbstractMotor::gearset::green);
  scorerMG.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  scorerMG.tarePosition();

  leverController->tarePosition();
}

void Intake::move(const int& rpm) {
  this->corridor(rpm);
  this->elevator(rpm);
  this->judge(rpm);
}

void Intake::corridor(const int& rpm) { corridorMG.moveVelocity(rpm); }

void Intake::elevator(const int& rpm) { elevatorMG.moveVelocity(rpm); }

void Intake::judge(const int& rpm) { judgeMG.moveVelocity(rpm); }

void Intake::scan() {
  colorSensor.set_led_pwm(50);
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
        this->corridor(0);
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
        if (ALLIANCE != Alliance::Skills && ((red && ALLIANCE == Alliance::Blue) || (blue && ALLIANCE == Alliance::Red))) {
          // Wrong alliance color detected — reverse motor to eject
          this->judge(-INTAKE_VELOCITY);
          pros::delay(100);
          this->judge(0);
        } else if ((red && ALLIANCE == Alliance::Red) || (blue && ALLIANCE == Alliance::Blue)) {
        // Correct alliance color detected — move motor to accept
          this->judge(INTAKE_VELOCITY);
          pros::delay(125);
          this->judge(0);
        }
      }
    }
    pros::delay(25);
  }
}

void Intake::pickUp(const int& delay) {
  this->corridor();
  this->elevator();
  if (delay == 0) return;
  pros::delay(delay);
  this->corridor(0);
  this->elevator(0);
}

void Intake::store(const int& delay) {
  this->move();
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::reject(const int& delay) {
  this->corridor();
  this->elevator();
  this->judge(-INTAKE_VELOCITY);
  if (delay == 0) return;
  pros::delay(delay);
  this->corridor(0);
  this->elevator(0);
  this->judge(0);
}

void Intake::lever(const uint32_t timeout) {
  this->extendLever();
  Timer timer(timeout);
  while(!this->leverFinished() && !timer.isCompleted()){ pros::delay(5); }
  this->resetLever();
}

void Intake::extendLever() {
  this->leverController->setTarget(150);
}

void Intake::resetLever() {
  this->leverController->setTarget(0);
}

bool Intake::leverFinished() {
  return this->leverController->getError() < 10;
}

void Intake::score(const Height& height, const int& delay) {
  if (height == TOP) {
    this->store();
  } else if (height == BOTTOM) {
    this->move(-INTAKE_VELOCITY);
  } else
    return;
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::toggleScorerHeight() { scorerPiston.toggle(); }

void Intake::raiseScorer() { scorerPiston.activate(); }

void Intake::lowerScorer() { scorerPiston.deactivate(); }

void Intake::toggleCart() { cart.toggle(); }

void Intake::dropCart() { cart.activate(); }

void Intake::raiseCart() { cart.deactivate(); }

void Intake::toggleTrapdoor() { trapdoor.toggle(); }

void Intake::openTrapdoor() { trapdoor.activate(); }

void Intake::closeTrapdoor() { trapdoor.deactivate(); }

void Intake::scorer(const int& rpm) {
  // TODO: modify logic for the lever
  scorerMG.moveVelocity(rpm);
}

void Intake::activateScan() {
  scanning = true;
}

void Intake::stopScan() {
  scanning = false;
  this->corridor(0);
  this->elevator(0);
}

#endif

void Intake::stop() { this->move(0); }

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= INTAKE_ACTIVATION_DISTANCE; }

bool Intake::isScanning() { return this->scanning; }

void Intake::kickBack() {
  this->move(-100);
  pros::delay(150);
  this->stop();
}

double Intake::hue() { return colorSensor.get_hue(); }

void Intake::setScoreDown(bool down) { scoreDown = down; }

bool Intake::isRed(const double& hue) { return (hue >= 356 && hue <= 359) || (hue >= 1 && hue <= 25); }
bool Intake::isBlue(const double& hue) { return 170 <= hue && hue <= 230; }

}  // namespace aon
