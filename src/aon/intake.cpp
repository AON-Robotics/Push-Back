#include "../../include/aon/intake/intake.hpp"

namespace aon {

#if USING_BIG_ROBOT

namespace {
constexpr std::uint32_t kSortLoopPeriodMs = 10;
constexpr std::uint32_t kSortAcknowledgeTimeoutMs = 250;

bool deadlineReached(std::uint32_t now, std::uint32_t deadline) {
  return static_cast<std::int32_t>(now - deadline) >= 0;
}
}

Intake::Intake(const std::initializer_list<std::int8_t>& elevatorPorts,
               const std::initializer_list<std::int8_t>& judgePorts,
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

void Intake::configure(pros::MotorBrake brakeMode, pros::MotorGears gearset) {
  elevatorMG.set_brake_mode_all(brakeMode);
  elevatorMG.set_gearing_all(gearset);
  elevatorMG.set_encoder_units_all(pros::MotorUnits::degrees);
  elevatorMG.tare_position_all();

  judgeMG.set_brake_mode_all(brakeMode);
  judgeMG.set_gearing_all(gearset);
  judgeMG.set_encoder_units_all(pros::MotorUnits::degrees);
  judgeMG.tare_position_all();
}

void Intake::move(const int& rpm) {
  this->elevator(rpm);
  this->judge(rpm);
}

void Intake::elevator(const int& rpm) { elevatorMG.move_velocity(rpm); }

void Intake::judge(const int& rpm) { judgeMG.move_velocity(rpm); }

bool Intake::commandSortMotors(std::uint32_t request, int elevatorRpm,
                               int judgeRpm) {
  sortMotorMutex.take();
  const bool ownsMotors = !sortFaulted.load(std::memory_order_acquire) &&
      releaseRequest.load(std::memory_order_acquire) == request;
  if (ownsMotors) {
    elevatorMG.move_velocity(elevatorRpm);
    judgeMG.move_velocity(judgeRpm);
    if (sortFaulted.load(std::memory_order_acquire) ||
        releaseRequest.load(std::memory_order_acquire) != request) {
      elevatorMG.move_velocity(0);
      judgeMG.move_velocity(0);
      sortMotorMutex.give();
      return false;
    }
  }
  sortMotorMutex.give();
  return ownsMotors;
}

void Intake::faultAndStopSortMotors() {
  sortFaulted.store(true, std::memory_order_release);
  if (!sortMotorMutex.take(kSortLoopPeriodMs)) {
    elevatorMG.move_velocity(0);
    judgeMG.move_velocity(0);
    return;
  }
  elevatorMG.move_velocity(0);
  judgeMG.move_velocity(0);
  sortMotorMutex.give();
}

void Intake::scan() {
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 2200;  // ms
  while (true) {
    if (scanning.load(std::memory_order_acquire)) {
      if (this->isObjectDetected()) {
        this->pickUp();
        stopTime = pros::millis() + DELAY_PER_BALL;
        pros::delay(500);  // this is to avoid counting the same block multiple times
      }
      // Only stop the elevator if sort isn't actively controlling it
      const bool releasing = intake_sync::releaseRequestActive(
          releaseRequest.load(std::memory_order_acquire));
      if (!releasing && stopTime != UINT32_MAX &&
          deadlineReached(pros::millis(), stopTime)) {
        this->elevator(0);
        stopTime = UINT32_MAX;
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
    const std::uint32_t requestedRequest =
        releaseRequest.load(std::memory_order_acquire);
    const bool requestedReleasing =
        intake_sync::releaseRequestActive(requestedRequest);
    const Height requestedAcceptHeight =
        acceptHeight.load(std::memory_order_acquire);
    const double hue = this->hue();
    const bool red = isRed(hue), blue = isBlue(hue);

    if (requestedReleasing) {
        if (!lastReleasing) {
          // Rising edge — start init reverse non-blocking
          commandSortMotors(requestedRequest, -INTAKE_VELOCITY,
                            -INTAKE_VELOCITY);
          timerEnd = pros::millis() + 350;
          sortState = INIT;
          armed = true;
          lastReleasing = true;
        }

        switch (sortState) {
        case INIT:
          // Wait for init reverse to finish, then go to IDLE
          if (deadlineReached(pros::millis(), timerEnd)) {
            commandSortMotors(requestedRequest, 0, 0);
            sortState = IDLE;
          }
          break;

        case IDLE:
          commandSortMotors(requestedRequest, INTAKE_VELOCITY * 2 / 3, 0);
          if (armed && (red || blue)) {
            armed = false;
            const Alliance alliance =
                ALLIANCE.load(std::memory_order_acquire);
            pendingCorrect = (alliance == Alliance::Skills) ||
                             (red && alliance == Alliance::Red) ||
                             (blue && alliance == Alliance::Blue);
            const Height height = pendingCorrect
                                      ? requestedAcceptHeight
                                      : (requestedAcceptHeight == TOP ? MIDDLE
                                                                      : TOP);
            if (height != TOP) {
              // Kickback — reverse briefly before routing
              commandSortMotors(requestedRequest, -INTAKE_VELOCITY, 0);
              timerEnd = pros::millis() + 265;
              sortState = KICKBACK;
            } else {
              commandSortMotors(requestedRequest,
                                INTAKE_VELOCITY * 2 / 3, INTAKE_VELOCITY);
              sortState = WAIT_ACCEPT;
            }
          }
          break;

        case KICKBACK:
          // Wait for kickback to finish, then spin judge
          if (deadlineReached(pros::millis(), timerEnd)) {
            const Height height = pendingCorrect
                                      ? requestedAcceptHeight
                                      : (requestedAcceptHeight == TOP ? MIDDLE
                                                                      : TOP);
            commandSortMotors(requestedRequest, INTAKE_VELOCITY * 2 / 3,
                              height == TOP ? INTAKE_VELOCITY
                                            : -INTAKE_VELOCITY);
            sortState = pendingCorrect ? WAIT_ACCEPT : WAIT_REJECT;
          }
          break;

        case WAIT_ACCEPT: {
          auto& sensor =
              (requestedAcceptHeight == TOP) ? acceptSensor : rejectSensor;
          if (sensor.isDetecting()) sortState = CONFIRM_ACCEPT;
          break;
        }

        case CONFIRM_ACCEPT: {
          auto& sensor =
              (requestedAcceptHeight == TOP) ? acceptSensor : rejectSensor;
          if (!sensor.isDetecting()) {
            commandSortMotors(requestedRequest,
                              INTAKE_VELOCITY * 2 / 3, 0);
            timerEnd = pros::millis() + 105;
            sortState = SETTLE;
          }
          break;
        }

        case WAIT_REJECT: {
          auto& sensor =
              (requestedAcceptHeight != TOP) ? acceptSensor : rejectSensor;
          if (sensor.isDetecting()) sortState = CONFIRM_REJECT;
          break;
        }

        case CONFIRM_REJECT: {
          auto& sensor =
              (requestedAcceptHeight != TOP) ? acceptSensor : rejectSensor;
          if (!sensor.isDetecting()) {
            commandSortMotors(requestedRequest,
                              INTAKE_VELOCITY * 2 / 3, 0);
            timerEnd = pros::millis() + 105;
            sortState = SETTLE;
          }
          break;
        }

        case SETTLE:
          // Brief pause after block clears before re-arming
          if (deadlineReached(pros::millis(), timerEnd)) {
            sortState = IDLE;
            armed = true;
          }
          break;
        }
      } else {
        if (lastReleasing) {
          // Falling edge — stop motors and reset for next press
          commandSortMotors(requestedRequest, 0, 0);
          sortState = INIT;
          timerEnd = UINT32_MAX; // prevent stale timer from skipping init on next press
        }
        lastReleasing = false;
      }
    // Publish completion only after the falling edge has stopped both motors.
    processedReleaseRequest.store(requestedRequest, std::memory_order_release);
    pros::delay(kSortLoopPeriodMs);
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

bool Intake::isCartDropped() const { return cart.isActivated(); }

void Intake::activateScan() {
  scanning.store(true, std::memory_order_release);
}

void Intake::stopScan() {
  scanning.store(false, std::memory_order_release);
}

std::uint32_t Intake::requestReleasing(bool active) {
  std::uint32_t current = releaseRequest.load(std::memory_order_relaxed);
  std::uint32_t next = 0;
  do {
    next = intake_sync::nextReleaseRequest(current, active);
  } while (!releaseRequest.compare_exchange_weak(
      current, next, std::memory_order_release, std::memory_order_relaxed));
  return next;
}

bool Intake::startReleasing(Height accept) {
  if (!stopReleasingAndWait()) return false;
  acceptHeight.store(accept, std::memory_order_release);
  sortMotorMutex.take();
  sortFaulted.store(false, std::memory_order_release);
  requestReleasing(true);
  sortMotorMutex.give();
  return true;
}

void Intake::stopReleasing() {
  requestReleasing(false);
}

bool Intake::stopReleasingAndWait() {
  const std::uint32_t targetRequest = requestReleasing(false);
  const std::uint32_t startedAt = pros::millis();
  while (processedReleaseRequest.load(std::memory_order_acquire) !=
         targetRequest) {
    if (pros::millis() - startedAt >= kSortAcknowledgeTimeoutMs) {
      faultAndStopSortMotors();
      return false;
    }
    pros::delay(kSortLoopPeriodMs);
  }
  return true;
}

Intake::SortState Intake::getSortingState() const {
  return sortState.load(std::memory_order_acquire);
}
  
#else

Intake::Intake(const std::initializer_list<std::int8_t>& corridorPorts,
               const std::initializer_list<std::int8_t>& elevatorPorts,
               const std::initializer_list<std::int8_t>& judgePorts,
               const std::initializer_list<std::int8_t>& scorerPorts,
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
      colorSensor(colorSensorPort) {}

void Intake::configure(pros::MotorBrake brakeMode, pros::MotorGears gearset) {
  corridorMG.set_brake_mode_all(brakeMode);
  corridorMG.set_gearing_all(gearset);
  corridorMG.set_encoder_units_all(pros::MotorUnits::degrees);
  corridorMG.tare_position_all();

  elevatorMG.set_brake_mode_all(brakeMode);
  elevatorMG.set_gearing_all(gearset);
  elevatorMG.set_encoder_units_all(pros::MotorUnits::degrees);
  elevatorMG.tare_position_all();

  judgeMG.set_brake_mode_all(brakeMode);
  judgeMG.set_gearing_all(gearset);
  judgeMG.set_encoder_units_all(pros::MotorUnits::degrees);
  judgeMG.tare_position_all();

  scorerMG.set_brake_mode_all(pros::MotorBrake::hold);
  scorerMG.set_gearing_all(pros::MotorGears::green);
  scorerMG.set_encoder_units_all(pros::MotorUnits::degrees);
  scorerMG.tare_position_all();
  leverTarget = 0.0;
}

void Intake::move(const int& rpm) {
  this->corridor(rpm);
  this->elevator(rpm);
  this->judge(rpm);
}

void Intake::corridor(const int& rpm) { corridorMG.move_velocity(rpm); }

void Intake::elevator(const int& rpm) { elevatorMG.move_velocity(rpm); }

void Intake::judge(const int& rpm) { judgeMG.move_velocity(rpm); }

void Intake::scan() {
  colorSensor.set_led_pwm(50);
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 2450;  // ms
  while (true) {
    if (scanning.load(std::memory_order_acquire)) {
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
    if (scanning.load(std::memory_order_acquire)) {
      const double hue = this->hue();
      const bool red = isRed(hue), blue = isBlue(hue);
      const Alliance alliance = ALLIANCE.load(std::memory_order_acquire);

      if (red || blue) {
        if (alliance != Alliance::Skills &&
            ((red && alliance == Alliance::Blue) ||
             (blue && alliance == Alliance::Red))) {
          // Wrong alliance color detected — reverse motor to eject
          this->judge(-INTAKE_VELOCITY);
          pros::delay(100);
          this->judge(0);
        } else if ((red && alliance == Alliance::Red) ||
                   (blue && alliance == Alliance::Blue)) {
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
  leverTarget = 150.0;
  scorerMG.move_absolute(leverTarget, 200);
}

void Intake::resetLever() {
  leverTarget = 0.0;
  scorerMG.move_absolute(leverTarget, 200);
}

bool Intake::leverFinished() {
  return std::abs(leverTarget - scorerMG.get_position()) < 10.0;
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

bool Intake::isScorerRaised() const { return scorerPiston.isActivated(); }

void Intake::toggleCart() { cart.toggle(); }

void Intake::dropCart() { cart.activate(); }

void Intake::raiseCart() { cart.deactivate(); }

bool Intake::isCartDropped() const { return cart.isActivated(); }

void Intake::toggleTrapdoor() { trapdoor.toggle(); }

void Intake::openTrapdoor() { trapdoor.activate(); }

void Intake::closeTrapdoor() { trapdoor.deactivate(); }

bool Intake::isTrapdoorOpen() const { return trapdoor.isActivated(); }

void Intake::scorer(const int& rpm) {
  // Manual scorer commands override position control so drivers can recover
  // the mechanism if the lever binds.
  scorerMG.move_velocity(rpm);
}

void Intake::activateScan() {
  scanning.store(true, std::memory_order_release);
}

void Intake::stopScan() {
  scanning.store(false, std::memory_order_release);
  this->corridor(0);
  this->elevator(0);
}

#endif

void Intake::stop() { this->move(0); }

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= INTAKE_ACTIVATION_DISTANCE; }

bool Intake::isScanning() {
  return scanning.load(std::memory_order_acquire);
}

void Intake::kickBack() {
  this->move(-100);
  pros::delay(150);
  this->stop();
}

double Intake::hue() { return colorSensor.get_hue(); }

void Intake::setScoreDown(bool down) {
  scoreDown.store(down, std::memory_order_release);
}

bool Intake::isRed(const double& hue) { return (hue >= 356 && hue <= 359) || (hue >= 1 && hue <= 25); }
bool Intake::isBlue(const double& hue) { return 170 <= hue && hue <= 230; }

}  // namespace aon
