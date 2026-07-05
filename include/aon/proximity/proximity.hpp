#pragma once

#include "pros/adi.hpp"

namespace aon {

class Proximity {
 private:
  pros::adi::DigitalIn sensor;

 public:
  Proximity(char port) : sensor(port) {}

  /**
   * @brief Reads the active-high proximity input.
   * @return `true` while an object drives the sensor input high.
   */
  bool isDetecting() const;

  /**
   * @brief Detects the rising edge of the proximity input.
   * @return `true` once when a new object enters the sensor range.
   *
   * Intake sorting uses this edge-triggered form to avoid queueing the same
   * object repeatedly while it remains in front of the sensor.
   */
  bool newObjectDetected() const;
};

}  // namespace aon
