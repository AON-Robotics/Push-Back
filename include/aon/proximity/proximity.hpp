#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"

namespace aon {

class Proximity {
 private:
  pros::ADIDigitalIn sensor;

 public:
  Proximity(char port) : sensor(port) {}

  /// @brief Checks whether the proximity sensor is currently detecting an object.
  /// @return `true` if an object is present in front of the sensor, `false` otherwise.
  bool isDetecting() const;

  /// @brief Detects a new object entering the sensor's range (edge-triggered).
  /// @return `true` only once when a new object is first detected, `false` otherwise.
  bool newObjectDetected() const;
};

}  // namespace aon
