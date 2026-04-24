#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"

namespace aon {

class Proximity {
 private:
  pros::ADIDigitalIn sensor;

 public:
  Proximity(char port) : sensor(port) {}

  bool isDetecting() const;

  bool newObjectDetected() const;
};

}  // namespace aon
