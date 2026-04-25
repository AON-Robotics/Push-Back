#include "../../include/aon/proximity/proximity.hpp"

namespace aon {

bool Proximity::isDetecting() const { return this->sensor.get_value() == HIGH; }

bool Proximity::newObjectDetected() const { return this->sensor.get_new_press() == 1; }

}