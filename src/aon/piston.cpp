#include "../../include/aon/piston/piston.hpp"

namespace aon {

Piston::State Piston::getState() const { return this->state; }

Piston::State Piston::set(State state) {
  this->solenoid.set_value(state == EXTENDED);
  this->state = state;
  return this->state;
}

void Piston::activate() { this->set(EXTENDED); }

void Piston::deactivate() { this->set(RETRACTED); }

bool Piston::isActivated() const { return this->state == EXTENDED; }

Piston::State Piston::toggle() {
  return this->set(this->isActivated() ? RETRACTED : EXTENDED);
}

}  // namespace aon
