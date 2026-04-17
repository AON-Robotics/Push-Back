#pragma once

#include "../../api.h"
#include "../../okapi/api.hpp"

namespace aon {

class Piston {
 public:
  enum State {
    RETRACTED,
    EXTENDED,
  };

 private:
  pros::ADIDigitalOut solenoid;
  State state;

  /// @brief Sets the value of the internal `solenoid` and the state of the
  /// piston to the passed `state`
  /// @param state The new state for the `solenoid` and piston
  /// @return The new `state`
  State set(State state);

 public:
  Piston(char port, State state) : solenoid(port), state(state) {
    this->set(state);
  }

  /// @brief Returns the current state of the piston
  /// @return The current state of the piston
  State getState() const;

  /// @brief Extends the piston
  void activate();

  /// @brief Retracts the piston
  void deactivate();

  /// @brief Determines whether the piston is extended via its `state`
  /// @return `true` if `state` is `EXTENDED`, `false` otherwise
  bool isActivated() const;

  /// @brief Set the `state` of the `solenoid` to be the opposite of what it
  /// currently is
  /// @return The new `state`
  State toggle();
};

}  // namespace aon
