#pragma once

#include "pros/adi.hpp"

namespace aon {

class Piston {
 public:
  enum State {
    RETRACTED,
    EXTENDED,
  };

 private:
  pros::adi::DigitalOut solenoid;
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

  /**
   * @brief Returns the last state commanded through this wrapper.
   * @return `EXTENDED` or `RETRACTED`.
   *
   * This is command state, not pneumatic feedback; no position sensor is
   * attached to the cylinder.
   */
  State getState() const;

  /** @brief Commands the solenoid to extend the piston. */
  void activate();

  /** @brief Commands the solenoid to retract the piston. */
  void deactivate();

  /**
   * @brief Reports whether extension was last commanded.
   * @return `true` after `activate()` or an equivalent toggle.
   */
  bool isActivated() const;

  /**
   * @brief Commands the opposite of the current software state.
   * @return The newly commanded state.
   */
  State toggle();
};

}  // namespace aon
