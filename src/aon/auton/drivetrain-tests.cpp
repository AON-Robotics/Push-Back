#include "aon/auton/native-tests.hpp"

#include "aon/globals.hpp"

#include "pros/rtos.hpp"

namespace aon::routines {

int RunNativeForwardReverseTest() {
  drivetrain.setMaxVelocity(MAX_RPM / 4);
  drivetrain.move(6);
  pros::delay(500);
  drivetrain.move(-6);
  drivetrain.stop();
  drivetrain.setMaxVelocity(MAX_RPM);
  return 1;
}

int RunNativeTurnTest() {
  drivetrain.setMaxVelocity(MAX_RPM / 4);
  drivetrain.turn(45);
  pros::delay(500);
  drivetrain.turn(-45);
  drivetrain.stop();
  drivetrain.setMaxVelocity(MAX_RPM);
  return 1;
}

}  // namespace aon::routines
