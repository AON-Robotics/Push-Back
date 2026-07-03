#pragma once

namespace aon::core {

/// Coordinates PROS lifecycle callbacks without taking hardware ownership yet.
///
/// Hardware remains in globals.hpp during this migration so the tested
/// construction order and mechanism behavior do not change.
class Robot {
 public:
  void initialize();
  void disabled();
  void competitionInitialize();
  void autonomous();
  void opcontrol();
};

}  // namespace aon::core
