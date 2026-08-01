#pragma once

namespace aon::auton {

struct JerryIoPathAuton {
  static constexpr const char* name = "TEST JerryIO Path";
  static constexpr double startX = -66.557;
  static constexpr double startY = -35.024;
  static constexpr double startHeading = 132.06;
  static constexpr float lookahead = 10.0F;
  static constexpr int timeoutMs = 14000;
  static constexpr int maximumPathSpeed = 127;
};

}  // namespace aon::auton
