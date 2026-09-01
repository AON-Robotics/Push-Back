#pragma once

namespace aon::auton {

struct JerryIoPathAuton {
  static constexpr const char* name = "TEST JerryIO Path";
  static constexpr double startX = -66.557;
  static constexpr double startY = -35.024;
  static constexpr double startHeading = 132.06;
  static constexpr double intakeX = -12.547;
  static constexpr double intakeY = -21.611;
  static constexpr double intakeHeading = 270.0;
  static constexpr double outtakeX = -39.91;
  static constexpr double outtakeY = -23.221;
  static constexpr double outtakeHeading = 270.0;
  static constexpr double pistonsX = -60.298;
  static constexpr double pistonsY = -58.81;
  static constexpr double pistonsHeading = 270.0;
  static constexpr float lookahead = 10.0F;
  static constexpr int timeoutMs = 14000;
  static constexpr int headingTimeoutMs = 1800;
  static constexpr int actionDurationMs = 2000;
  static constexpr int maximumPathSpeed = 127;
};

}  // namespace aon::auton
