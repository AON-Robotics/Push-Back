#pragma once

namespace aon::auton {

struct FigureEightValidation {
  static constexpr const char* name = "TEST LemLib Figure 8";
  static constexpr double startX = 0.0;
  static constexpr double startY = 18.0;
  static constexpr double startHeading = 90.0;
  static constexpr float lookahead = 7.0F;
  static constexpr int timeoutMs = 12000;
  static constexpr int maximumPathSpeed = 100;
};

}  // namespace aon::auton
