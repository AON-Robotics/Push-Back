#pragma once

namespace aon::auton {

struct FigureEightValidation {
  static constexpr const char* name = "TEST LemLib Figure 8";
  static constexpr double startX = -20.0;
  static constexpr double startY = 22.0;
  static constexpr double startHeading = 0.0;
  static constexpr float lookahead = 8.0F;
  static constexpr int timeoutMs = 12000;
  static constexpr int maximumPathSpeed = 65;
};

}  // namespace aon::auton
