#pragma once

#include <cstddef>

namespace aon::auton {

struct RedSixBlockGenerated {
  static constexpr double loaderStartX = 0.0;
  static constexpr double loaderStartY = 0.0;
  static constexpr double loaderEndX = 0.0;
  static constexpr double loaderEndY = 24.0;
  static constexpr double goalStartX = -9.0;
  static constexpr double goalStartY = 31.0;
  static constexpr double goalStageX = -8.3;
  static constexpr double goalStageY = 27.0;
  static constexpr double goalStageHeading = 171.0;
  static constexpr std::size_t loaderPointCount = 25;
  static constexpr std::size_t goalPointCount = 13;
};

}  // namespace aon::auton