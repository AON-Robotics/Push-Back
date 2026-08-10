#pragma once

#include <array>
#include <cstddef>

#include "aon/field/geometry.hpp"

namespace aon::field {

enum class FieldMapIssue {
  None,
  NonFiniteBounds,
  InvertedBounds,
  TooManyWalls,
  InvalidWall,
};

class FieldMap {
 public:
  static constexpr std::size_t kMaximumWalls = 16;

  FieldMap(Bounds bounds,
           const std::array<Segment, kMaximumWalls>& walls,
           std::size_t wallCount) noexcept;

  [[nodiscard]] FieldMapIssue validate() const noexcept;
  [[nodiscard]] bool contains(Point2 point,
                              double clearanceInches = 0.0) const noexcept;
  [[nodiscard]] double distanceToNearestWall(Point2 point) const noexcept;
  [[nodiscard]] bool segmentHasClearance(
      const Segment& segment, double clearanceInches) const noexcept;
  [[nodiscard]] const Bounds& bounds() const noexcept;
  [[nodiscard]] std::size_t wallCount() const noexcept;
  [[nodiscard]] const Segment* wall(std::size_t index) const noexcept;

 private:
  Bounds bounds_;
  std::array<Segment, kMaximumWalls> walls_;
  std::size_t wallCount_;
};

}  // namespace aon::field
