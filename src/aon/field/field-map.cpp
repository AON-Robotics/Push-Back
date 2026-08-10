#include "aon/field/field-map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::field {

FieldMap::FieldMap(Bounds bounds,
                   const std::array<Segment, kMaximumWalls>& walls,
                   std::size_t wallCount) noexcept
    : bounds_(bounds), walls_(walls), wallCount_(wallCount) {}

FieldMapIssue FieldMap::validate() const noexcept {
  if (!std::isfinite(bounds_.minimumXInches) ||
      !std::isfinite(bounds_.maximumXInches) ||
      !std::isfinite(bounds_.minimumYInches) ||
      !std::isfinite(bounds_.maximumYInches)) {
    return FieldMapIssue::NonFiniteBounds;
  }
  if (bounds_.minimumXInches >= bounds_.maximumXInches ||
      bounds_.minimumYInches >= bounds_.maximumYInches) {
    return FieldMapIssue::InvertedBounds;
  }
  if (wallCount_ > walls_.size()) return FieldMapIssue::TooManyWalls;
  for (std::size_t index = 0; index < wallCount_; ++index) {
    if (!isFinite(walls_[index])) return FieldMapIssue::InvalidWall;
  }
  return FieldMapIssue::None;
}

bool FieldMap::contains(Point2 point, double clearanceInches) const noexcept {
  if (!isFinite(point) || !std::isfinite(clearanceInches) ||
      clearanceInches < 0.0) {
    return false;
  }
  return point.xInches >= bounds_.minimumXInches + clearanceInches &&
         point.xInches <= bounds_.maximumXInches - clearanceInches &&
         point.yInches >= bounds_.minimumYInches + clearanceInches &&
         point.yInches <= bounds_.maximumYInches - clearanceInches;
}

double FieldMap::distanceToNearestWall(Point2 point) const noexcept {
  if (!isFinite(point)) return std::numeric_limits<double>::quiet_NaN();
  return std::min(
      {point.xInches - bounds_.minimumXInches,
       bounds_.maximumXInches - point.xInches,
       point.yInches - bounds_.minimumYInches,
       bounds_.maximumYInches - point.yInches});
}

bool FieldMap::segmentHasClearance(const Segment& segment,
                                   double clearanceInches) const noexcept {
  // The field boundary is convex, so a segment is contained when both of its
  // endpoints are contained by the same inset boundary.
  return isFinite(segment) && contains(segment.start, clearanceInches) &&
         contains(segment.end, clearanceInches);
}

const Bounds& FieldMap::bounds() const noexcept { return bounds_; }

std::size_t FieldMap::wallCount() const noexcept { return wallCount_; }

const Segment* FieldMap::wall(std::size_t index) const noexcept {
  return index < wallCount_ ? &walls_[index] : nullptr;
}

}  // namespace aon::field
