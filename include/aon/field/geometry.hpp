#pragma once

#include <cmath>

namespace aon::field {

struct Point2 {
  double xInches = 0.0;
  double yInches = 0.0;
};

struct Segment {
  Point2 start;
  Point2 end;
};

struct Circle {
  Point2 center;
  double radiusInches = 0.0;
};

struct Rectangle {
  Point2 center;
  double halfWidthInches = 0.0;
  double halfHeightInches = 0.0;
  double headingRadians = 0.0;
};

struct Bounds {
  double minimumXInches = 0.0;
  double maximumXInches = 0.0;
  double minimumYInches = 0.0;
  double maximumYInches = 0.0;
};

struct NamedPose {
  const char* name = nullptr;
  Point2 position;
  double headingRadians = 0.0;
};

inline bool isFinite(Point2 point) noexcept {
  return std::isfinite(point.xInches) && std::isfinite(point.yInches);
}

inline bool isFinite(const Segment& segment) noexcept {
  return isFinite(segment.start) && isFinite(segment.end);
}

}  // namespace aon::field
