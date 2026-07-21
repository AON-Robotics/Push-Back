#pragma once

#include "aon/shadow/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace aon::shadow {

constexpr std::size_t kMaximumSegments = 64;
constexpr std::size_t kMaximumPathPoints = 1000;

enum class SegmentKind : std::uint8_t { Motion, Dwell };

struct PathPoint {
  float x = 0, y = 0, speed = 0;
};

struct RouteSegment {
  SegmentKind kind = SegmentKind::Motion;
  Direction direction = Direction::Forward;
  std::uint16_t firstPoint = 0, pointCount = 0;
  std::uint32_t durationMs = 0;
};

struct AnchoredEvent {
  MechanismEvent event{};
  std::uint16_t segmentIndex = 0;
  float progress = 0;
  std::uint32_t offsetMs = 0;
};

struct ProcessedRoute {
  ResultCode result = ResultCode::EmptyRecording;
  RawSample start{};
  std::array<RouteSegment, kMaximumSegments> segments{};
  std::array<PathPoint, kMaximumPathPoints> points{};
  std::array<AnchoredEvent, kMaximumEvents> events{};
  std::size_t segmentCount = 0, pointCount = 0, eventCount = 0;
};

ProcessedRoute process(const Capture& capture);
ResultCode process(const Capture& capture, ProcessedRoute& out);

}  // namespace aon::shadow
