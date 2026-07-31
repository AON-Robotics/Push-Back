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

struct ProcessorSource {
  SegmentKind kind = SegmentKind::Motion;
  Direction direction = Direction::Forward;
  std::size_t first = 0;
  std::size_t last = 0;
};

struct ProcessorWorkspace {
  std::array<bool, kMaximumSamples> dwell{};
  std::array<bool, kMaximumSamples> retained{};
  std::array<bool, kMaximumSamples> eventAnchor{};
  std::array<Direction, kMaximumSamples> direction{};
  std::array<std::size_t, kMaximumSamples> intervalFirst{};
  std::array<std::size_t, kMaximumSamples> intervalLast{};
  std::array<ProcessorSource, kMaximumSegments> sources{};
  std::array<std::size_t, kMaximumEvents> eventOrder{};
  std::array<std::size_t, kMaximumEvents> eventSource{};
};

ProcessedRoute process(const Capture& capture);
ResultCode process(const Capture& capture, ProcessedRoute& out);
ResultCode process(const Capture& capture, ProcessedRoute& out,
                   ProcessorWorkspace& workspace);

}  // namespace aon::shadow
