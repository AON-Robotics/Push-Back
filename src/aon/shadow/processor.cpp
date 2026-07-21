#include "aon/shadow/processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace aon::shadow {
namespace {

constexpr std::size_t kDwellSampleCount = 5;
constexpr std::uint32_t kMinimumDwellDurationMs = 100;
constexpr float kMaximumPerpendicularError = 0.5F;
constexpr float kHeadingRetentionDegrees = 5.0F;

struct SegmentSource {
  SegmentKind kind = SegmentKind::Motion;
  Direction direction = Direction::Forward;
  std::size_t first = 0;
  std::size_t last = 0;
};

float headingDifference(float first, float second) {
  return std::fabs(std::remainder(second - first, 360.0F));
}

float distance(const RawSample& first, const RawSample& second) {
  return std::hypot(second.x - first.x, second.y - first.y);
}

float perpendicularError(const RawSample& point, const RawSample& first,
                         const RawSample& last) {
  const float dx = last.x - first.x;
  const float dy = last.y - first.y;
  const float length = std::hypot(dx, dy);
  if (length == 0.0F) return distance(point, first);
  return std::fabs(dy * point.x - dx * point.y + last.x * first.y -
                   last.y * first.x) /
         length;
}

float inferredSpeed(const Capture& capture, std::size_t index,
                    const SegmentSource& source) {
  std::size_t other = index;
  if (index < source.last) {
    other = index + 1;
  } else if (index > source.first) {
    other = index - 1;
  }
  const float raw = std::round(distance(capture.samples[index],
                                        capture.samples[other]) /
                               0.02F * 2.0F);
  return std::clamp(raw, 20.0F, 100.0F);
}

std::size_t nearestSample(const Capture& capture, const SegmentSource& source,
                          std::uint32_t timeMs) {
  std::size_t nearest = source.first;
  std::uint32_t best = capture.samples[nearest].timeMs > timeMs
                           ? capture.samples[nearest].timeMs - timeMs
                           : timeMs - capture.samples[nearest].timeMs;
  for (std::size_t i = source.first + 1; i <= source.last; ++i) {
    const std::uint32_t difference = capture.samples[i].timeMs > timeMs
                                         ? capture.samples[i].timeMs - timeMs
                                         : timeMs - capture.samples[i].timeMs;
    if (difference < best) {
      best = difference;
      nearest = i;
    }
  }
  return nearest;
}

std::size_t sourceForEvent(const Capture& capture,
                           const std::array<SegmentSource,
                                            kMaximumSegments>& sources,
                           std::size_t sourceCount, std::uint32_t timeMs) {
  for (std::size_t i = 0; i < sourceCount; ++i) {
    if (sources[i].kind != SegmentKind::Dwell) continue;
    const auto start = capture.samples[sources[i].first].timeMs;
    const auto end = capture.samples[sources[i].last].timeMs;
    if (timeMs >= start && timeMs <= end) return i;
  }
  for (std::size_t i = 0; i < sourceCount; ++i) {
    const auto start = capture.samples[sources[i].first].timeMs;
    const auto end = capture.samples[sources[i].last].timeMs;
    if (timeMs >= start && timeMs <= end) return i;
    if (timeMs < start) return i;
  }
  return sourceCount - 1;
}

float progressAt(const Capture& capture, const SegmentSource& source,
                 std::uint32_t timeMs) {
  float total = 0.0F;
  float elapsed = 0.0F;
  for (std::size_t i = source.first + 1; i <= source.last; ++i) {
    const auto& previous = capture.samples[i - 1];
    const auto& current = capture.samples[i];
    const float step = distance(previous, current);
    total += step;
    if (timeMs >= current.timeMs) {
      elapsed += step;
    } else if (timeMs > previous.timeMs && current.timeMs > previous.timeMs) {
      const float fraction = static_cast<float>(timeMs - previous.timeMs) /
                             static_cast<float>(current.timeMs -
                                                previous.timeMs);
      elapsed += step * fraction;
    }
  }
  return total == 0.0F ? 0.0F : std::clamp(elapsed / total, 0.0F, 1.0F);
}

bool appendMotionPoints(
    const Capture& capture, const SegmentSource& source,
    const std::array<bool, kMaximumSamples>& eventAnchor,
    ProcessedRoute& route) {
  std::array<bool, kMaximumSamples> retained{};
  retained[source.first] = true;
  retained[source.last] = true;
  for (std::size_t i = source.first; i <= source.last; ++i) {
    retained[i] = retained[i] || eventAnchor[i];
  }

  std::array<std::size_t, kMaximumSamples> intervalFirst{};
  std::array<std::size_t, kMaximumSamples> intervalLast{};
  std::size_t stackSize = 0;
  std::size_t anchor = source.first;
  while (anchor < source.last) {
    std::size_t next = anchor + 1;
    while (next < source.last && !retained[next]) ++next;
    intervalFirst[stackSize] = anchor;
    intervalLast[stackSize] = next;
    ++stackSize;
    anchor = next;
  }

  while (stackSize > 0) {
    --stackSize;
    const std::size_t first = intervalFirst[stackSize];
    const std::size_t last = intervalLast[stackSize];
    float maximumError = kMaximumPerpendicularError;
    std::size_t maximumIndex = last;
    for (std::size_t i = first + 1; i < last; ++i) {
      const float error = perpendicularError(capture.samples[i],
                                             capture.samples[first],
                                             capture.samples[last]);
      if (error > maximumError) {
        maximumError = error;
        maximumIndex = i;
      }
    }
    if (maximumIndex != last) {
      retained[maximumIndex] = true;
      intervalFirst[stackSize] = first;
      intervalLast[stackSize] = maximumIndex;
      ++stackSize;
      intervalFirst[stackSize] = maximumIndex;
      intervalLast[stackSize] = last;
      ++stackSize;
    }
  }

  std::size_t lastHeadingPoint = source.first;
  for (std::size_t i = source.first + 1; i < source.last; ++i) {
    if (retained[i]) {
      lastHeadingPoint = i;
    } else if (headingDifference(capture.samples[lastHeadingPoint].heading,
                                 capture.samples[i].heading) >=
               kHeadingRetentionDegrees) {
      retained[i] = true;
      lastHeadingPoint = i;
    }
  }

  auto& segment = route.segments[route.segmentCount];
  segment.firstPoint = static_cast<std::uint16_t>(route.pointCount);
  for (std::size_t i = source.first; i <= source.last; ++i) {
    if (!retained[i]) continue;
    if (route.pointCount == kMaximumPathPoints) return false;
    route.points[route.pointCount++] = {
        capture.samples[i].x, capture.samples[i].y,
        inferredSpeed(capture, i, source)};
    ++segment.pointCount;
  }
  return true;
}

}  // namespace

ResultCode process(const Capture& capture, ProcessedRoute& route) {
  route = {};
  if (capture.sampleCount == 0) return route.result;
  if (capture.sampleCount > kMaximumSamples ||
      capture.eventCount > kMaximumEvents) {
    route.result = ResultCode::CapacityReached;
    return route.result;
  }
  route.start = capture.samples[0];

  std::array<bool, kMaximumSamples> dwell{};
  for (std::size_t first = 0; first < capture.sampleCount;) {
    if (capture.samples[first].direction != Direction::Stopped) {
      ++first;
      continue;
    }
    std::size_t last = first;
    while (last + 1 < capture.sampleCount &&
           capture.samples[last + 1].direction == Direction::Stopped) {
      ++last;
    }
    const auto firstTime = capture.samples[first].timeMs;
    const auto lastTime = capture.samples[last].timeMs;
    if (last - first + 1 >= kDwellSampleCount && lastTime >= firstTime &&
        lastTime - firstTime >= kMinimumDwellDurationMs) {
      for (std::size_t i = first; i <= last; ++i) dwell[i] = true;
    }
    first = last + 1;
  }

  std::array<Direction, kMaximumSamples> direction{};
  Direction previous = Direction::Stopped;
  for (std::size_t i = 0; i < capture.sampleCount; ++i) {
    if (!dwell[i] && capture.samples[i].direction != Direction::Stopped) {
      previous = capture.samples[i].direction;
    }
    direction[i] = previous;
  }
  Direction next = Direction::Forward;
  for (std::size_t i = capture.sampleCount; i-- > 0;) {
    if (!dwell[i] && capture.samples[i].direction != Direction::Stopped) {
      next = capture.samples[i].direction;
    }
    if (!dwell[i] && direction[i] == Direction::Stopped) direction[i] = next;
  }

  std::array<SegmentSource, kMaximumSegments> sources{};
  std::size_t sourceCount = 0;
  for (std::size_t first = 0; first < capture.sampleCount;) {
    const SegmentKind kind = dwell[first] ? SegmentKind::Dwell
                                          : SegmentKind::Motion;
    const Direction runDirection = kind == SegmentKind::Dwell
                                       ? Direction::Stopped
                                       : direction[first];
    std::size_t last = first;
    while (
        last + 1 < capture.sampleCount &&
        (dwell[last + 1] ? SegmentKind::Dwell : SegmentKind::Motion) == kind &&
        (kind == SegmentKind::Dwell || direction[last + 1] == runDirection)) {
      ++last;
    }
    if (sourceCount == kMaximumSegments) {
      route.result = ResultCode::CapacityReached;
      return route.result;
    }
    sources[sourceCount++] = {kind, runDirection, first, last};
    first = last + 1;
  }
  for (std::size_t i = 1; i < sourceCount; ++i) {
    if (sources[i - 1].kind == SegmentKind::Motion &&
        sources[i].kind == SegmentKind::Dwell) {
      sources[i - 1].last = sources[i].first;
    } else if (sources[i - 1].kind == SegmentKind::Dwell &&
               sources[i].kind == SegmentKind::Motion) {
      sources[i].first = sources[i - 1].last;
    }
  }

  std::array<std::size_t, kMaximumEvents> eventOrder{};
  for (std::size_t i = 0; i < capture.eventCount; ++i) {
    std::size_t position = i;
    while (position > 0 &&
           capture.events[eventOrder[position - 1]].timeMs >
               capture.events[i].timeMs) {
      eventOrder[position] = eventOrder[position - 1];
      --position;
    }
    eventOrder[position] = i;
  }

  std::array<std::size_t, kMaximumEvents> eventSource{};
  std::array<bool, kMaximumSamples> eventAnchor{};
  for (std::size_t i = 0; i < capture.eventCount; ++i) {
    const auto& event = capture.events[eventOrder[i]];
    eventSource[i] =
        sourceForEvent(capture, sources, sourceCount, event.timeMs);
    const auto& source = sources[eventSource[i]];
    if (source.kind == SegmentKind::Motion) {
      eventAnchor[nearestSample(capture, source, event.timeMs)] = true;
    }
  }

  for (std::size_t i = 0; i < sourceCount; ++i) {
    const auto& source = sources[i];
    auto& segment = route.segments[route.segmentCount];
    segment.kind = source.kind;
    segment.direction = source.direction;
    if (source.kind == SegmentKind::Dwell) {
      segment.firstPoint = static_cast<std::uint16_t>(route.pointCount);
      segment.durationMs = capture.samples[source.last].timeMs -
                           capture.samples[source.first].timeMs;
    } else if (!appendMotionPoints(capture, source, eventAnchor, route)) {
      route.result = ResultCode::CapacityReached;
      route.segmentCount = 0;
      route.pointCount = 0;
      route.eventCount = 0;
      return route.result;
    }
    ++route.segmentCount;
  }

  for (std::size_t i = 0; i < capture.eventCount; ++i) {
    const auto& event = capture.events[eventOrder[i]];
    const auto& source = sources[eventSource[i]];
    AnchoredEvent anchored{};
    anchored.event = event;
    anchored.segmentIndex = static_cast<std::uint16_t>(eventSource[i]);
    if (source.kind == SegmentKind::Dwell) {
      const auto start = capture.samples[source.first].timeMs;
      anchored.offsetMs = event.timeMs > start ? event.timeMs - start : 0;
      anchored.offsetMs = std::min(anchored.offsetMs,
                                   route.segments[eventSource[i]].durationMs);
    } else {
      anchored.progress = progressAt(capture, source, event.timeMs);
    }
    route.events[route.eventCount++] = anchored;
  }

  route.result = ResultCode::Ok;
  return route.result;
}

ProcessedRoute process(const Capture& capture) {
  ProcessedRoute route{};
  process(capture, route);
  return route;
}

}  // namespace aon::shadow
