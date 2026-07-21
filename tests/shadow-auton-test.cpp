#include "aon/shadow/recorder.hpp"
#include "aon/shadow/processor.hpp"

#include <cstdlib>
#include <iostream>

#define CHECK(x) do { if (!(x)) { std::cerr << #x << '\n'; std::exit(1); } } while (false)

using namespace aon::shadow;

RawSample frame(std::uint32_t ms, float x) {
  RawSample value{};
  value.timeMs = ms;
  value.x = x;
  value.poseValid = true;
  return value;
}

RawSample routeFrame(std::uint32_t ms, float x, float y, float heading,
                     Direction direction) {
  RawSample value{};
  value.timeMs = ms;
  value.x = x;
  value.y = y;
  value.heading = heading;
  value.direction = direction;
  value.poseValid = true;
  return value;
}

Capture captureAt(std::initializer_list<RawSample> samples) {
  Capture capture{};
  for (const auto& sample : samples) {
    capture.samples[capture.sampleCount++] = sample;
    capture.durationMs = sample.timeMs;
  }
  return capture;
}

Capture captureWithPauseAndCartEvent() {
  Capture capture = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(100, 0, 6, 0, Direction::Forward),
      routeFrame(200, 0, 6, 0, Direction::Stopped),
      routeFrame(300, 0, 6, 0, Direction::Stopped),
      routeFrame(400, 0, 6, 0, Direction::Stopped),
      routeFrame(500, 0, 6, 0, Direction::Stopped),
      routeFrame(600, 0, 6, 0, Direction::Stopped),
      routeFrame(700, 0, 6, 0, Direction::Stopped),
      routeFrame(800, 0, 12, 0, Direction::Forward),
  });
  capture.events[capture.eventCount++] = {400, MechanismKind::Cart, 1};
  return capture;
}

void recorderTests() {
  Recorder recorder;
  CHECK(recorder.start(RobotIdentity::Small) == ResultCode::Ok);
  CHECK(recorder.sample(frame(0, 0)) == ResultCode::Ok);
  CHECK(recorder.sample(frame(10, 1)) == ResultCode::SampleTooSoon);
  CHECK(recorder.sample(frame(20, 1)) == ResultCode::Ok);
  CHECK(recorder.event({20, MechanismKind::Cart, 1}) == ResultCode::Ok);
  CHECK(recorder.event({30, MechanismKind::Cart, 1}) == ResultCode::DuplicateEvent);
  CHECK(recorder.event({40, MechanismKind::Cart, 0}) == ResultCode::Ok);
  auto bad = frame(40, 2);
  bad.poseValid = false;
  CHECK(recorder.sample(bad) == ResultCode::Ok);
  bad.timeMs = 60;
  CHECK(recorder.sample(bad) == ResultCode::Ok);
  bad.timeMs = 80;
  CHECK(recorder.sample(bad) == ResultCode::InvalidPose);
  CHECK(!recorder.isRecording());

  Recorder jumping;
  CHECK(jumping.start(RobotIdentity::Small) == ResultCode::Ok);
  CHECK(jumping.sample(frame(0, 0)) == ResultCode::Ok);
  CHECK(jumping.sample(frame(20, 20)) == ResultCode::Ok);
  CHECK(jumping.sample(frame(40, 40)) == ResultCode::PoseJump);
}

void capacityTests() {
  Recorder samples;
  CHECK(samples.start(RobotIdentity::Small) == ResultCode::Ok);
  for (std::size_t i = 0; i < kMaximumSamples; ++i) {
    CHECK(samples.sample(frame(static_cast<std::uint32_t>(i) * kSamplePeriodMs,
                               0)) == ResultCode::Ok);
  }
  CHECK(samples.sample(frame(kMaximumDurationMs, 0)) ==
        ResultCode::CapacityReached);
  CHECK(samples.capture().sampleCount == kMaximumSamples);
  CHECK(samples.capture().samples[kMaximumSamples - 1].timeMs ==
        kMaximumDurationMs - kSamplePeriodMs);
  CHECK(!samples.isRecording());

  Recorder events;
  CHECK(events.start(RobotIdentity::Small) == ResultCode::Ok);
  for (std::size_t i = 0; i < kMaximumEvents; ++i) {
    const auto kind = i % 2 == 0 ? MechanismKind::Cart
                                : MechanismKind::Trapdoor;
    CHECK(events.event({static_cast<std::uint32_t>(i), kind, 1}) ==
          ResultCode::Ok);
  }
  CHECK(events.event({static_cast<std::uint32_t>(kMaximumEvents),
                      MechanismKind::Cart, 1}) ==
        ResultCode::CapacityReached);
  CHECK(events.capture().eventCount == kMaximumEvents);
  CHECK(events.capture().events[kMaximumEvents - 1].timeMs ==
        kMaximumEvents - 1);
  CHECK(!events.isRecording());
}

const PathPoint& segmentPoint(const ProcessedRoute& route,
                              std::size_t segmentIndex,
                              std::size_t pointIndex) {
  const auto& segment = route.segments[segmentIndex];
  CHECK(pointIndex < segment.pointCount);
  return route.points[segment.firstPoint + pointIndex];
}

void processorTests() {
  const ProcessedRoute straight = process(captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
      routeFrame(40, 0, 12, 0, Direction::Forward),
  }));
  CHECK(straight.result == ResultCode::Ok);
  CHECK(straight.segmentCount == 1);
  CHECK(straight.segments[0].kind == SegmentKind::Motion);
  CHECK(straight.segments[0].direction == Direction::Forward);
  CHECK(straight.segments[0].pointCount == 2);
  CHECK(segmentPoint(straight, 0, 0).y == 0.0F);
  CHECK(segmentPoint(straight, 0, 1).y == 12.0F);

  const ProcessedRoute corner = process(captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
      routeFrame(40, 6, 6, 90, Direction::Forward),
  }));
  CHECK(corner.segmentCount == 1);
  CHECK(corner.segments[0].pointCount == 3);
  CHECK(segmentPoint(corner, 0, 1).x == 0.0F);
  CHECK(segmentPoint(corner, 0, 1).y == 6.0F);

  const ProcessedRoute wrappedHeading = process(captureAt({
      routeFrame(0, 0, 0, 179, Direction::Forward),
      routeFrame(20, 0, 6, -179, Direction::Forward),
      routeFrame(40, 0, 12, -175, Direction::Forward),
  }));
  CHECK(wrappedHeading.segmentCount == 1);
  CHECK(wrappedHeading.segments[0].pointCount == 2);

  const ProcessedRoute retainedHeadingReference = process(captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 4, Direction::Forward),
      routeFrame(40, 6, 6, 8, Direction::Forward),
      routeFrame(60, 12, 6, 8, Direction::Forward),
  }));
  CHECK(retainedHeadingReference.segments[0].pointCount == 3);

  const ProcessedRoute reversed = process(captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
      routeFrame(40, 0, 6, 0, Direction::Reverse),
      routeFrame(60, 0, 0, 0, Direction::Reverse),
  }));
  CHECK(reversed.segmentCount == 2);
  CHECK(reversed.segments[0].direction == Direction::Forward);
  CHECK(reversed.segments[1].direction == Direction::Reverse);
  CHECK(segmentPoint(reversed, 0, reversed.segments[0].pointCount - 1).y ==
        6.0F);
  CHECK(segmentPoint(reversed, 1, 0).y == 6.0F);

  const ProcessedRoute paused = process(captureWithPauseAndCartEvent());
  CHECK(paused.segmentCount == 3);
  CHECK(paused.segments[1].kind == SegmentKind::Dwell);
  CHECK(paused.segments[1].durationMs == 500);
  CHECK(paused.eventCount == 1);
  CHECK(paused.events[0].segmentIndex == 1);
  CHECK(paused.events[0].offsetMs == 200);
  CHECK(paused.segments[2].pointCount == 2);
  CHECK(segmentPoint(paused, 2, 0).y == 6.0F);
  CHECK(segmentPoint(paused, 2, 1).y == 12.0F);

  const ProcessedRoute stoppedFor99Ms = process(captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(100, 0, 6, 0, Direction::Stopped),
      routeFrame(120, 0, 6, 0, Direction::Stopped),
      routeFrame(140, 0, 6, 0, Direction::Stopped),
      routeFrame(160, 0, 6, 0, Direction::Stopped),
      routeFrame(199, 0, 6, 0, Direction::Stopped),
      routeFrame(220, 0, 12, 0, Direction::Forward),
  }));
  CHECK(stoppedFor99Ms.segmentCount == 1);

  const ProcessedRoute stoppedFor100Ms = process(captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(100, 0, 6, 0, Direction::Stopped),
      routeFrame(120, 0, 6, 0, Direction::Stopped),
      routeFrame(140, 0, 6, 0, Direction::Stopped),
      routeFrame(160, 0, 6, 0, Direction::Stopped),
      routeFrame(200, 0, 6, 0, Direction::Stopped),
      routeFrame(220, 0, 12, 0, Direction::Forward),
  }));
  CHECK(stoppedFor100Ms.segmentCount == 3);
  CHECK(stoppedFor100Ms.segments[1].kind == SegmentKind::Dwell);
  CHECK(stoppedFor100Ms.segments[1].durationMs == 100);

  Capture anchoredCorner = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
      routeFrame(40, 6, 6, 0, Direction::Forward),
      routeFrame(60, 12, 6, 0, Direction::Forward),
  });
  anchoredCorner.events[anchoredCorner.eventCount++] =
      {40, MechanismKind::Cart, 1};
  anchoredCorner.events[anchoredCorner.eventCount++] =
      {20, MechanismKind::Trapdoor, 1};
  const ProcessedRoute anchored = process(anchoredCorner);
  CHECK(anchored.eventCount == 2);
  CHECK(anchored.events[0].event.kind == MechanismKind::Trapdoor);
  CHECK(anchored.events[1].event.kind == MechanismKind::Cart);
  CHECK(anchored.segments[0].pointCount == 4);
  CHECK(segmentPoint(anchored, 0, 2).x == 6.0F);
  CHECK(segmentPoint(anchored, 0, 2).y == 6.0F);
  CHECK(anchored.events[1].segmentIndex == 0);
  CHECK(anchored.events[1].progress > 0.0F);
  CHECK(anchored.events[1].progress < 1.0F);
}

int main() {
  recorderTests();
  capacityTests();
  processorTests();
  std::cout << "shadow auton tests passed\n";
}
