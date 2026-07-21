#include "aon/shadow/codec.hpp"
#include "aon/shadow/recorder.hpp"
#include "aon/shadow/processor.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
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

std::uint16_t encodedU16(const EncodedRecording& bytes, std::size_t offset) {
  return static_cast<std::uint16_t>(bytes.data[offset]) |
         static_cast<std::uint16_t>(bytes.data[offset + 1] << 8U);
}

std::uint32_t encodedU32(const EncodedRecording& bytes, std::size_t offset) {
  return static_cast<std::uint32_t>(bytes.data[offset]) |
         (static_cast<std::uint32_t>(bytes.data[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(bytes.data[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(bytes.data[offset + 3]) << 24U);
}

void setEncodedU16(EncodedRecording& bytes, std::size_t offset,
                   std::uint16_t value) {
  bytes.data[offset] = static_cast<std::uint8_t>(value);
  bytes.data[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

void setEncodedU32(EncodedRecording& bytes, std::size_t offset,
                   std::uint32_t value) {
  for (std::size_t index = 0; index < 4; ++index) {
    bytes.data[offset + index] =
        static_cast<std::uint8_t>(value >> (index * 8U));
  }
}

void codecTests() {
  Capture capture = captureWithPauseAndCartEvent();
  capture.robot = RobotIdentity::Small;
  capture.samples[1].leftX = -12;
  capture.samples[1].leftY = 34;
  capture.samples[1].rightX = -56;
  capture.samples[1].rightY = 78;
  capture.samples[1].leftCommand = -90;
  capture.samples[1].rightCommand = 91;
  const ProcessedRoute route = process(capture);

  static EncodedRecording bytes{};
  CHECK(encode(capture, route, 7, bytes) == ResultCode::Ok);
  CHECK(bytes.size > 0);
  static EncodedRecording repeated{};
  CHECK(encode(capture, route, 7, repeated) == ResultCode::Ok);
  CHECK(repeated.size == bytes.size);
  CHECK(std::memcmp(bytes.data.data(), repeated.data.data(), bytes.size) == 0);

  static DecodedRecording decoded{};
  CHECK(decode(bytes.data.data(), bytes.size, RobotIdentity::Small, decoded) ==
        ResultCode::Ok);
  CHECK(decoded.generation == 7);
  CHECK(decoded.capture.robot == capture.robot);
  CHECK(decoded.capture.sampleCount == capture.sampleCount);
  CHECK(decoded.capture.eventCount == capture.eventCount);
  CHECK(decoded.capture.durationMs == capture.durationMs);
  CHECK(decoded.capture.samples[1].y == capture.samples[1].y);
  CHECK(decoded.capture.samples[1].leftX == capture.samples[1].leftX);
  CHECK(decoded.capture.samples[1].rightY == capture.samples[1].rightY);
  CHECK(decoded.capture.samples[1].leftCommand ==
        capture.samples[1].leftCommand);
  CHECK(decoded.capture.samples[1].direction ==
        capture.samples[1].direction);
  CHECK(decoded.capture.samples[1].poseValid ==
        capture.samples[1].poseValid);
  CHECK(decoded.capture.events[0].kind == capture.events[0].kind);
  CHECK(decoded.capture.events[0].timeMs == capture.events[0].timeMs);
  CHECK(decoded.capture.events[0].value == capture.events[0].value);
  CHECK(decoded.route.result == ResultCode::Ok);
  CHECK(decoded.route.segmentCount == route.segmentCount);
  CHECK(decoded.route.pointCount == route.pointCount);
  CHECK(decoded.route.eventCount == route.eventCount);
  CHECK(decoded.route.start.x == route.start.x);
  CHECK(decoded.route.points[1].y == route.points[1].y);
  CHECK(decoded.route.segments[1].durationMs == route.segments[1].durationMs);
  CHECK(decoded.route.points[1].speed == route.points[1].speed);
  CHECK(decoded.route.events[0].progress == route.events[0].progress);
  CHECK(decoded.route.events[0].offsetMs == route.events[0].offsetMs);

  static EncodedRecording corrupted{};
  corrupted = bytes;
  corrupted.data[corrupted.size - 1] ^= 0x01U;
  CHECK(decode(corrupted.data.data(), corrupted.size, RobotIdentity::Small,
               decoded) == ResultCode::CorruptFile);

  constexpr std::size_t kHeaderSize = 69;
  CHECK(encodedU16(bytes, 10) == kHeaderSize);
  std::array<std::size_t, 6> boundaries{};
  boundaries[0] = kHeaderSize;
  for (std::size_t section = 0; section < 5; ++section) {
    boundaries[section + 1] = boundaries[section] +
        encodedU32(bytes, 45 + section * 4);
  }
  CHECK(boundaries[5] == bytes.size);
  for (std::size_t section = 0; section < 5; ++section) {
    CHECK(decode(bytes.data.data(), boundaries[section], RobotIdentity::Small,
                 decoded) == ResultCode::CorruptFile);
  }
  for (const auto boundary : boundaries) {
    CHECK(decode(bytes.data.data(), boundary - 1, RobotIdentity::Small,
                 decoded) == ResultCode::CorruptFile);
  }

  static EncodedRecording wrongMagic{};
  wrongMagic = bytes;
  wrongMagic.data[0] ^= 0x01U;
  CHECK(decode(wrongMagic.data.data(), wrongMagic.size, RobotIdentity::Small,
               decoded) == ResultCode::CorruptFile);

  static EncodedRecording versionTwo{};
  versionTwo = bytes;
  setEncodedU16(versionTwo, 8, 2);
  CHECK(decode(versionTwo.data.data(), versionTwo.size, RobotIdentity::Small,
               decoded) == ResultCode::UnsupportedVersion);
  CHECK(decode(bytes.data.data(), bytes.size, RobotIdentity::Big, decoded) ==
        ResultCode::WrongRobot);

  static EncodedRecording nonFinite{};
  nonFinite = bytes;
  setEncodedU32(nonFinite, 23, 0x7F800000U);
  CHECK(decode(nonFinite.data.data(), nonFinite.size, RobotIdentity::Small,
               decoded) == ResultCode::CorruptFile);

  static EncodedRecording impossibleCount{};
  impossibleCount = bytes;
  setEncodedU16(impossibleCount, 35,
                static_cast<std::uint16_t>(kMaximumSamples + 1));
  CHECK(decode(impossibleCount.data.data(), impossibleCount.size,
               RobotIdentity::Small, decoded) == ResultCode::CorruptFile);

  CHECK(chooseGeneration({true, 4}, {true, 9}) == Generation::B);
  CHECK(chooseGeneration({true, 4}, {false, 0}) == Generation::A);
  CHECK(chooseGeneration({false, 0}, {true, 9}) == Generation::B);
  CHECK(chooseGeneration({false, 0}, {false, 0}) == Generation::None);
  CHECK(chooseGeneration({true, 9}, {true, 9}) == Generation::A);
}

int main() {
  recorderTests();
  capacityTests();
  processorTests();
  codecTests();
  std::cout << "shadow auton tests passed\n";
}
