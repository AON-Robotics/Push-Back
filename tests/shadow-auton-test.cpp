#include "aon/shadow/codec.hpp"
#include "aon/shadow/recorder.hpp"
#include "aon/shadow/processor.hpp"
#include "aon/shadow/service-state.hpp"
#include "aon/shadow/storage.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>

#define CHECK(x) do { if (!(x)) { std::cerr << #x << '\n'; std::exit(1); } } while (false)

using namespace aon::shadow;

class MemoryFileStore final : public FileStore {
 public:
  ResultCode read(const char* path, EncodedRecording& out) const override {
    const auto found = files.find(path);
    if (found == files.end()) return ResultCode::EmptyRecording;
    out = found->second;
    return ResultCode::Ok;
  }

  ResultCode write(const char* path, const std::uint8_t* data,
                   std::size_t size) override {
    if (failNextWrite) {
      failNextWrite = false;
      return ResultCode::WriteFailed;
    }
    if (data == nullptr || size > kMaximumEncodedBytes) {
      return ResultCode::WriteFailed;
    }
    auto& output = files[path];
    std::memcpy(output.data.data(), data, size);
    output.size = size;
    return ResultCode::Ok;
  }

  ResultCode erase(const char* path) override {
    files.erase(path);
    return ResultCode::Ok;
  }

  bool failNextWrite = false;
  std::unordered_map<std::string, EncodedRecording> files;
};

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
  static ProcessedRoute outputRoute{};
  const Capture outputCapture = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
  });
  CHECK(process(outputCapture, outputRoute) == ResultCode::Ok);
  CHECK(outputRoute.segmentCount == 1);

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
  for (std::size_t index = 0; index < capture.sampleCount; ++index) {
    auto& sample = capture.samples[index];
    const auto sampleIndex = static_cast<int>(index);
    sample.leftX = static_cast<std::int8_t>(-10 - sampleIndex);
    sample.leftY = static_cast<std::int8_t>(20 + sampleIndex);
    sample.rightX = static_cast<std::int8_t>(-30 - sampleIndex);
    sample.rightY = static_cast<std::int8_t>(40 + sampleIndex);
    sample.leftCommand = static_cast<std::int8_t>(-50 - sampleIndex);
    sample.rightCommand = static_cast<std::int8_t>(60 + sampleIndex);
  }
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
  for (std::size_t index = 0; index < capture.sampleCount; ++index) {
    const auto& expected = capture.samples[index];
    const auto& actual = decoded.capture.samples[index];
    CHECK(actual.timeMs == expected.timeMs);
    CHECK(actual.x == expected.x);
    CHECK(actual.y == expected.y);
    CHECK(actual.heading == expected.heading);
    CHECK(actual.leftX == expected.leftX);
    CHECK(actual.leftY == expected.leftY);
    CHECK(actual.rightX == expected.rightX);
    CHECK(actual.rightY == expected.rightY);
    CHECK(actual.leftCommand == expected.leftCommand);
    CHECK(actual.rightCommand == expected.rightCommand);
    CHECK(actual.direction == expected.direction);
    CHECK(actual.poseValid == expected.poseValid);
  }
  for (std::size_t index = 0; index < capture.eventCount; ++index) {
    const auto& expected = capture.events[index];
    const auto& actual = decoded.capture.events[index];
    CHECK(actual.timeMs == expected.timeMs);
    CHECK(actual.kind == expected.kind);
    CHECK(actual.value == expected.value);
  }
  CHECK(decoded.route.result == ResultCode::Ok);
  CHECK(decoded.route.segmentCount == route.segmentCount);
  CHECK(decoded.route.pointCount == route.pointCount);
  CHECK(decoded.route.eventCount == route.eventCount);
  CHECK(decoded.route.start.x == route.start.x);
  CHECK(decoded.route.start.y == route.start.y);
  CHECK(decoded.route.start.heading == route.start.heading);
  for (std::size_t index = 0; index < route.segmentCount; ++index) {
    const auto& expected = route.segments[index];
    const auto& actual = decoded.route.segments[index];
    CHECK(actual.kind == expected.kind);
    CHECK(actual.direction == expected.direction);
    CHECK(actual.firstPoint == expected.firstPoint);
    CHECK(actual.pointCount == expected.pointCount);
    CHECK(actual.durationMs == expected.durationMs);
  }
  for (std::size_t index = 0; index < route.pointCount; ++index) {
    const auto& expected = route.points[index];
    const auto& actual = decoded.route.points[index];
    CHECK(actual.x == expected.x);
    CHECK(actual.y == expected.y);
    CHECK(actual.speed == expected.speed);
  }
  for (std::size_t index = 0; index < route.eventCount; ++index) {
    const auto& expected = route.events[index];
    const auto& actual = decoded.route.events[index];
    CHECK(actual.event.timeMs == expected.event.timeMs);
    CHECK(actual.event.kind == expected.event.kind);
    CHECK(actual.event.value == expected.event.value);
    CHECK(actual.segmentIndex == expected.segmentIndex);
    CHECK(actual.progress == expected.progress);
    CHECK(actual.offsetMs == expected.offsetMs);
  }

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

void storageTests() {
  static MemoryFileStore files;
  static Storage storage(files);
  const Capture firstCapture = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
  });
  const ProcessedRoute firstRoute = process(firstCapture);
  CHECK(storage.save(0, RobotIdentity::Small, firstCapture, firstRoute) ==
        ResultCode::InvalidSlot);
  CHECK(storage.save(1, RobotIdentity::Small, firstCapture, firstRoute) ==
        ResultCode::Ok);

  Capture secondCapture = firstCapture;
  secondCapture.samples[1].y = 12;
  const ProcessedRoute secondRoute = process(secondCapture);
  files.failNextWrite = true;
  CHECK(storage.save(1, RobotIdentity::Small, secondCapture, secondRoute) ==
        ResultCode::WriteFailed);

  static DecodedRecording loaded{};
  CHECK(storage.load(1, RobotIdentity::Small, loaded) == ResultCode::Ok);
  CHECK(loaded.generation == 1);
  CHECK(loaded.capture.samples[1].y == 6);

  const SlotSummary summary = storage.inspect(1, RobotIdentity::Small);
  CHECK(summary.result == ResultCode::Ok);
  CHECK(summary.valid);
  CHECK(summary.generation == 1);
  CHECK(summary.durationMs == firstCapture.durationMs);
  CHECK(summary.startX == firstRoute.start.x);
  CHECK(storage.erase(1) == ResultCode::Ok);
  CHECK(storage.load(1, RobotIdentity::Small, loaded) ==
        ResultCode::EmptyRecording);
  CHECK(storage.erase(4) == ResultCode::InvalidSlot);
}

void serviceStateTests() {
  ServiceStateMachine state;
  CHECK(state.beginRecord(0, true) == ResultCode::InvalidSlot);
  CHECK(state.beginRecord(1, false) == ResultCode::Ok);
  CHECK(state.status().mode == ServiceMode::Recording);
  CHECK(state.authorizePlay(false, true, true) == ResultCode::PlayLocked);
  CHECK(state.beginProcessing() == ResultCode::Ok);
  CHECK(state.status().mode == ServiceMode::Processing);
  CHECK(state.finishSave(ResultCode::WriteFailed) == ResultCode::WriteFailed);
  CHECK(state.status().mode == ServiceMode::Invalid);
  CHECK(state.status().slot == 1);
  CHECK(state.status().result == ResultCode::WriteFailed);

  state.cancel();
  CHECK(state.status().mode == ServiceMode::Cancelled);
  CHECK(state.beginRecord(1, false) == ResultCode::Ok);
  CHECK(state.beginProcessing() == ResultCode::Ok);
  CHECK(state.finishSave(ResultCode::Ok) == ResultCode::Ok);
  CHECK(state.status().mode == ServiceMode::Saved);
  CHECK(state.beginRecord(1, false) == ResultCode::UnsafeState);
  CHECK(state.beginRecord(1, true) == ResultCode::Ok);
  CHECK(state.beginProcessing() == ResultCode::Ok);
  CHECK(state.finishSave(ResultCode::Ok) == ResultCode::Ok);

  CHECK(state.authorizePlay(true, false, true) == ResultCode::PlayLocked);
  CHECK(state.authorizePlay(true, true, false) == ResultCode::EmptyRecording);
  CHECK(state.authorizePlay(true, true, true) == ResultCode::Ok);
  CHECK(state.armPlay(1) == ResultCode::Ok);
  CHECK(state.consumeArm(1));
  CHECK(!state.consumeArm(1));
  CHECK(state.armPlay(0) == ResultCode::InvalidSlot);
}

int main() {
  recorderTests();
  capacityTests();
  processorTests();
  codecTests();
  storageTests();
  serviceStateTests();
  std::cout << "shadow auton tests passed\n";
}
