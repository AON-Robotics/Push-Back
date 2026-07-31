#include "aon/shadow/codec.hpp"
#include "aon/intake/release-request.hpp"
#include "aon/auton/routines.hpp"
#include "aon/lemlib/drive-command.hpp"
#include "aon/shadow/mechanisms.hpp"
#include "aon/shadow/player.hpp"
#include "aon/shadow/player-pros.hpp"
#include "aon/shadow/recorder.hpp"
#include "aon/shadow/processor.hpp"
#include "aon/shadow/service.hpp"
#include "aon/shadow/service-state.hpp"
#include "aon/shadow/storage.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#define CHECK(x) do { if (!(x)) { std::cerr << #x << '\n'; std::exit(1); } } while (false)

using namespace aon::shadow;
using aon::lemlib_integration::packDriveCommand;
using aon::lemlib_integration::unpackDriveCommand;

static_assert(std::is_same_v<decltype(&aon::routines::RunShadowPlayback),
                             int (*)()>);

DecodedRecording playbackRecording() {
  DecodedRecording recording{};
  recording.capture.robot = RobotIdentity::Small;
  recording.capture.durationMs = 300;
  recording.route.result = ResultCode::Ok;
  recording.route.segmentCount = 3;
  recording.route.pointCount = 4;
  recording.route.eventCount = 3;
  recording.route.points[0] = {0.0F, 0.0F, 40.0F};
  recording.route.points[1] = {0.0F, 12.0F, 0.0F};
  recording.route.points[2] = {0.0F, 12.0F, 40.0F};
  recording.route.points[3] = {0.0F, 0.0F, 0.0F};
  recording.route.segments[0] =
      {SegmentKind::Motion, Direction::Forward, 0, 2, 100};
  recording.route.segments[1] =
      {SegmentKind::Dwell, Direction::Stopped, 0, 0, 100};
  recording.route.segments[2] =
      {SegmentKind::Motion, Direction::Reverse, 2, 2, 100};
  recording.route.events[0] =
      {{40, MechanismKind::Cart, 1}, 0, 0.4F, 0};
  recording.route.events[1] =
      {{50, MechanismKind::IntakeMode,
        static_cast<std::int16_t>(IntakeIntent::Corridor)},
       0, 0.4F, 0};
  recording.route.events[2] =
      {{150, MechanismKind::Cart, 0}, 1, 0.0F, 50};
  return recording;
}

struct PlaybackSpy {
  int follows = 0;
  int dwells = 0;
  int mechanisms = 0;
  int stops = 0;
  bool cancel = false;
  ResultCode followResult = ResultCode::Ok;
  ResultCode mechanismResult = ResultCode::Ok;
  std::vector<std::string> log;
};

PlaybackCallbacks playbackCallbacks(PlaybackSpy& spy) {
  return {
      [&spy](const ProcessedRoute&, const RouteSegment& segment,
             const MotionProgress& progress) {
        ++spy.follows;
        spy.log.push_back(segment.direction == Direction::Forward
                              ? "follow-forward"
                              : "follow-reverse");
        if (spy.followResult != ResultCode::Ok) return spy.followResult;
        for (const float value : {0.0F, 0.2F, 0.6F, 1.0F}) {
          const ResultCode result = progress(value);
          if (result != ResultCode::Ok) return result;
        }
        return ResultCode::Ok;
      },
      [&spy](std::uint32_t durationMs, const DwellProgress& progress) {
        ++spy.dwells;
        spy.log.push_back("dwell");
        for (const std::uint32_t value : {0U, 20U, 40U, durationMs}) {
          const ResultCode result = progress(value);
          if (result != ResultCode::Ok) return result;
        }
        return ResultCode::Ok;
      },
      [&spy](const MechanismEvent& event) {
        ++spy.mechanisms;
        spy.log.push_back("event-" + std::to_string(event.value));
        return spy.mechanismResult;
      },
      [&spy]() { return spy.cancel; },
      [&spy]() {
        ++spy.stops;
        spy.log.push_back("stop");
      },
  };
}

void playbackPolicyTests() {
  const auto recording = playbackRecording();
  const PlaybackPolicy policies[] = {
      {false, true, RobotIdentity::Small},
      {true, false, RobotIdentity::Small},
      {true, true, RobotIdentity::Big},
  };
  const ResultCode expected[] = {
      ResultCode::PlayLocked, ResultCode::PlayLocked,
      ResultCode::UnsupportedRobot,
  };
  for (std::size_t index = 0; index < 3; ++index) {
    PlaybackSpy spy;
    auto callbacks = playbackCallbacks(spy);
    CHECK(playRecording(recording, policies[index], callbacks) ==
          expected[index]);
    CHECK(spy.follows == 0);
    CHECK(spy.dwells == 0);
    CHECK(spy.mechanisms == 0);
    CHECK(spy.stops == 1);
  }

  auto wrongRobot = recording;
  wrongRobot.capture.robot = RobotIdentity::Big;
  PlaybackSpy wrongRobotSpy;
  auto wrongRobotCallbacks = playbackCallbacks(wrongRobotSpy);
  CHECK(playRecording(wrongRobot,
                      {true, true, RobotIdentity::Small},
                      wrongRobotCallbacks) == ResultCode::WrongRobot);
  CHECK(wrongRobotSpy.follows == 0);
  CHECK(wrongRobotSpy.stops == 1);

  std::vector<DecodedRecording> corruptRecordings;
  auto corruptPoint = recording;
  corruptPoint.route.points[1].x = std::numeric_limits<float>::infinity();
  corruptRecordings.push_back(corruptPoint);
  auto corruptRange = recording;
  corruptRange.route.segments[0].firstPoint = kMaximumPathPoints - 1;
  corruptRecordings.push_back(corruptRange);
  auto corruptDirection = recording;
  corruptDirection.route.segments[0].direction = Direction::Stopped;
  corruptRecordings.push_back(corruptDirection);
  auto corruptEventIndex = recording;
  corruptEventIndex.route.events[0].segmentIndex = 3;
  corruptRecordings.push_back(corruptEventIndex);
  auto corruptMechanism = recording;
  corruptMechanism.route.events[0].event.value = 2;
  corruptRecordings.push_back(corruptMechanism);

  for (const auto& corrupt : corruptRecordings) {
    PlaybackSpy corruptSpy;
    auto corruptCallbacks = playbackCallbacks(corruptSpy);
    CHECK(playRecording(corrupt, {true, true, RobotIdentity::Small},
                        corruptCallbacks) == ResultCode::CorruptFile);
    CHECK(corruptSpy.follows == 0);
    CHECK(corruptSpy.dwells == 0);
    CHECK(corruptSpy.mechanisms == 0);
    CHECK(corruptSpy.stops == 1);
  }

  CHECK(playbackTimeoutMs(0) == 2000);
  CHECK(playbackTimeoutMs(600) == 2200);
  CHECK(playbackTimeoutMs(std::numeric_limits<std::uint32_t>::max()) ==
        std::numeric_limits<int>::max());
}

void playbackSchedulerTests() {
  const auto recording = playbackRecording();
  const PlaybackPolicy policy{true, true, RobotIdentity::Small};

  PlaybackSpy success;
  auto successCallbacks = playbackCallbacks(success);
  CHECK(playRecording(recording, policy, successCallbacks) == ResultCode::Ok);
  CHECK(success.log == std::vector<std::string>({
      "follow-forward", "event-1", "event-2", "dwell", "event-0",
      "follow-reverse", "stop"}));
  CHECK(success.mechanisms == 3);
  CHECK(success.stops == 1);

  PlaybackSpy motionFailure;
  motionFailure.followResult = ResultCode::MotionFailure;
  auto motionFailureCallbacks = playbackCallbacks(motionFailure);
  CHECK(playRecording(recording, policy, motionFailureCallbacks) ==
        ResultCode::MotionFailure);
  CHECK(motionFailure.follows == 1);
  CHECK(motionFailure.dwells == 0);
  CHECK(motionFailure.mechanisms == 0);
  CHECK(motionFailure.stops == 1);

  PlaybackSpy mechanismFailure;
  mechanismFailure.mechanismResult = ResultCode::MotionFailure;
  auto mechanismFailureCallbacks = playbackCallbacks(mechanismFailure);
  CHECK(playRecording(recording, policy, mechanismFailureCallbacks) ==
        ResultCode::MotionFailure);
  CHECK(mechanismFailure.follows == 1);
  CHECK(mechanismFailure.dwells == 0);
  CHECK(mechanismFailure.mechanisms == 1);
  CHECK(mechanismFailure.stops == 1);

  PlaybackSpy motionCancellation;
  auto motionCancellationCallbacks = playbackCallbacks(motionCancellation);
  motionCancellationCallbacks.follow =
      [&motionCancellation](const ProcessedRoute&, const RouteSegment&,
                            const MotionProgress& progress) {
        ++motionCancellation.follows;
        CHECK(progress(0.2F) == ResultCode::Ok);
        motionCancellation.cancel = true;
        return progress(0.6F);
      };
  CHECK(playRecording(recording, policy, motionCancellationCallbacks) ==
        ResultCode::Cancelled);
  CHECK(motionCancellation.follows == 1);
  CHECK(motionCancellation.dwells == 0);
  CHECK(motionCancellation.mechanisms == 0);
  CHECK(motionCancellation.stops == 1);

  PlaybackSpy dwellCancellation;
  auto dwellCancellationCallbacks = playbackCallbacks(dwellCancellation);
  dwellCancellationCallbacks.dwell =
      [&dwellCancellation](std::uint32_t, const DwellProgress& progress) {
        ++dwellCancellation.dwells;
        dwellCancellation.log.push_back("dwell");
        CHECK(progress(20) == ResultCode::Ok);
        dwellCancellation.cancel = true;
        return progress(40);
      };
  CHECK(playRecording(recording, policy, dwellCancellationCallbacks) ==
        ResultCode::Cancelled);
  CHECK(dwellCancellation.follows == 1);
  CHECK(dwellCancellation.dwells == 1);
  CHECK(dwellCancellation.mechanisms == 2);
  CHECK(dwellCancellation.stops == 1);
}

void playbackProjectionTests() {
  const PathPoint line[] = {
      {0.0F, 0.0F, 40.0F}, {10.0F, 0.0F, 40.0F},
      {20.0F, 0.0F, 0.0F}};
  CHECK(monotonicPolylineProgress(line, 3, 0.0F, 0.0F, 0.0F) == 0.0F);
  CHECK(monotonicPolylineProgress(line, 3, 10.0F, 0.0F, 0.0F) == 0.5F);
  CHECK(monotonicPolylineProgress(line, 3, 20.0F, 0.0F, 0.0F) == 1.0F);
  CHECK(monotonicPolylineProgress(line, 3, 5.0F, 3.0F, 0.0F) == 0.25F);
  CHECK(monotonicPolylineProgress(line, 3, 2.0F, 0.0F, 0.7F) == 0.7F);
  CHECK(monotonicPolylineProgress(nullptr, 0, 0.0F, 0.0F, 0.4F) ==
        0.4F);
  const PathPoint zeroLength[] = {
      {1.0F, 1.0F, 10.0F}, {1.0F, 1.0F, 0.0F}};
  CHECK(monotonicPolylineProgress(zeroLength, 2, 1.0F, 1.0F, 0.3F) ==
        0.3F);
  CHECK(monotonicPolylineProgress(line, 3,
                                  std::numeric_limits<float>::infinity(),
                                  0.0F, 0.2F) == 0.2F);
  const PathPoint crossing[] = {
      {-1.0F, -1.0F, 20.0F}, {1.0F, 1.0F, 20.0F},
      {-1.0F, 1.0F, 20.0F}, {1.0F, -1.0F, 0.0F}};
  CHECK(monotonicPolylineProgress(crossing, 4, 0.0F, 0.0F, 0.7F) >=
        0.7F);
}

void runtimePathSerializationTests() {
  const auto recording = playbackRecording();
  RuntimePath output{};
  CHECK(serializeRuntimePath(recording.route, recording.route.segments[0],
                             output) == ResultCode::Ok);
  CHECK(output.used > 0);
  CHECK(output.used <= kRuntimePathCapacity);
  const std::string text(reinterpret_cast<const char*>(output.bytes.data()),
                         output.used);
  CHECK(text.find("0.000000, 0.000000, 40.000000\n") == 0);
  CHECK(text.find("0.000000, 12.000000, 0.000000\n") !=
        std::string::npos);
  CHECK(text.find("endData\n") != std::string::npos);
  CHECK(text.find("#PATH.JERRYIO-DATA {\"appVersion\":\"0.11.0\",") !=
        std::string::npos);
  CHECK(text.find("\"format\":\"LemLib v0.5\"") != std::string::npos);
  CHECK(text.find("\"name\":\"Shadow Runtime\"}") !=
        std::string::npos);

  auto invalidRange = recording.route;
  invalidRange.segments[0].firstPoint = kMaximumPathPoints - 1;
  CHECK(serializeRuntimePath(invalidRange, invalidRange.segments[0], output) ==
        ResultCode::CorruptFile);
  CHECK(output.used == 0);

  auto invalidCount = recording.route;
  invalidCount.segments[0].pointCount = 1;
  CHECK(serializeRuntimePath(invalidCount, invalidCount.segments[0], output) ==
        ResultCode::CorruptFile);
  CHECK(output.used == 0);

  auto invalidNan = recording.route;
  invalidNan.points[0].x = std::numeric_limits<float>::quiet_NaN();
  CHECK(serializeRuntimePath(invalidNan, invalidNan.segments[0], output) ==
        ResultCode::CorruptFile);
  CHECK(output.used == 0);

  auto invalidInfinity = recording.route;
  invalidInfinity.points[0].speed =
      std::numeric_limits<float>::infinity();
  CHECK(serializeRuntimePath(invalidInfinity, invalidInfinity.segments[0],
                             output) == ResultCode::CorruptFile);
  CHECK(output.used == 0);

  auto invalidZeroLength = recording.route;
  invalidZeroLength.points[1] = invalidZeroLength.points[0];
  CHECK(serializeRuntimePath(invalidZeroLength,
                             invalidZeroLength.segments[0], output) ==
        ResultCode::CorruptFile);
  CHECK(output.used == 0);

  ProcessedRoute oversized{};
  oversized.result = ResultCode::Ok;
  oversized.pointCount = kMaximumPathPoints;
  RouteSegment oversizedSegment{SegmentKind::Motion, Direction::Forward, 0,
                                kMaximumPathPoints, 1000};
  for (std::size_t index = 0; index < oversized.pointCount; ++index) {
    oversized.points[index] = {
        index % 2 == 0 ? std::numeric_limits<float>::max()
                       : -std::numeric_limits<float>::max(),
        static_cast<float>(index), index + 1 == oversized.pointCount ? 0.0F
                                                                     : 100.0F};
  }
  CHECK(serializeRuntimePath(oversized, oversizedSegment, output) ==
        ResultCode::CapacityReached);
  CHECK(output.used == 0);
}

void releaseRequestTests() {
  using aon::intake_sync::nextReleaseRequest;
  using aon::intake_sync::releaseRequestActive;

  const std::uint32_t releaseOn = nextReleaseRequest(0, true);
  CHECK(releaseOn != 0);
  CHECK(releaseRequestActive(releaseOn));

  const std::uint32_t releaseOff = nextReleaseRequest(releaseOn, false);
  CHECK(releaseOff != releaseOn);
  CHECK(!releaseRequestActive(releaseOff));

  const std::uint32_t wrapped = nextReleaseRequest(0xfffffffeU, true);
  CHECK(wrapped != 0xfffffffeU);
  CHECK(releaseRequestActive(wrapped));
}

class MemoryFileStore final : public FileStore {
 public:
  ResultCode read(const char* path, EncodedRecording& out) const override {
    if (!failReadPath.empty() && failReadPath == path) {
      failReadPath.clear();
      return failReadResult;
    }
    const auto found = files.find(path);
    if (found == files.end()) return ResultCode::EmptyRecording;
    out = found->second;
    return ResultCode::Ok;
  }

  ResultCode write(const char* path, const std::uint8_t* data,
                   std::size_t size) override {
    ++writeCount;
    if (failNextWrite != ResultCode::Ok) {
      const ResultCode result = failNextWrite;
      failNextWrite = ResultCode::Ok;
      return result;
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

  mutable std::string failReadPath;
  ResultCode failReadResult = ResultCode::ReadFailed;
  ResultCode failNextWrite = ResultCode::Ok;
  std::size_t writeCount = 0;
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
    const auto value = static_cast<std::int16_t>((i / 2) % 2);
    CHECK(events.event({static_cast<std::uint32_t>(i), kind, value}) ==
          ResultCode::Ok);
  }
  CHECK(events.event({static_cast<std::uint32_t>(kMaximumEvents),
                      MechanismKind::Cart, 0}) ==
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
  static ProcessorWorkspace workspace{};
  const Capture outputCapture = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
  });
  CHECK(process(outputCapture, outputRoute, workspace) == ResultCode::Ok);
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
  files.failNextWrite = ResultCode::WriteFailed;
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

  const ResultCode writeFailures[] = {
      ResultCode::WriteFailed, ResultCode::FlushFailed,
      ResultCode::CloseFailed};
  for (const ResultCode failure : writeFailures) {
    files.files.clear();
    CHECK(storage.save(1, RobotIdentity::Small, firstCapture, firstRoute) ==
          ResultCode::Ok);
    files.failNextWrite = failure;
    CHECK(storage.save(1, RobotIdentity::Small, secondCapture, secondRoute) ==
          failure);
    CHECK(storage.load(1, RobotIdentity::Small, loaded) == ResultCode::Ok);
    CHECK(loaded.generation == 1);
    CHECK(loaded.capture.samples[1].y == 6);
  }

  files.files.clear();
  CHECK(storage.save(1, RobotIdentity::Small, firstCapture, firstRoute) ==
        ResultCode::Ok);
  const std::size_t writesBeforeReadFailure = files.writeCount;
  files.failReadPath = "/usd/aon-shadow-slot-1-a.bin";
  files.failReadResult = ResultCode::ReadFailed;
  CHECK(storage.save(1, RobotIdentity::Small, secondCapture, secondRoute) ==
        ResultCode::ReadFailed);
  CHECK(files.writeCount == writesBeforeReadFailure);
  CHECK(storage.load(1, RobotIdentity::Small, loaded) == ResultCode::Ok);
  CHECK(loaded.generation == 1);
  CHECK(loaded.capture.samples[1].y == 6);
}

void playbackServicePolicyTests() {
  SlotSummary valid{};
  valid.result = ResultCode::Ok;
  valid.valid = true;
  CHECK(authorizePlaybackArm(false, RobotIdentity::Small, true, valid) ==
        ResultCode::PlayLocked);
  CHECK(authorizePlaybackArm(true, RobotIdentity::Big, true, valid) ==
        ResultCode::UnsupportedRobot);
  CHECK(authorizePlaybackArm(true, RobotIdentity::Small, false, valid) ==
        ResultCode::UnsafeState);
  SlotSummary wrongRobot{};
  wrongRobot.result = ResultCode::WrongRobot;
  CHECK(authorizePlaybackArm(true, RobotIdentity::Small, true, wrongRobot) ==
        ResultCode::WrongRobot);
  SlotSummary empty{};
  CHECK(authorizePlaybackArm(true, RobotIdentity::Small, true, empty) ==
        ResultCode::EmptyRecording);
  CHECK(authorizePlaybackArm(true, RobotIdentity::Small, true, valid) ==
        ResultCode::Ok);

  MemoryFileStore files;
  Storage storage(files);
  const Capture capture = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(20, 0, 6, 0, Direction::Forward),
  });
  const ProcessedRoute route = process(capture);
  CHECK(storage.save(2, RobotIdentity::Small, capture, route) ==
        ResultCode::Ok);

  ServiceStateMachine state;
  CHECK(state.armPlay(2, 10) == ResultCode::Ok);
  DecodedRecording snapshot{};
  int runnerCalls = 0;
  const PlaybackRunner failingRunner =
      [&](const DecodedRecording& loaded, const PlaybackPolicy& policy) {
        ++runnerCalls;
        CHECK(loaded.capture.robot == RobotIdentity::Small);
        CHECK(policy.authorized && policy.armed);
        CHECK(policy.activeRobot == RobotIdentity::Small);
        return ResultCode::MotionFailure;
      };
  CHECK(dispatchArmedPlayback(state, storage, 2, RobotIdentity::Small,
                              snapshot, failingRunner, 20) ==
        ResultCode::MotionFailure);
  CHECK(runnerCalls == 1);
  CHECK(state.status().mode == ServiceMode::Invalid);
  CHECK(state.status().result == ResultCode::MotionFailure);
  CHECK(dispatchArmedPlayback(state, storage, 2, RobotIdentity::Small,
                              snapshot, failingRunner, 21) ==
        ResultCode::PlayLocked);
  CHECK(runnerCalls == 1);

  ServiceStateMachine missingState;
  CHECK(missingState.armPlay(3, 30) == ResultCode::Ok);
  CHECK(dispatchArmedPlayback(missingState, storage, 3,
                              RobotIdentity::Small, snapshot, failingRunner,
                              31) == ResultCode::EmptyRecording);
  CHECK(missingState.status().mode == ServiceMode::Invalid);
  CHECK(missingState.status().result == ResultCode::EmptyRecording);
  CHECK(dispatchArmedPlayback(missingState, storage, 3,
                              RobotIdentity::Small, snapshot, failingRunner,
                              32) == ResultCode::PlayLocked);
}

void serviceStateTests() {
  CHECK(!confirmationExpired(4999, 5000));
  CHECK(confirmationExpired(5000, 5000));
  CHECK(confirmationExpired(5, std::numeric_limits<std::uint32_t>::max()));

  ServiceStateMachine pendingStart;
  const std::uint32_t pendingRevision = pendingStart.revision();
  CHECK(pendingStart.revalidatePendingStart(pendingRevision, true) ==
        ResultCode::Ok);
  CHECK(pendingStart.revalidatePendingStart(pendingRevision, false) ==
        ResultCode::UnsafeState);
  CHECK(pendingStart.revalidateImmediateStart(true) == ResultCode::Ok);
  CHECK(pendingStart.revalidateImmediateStart(false) ==
        ResultCode::UnsafeState);
  pendingStart.cancel();
  CHECK(pendingStart.revalidatePendingStart(pendingRevision, true) ==
        ResultCode::Cancelled);

  ServiceStateMachine state;
  CHECK(state.beginRecord(0, true) == ResultCode::InvalidSlot);
  CHECK(state.beginRecord(1, false) == ResultCode::Ok);
  const std::uint32_t firstSession = state.recordingSession();
  CHECK(state.acceptsSample(firstSession));
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
  const std::uint32_t secondSession = state.recordingSession();
  CHECK(secondSession != firstSession);
  CHECK(!state.acceptsSample(firstSession));
  CHECK(state.acceptsSample(secondSession));
  CHECK(state.beginProcessing() == ResultCode::Ok);
  CHECK(state.finishSave(ResultCode::Ok, 0, firstSession) ==
        ResultCode::Cancelled);
  CHECK(state.status().mode == ServiceMode::Processing);
  CHECK(state.finishSave(ResultCode::Ok, 0, secondSession) == ResultCode::Ok);
  CHECK(state.status().mode == ServiceMode::Saved);
  CHECK(state.beginRecord(1, false) == ResultCode::UnsafeState);
  CHECK(state.beginRecord(1, true) == ResultCode::Ok);
  const std::uint32_t saveOperation = state.recordingSession();
  CHECK(state.beginProcessing() == ResultCode::Ok);
  state.cancel();
  CHECK(state.finishSave(ResultCode::Ok, 0, saveOperation) ==
        ResultCode::Cancelled);
  CHECK(state.status().mode == ServiceMode::Cancelled);

  CHECK(state.authorizePlay(true, false, true) == ResultCode::PlayLocked);
  CHECK(state.authorizePlay(true, true, false) == ResultCode::EmptyRecording);
  CHECK(state.authorizePlay(true, true, true) == ResultCode::Ok);
  CHECK(state.armPlay(2, 100) == ResultCode::Ok);
  CHECK(state.status().mode == ServiceMode::Armed);
  CHECK(!state.consumeArm(1, 101));
  CHECK(state.consumeArm(2, 102));
  CHECK(state.status().mode == ServiceMode::Playing);
  CHECK(state.status().changedAt == 102);
  CHECK(!state.consumeArm(2, 103));
  CHECK(state.finishPlayback(ResultCode::Ok, 200) == ResultCode::Ok);
  CHECK(state.status().mode == ServiceMode::Finished);
  CHECK(state.status().result == ResultCode::Ok);

  CHECK(state.armPlay(1, 210) == ResultCode::Ok);
  CHECK(state.beginRecord(1, true, 211) == ResultCode::Ok);
  CHECK(!state.consumeArm(1, 212));
  state.cancel(213);
  CHECK(state.armPlay(1, 214) == ResultCode::Ok);
  state.cancel(215);
  CHECK(!state.consumeArm(1, 216));

  ServiceStateMachine failedPlayback;
  CHECK(failedPlayback.armPlay(1, 300) == ResultCode::Ok);
  CHECK(failedPlayback.consumeArm(1, 301));
  CHECK(failedPlayback.finishPlayback(ResultCode::MotionFailure, 302) ==
        ResultCode::MotionFailure);
  CHECK(failedPlayback.status().mode == ServiceMode::Invalid);
  CHECK(failedPlayback.status().result == ResultCode::MotionFailure);
  CHECK(failedPlayback.armPlay(1, 303) == ResultCode::Ok);
  CHECK(failedPlayback.consumeArm(1, 304));
  CHECK(failedPlayback.finishPlayback(ResultCode::Cancelled, 305) ==
        ResultCode::Cancelled);
  CHECK(failedPlayback.status().mode == ServiceMode::Cancelled);
  CHECK(state.armPlay(0) == ResultCode::InvalidSlot);
}

MechanismEvent intakeEvent(std::uint32_t timeMs, IntakeIntent intent) {
  return {timeMs, MechanismKind::IntakeMode,
          static_cast<std::int16_t>(intent)};
}

void mechanismTests() {
  constexpr std::array<IntakeIntent, 9> intakeIntents = {
      IntakeIntent::Idle,         IntakeIntent::Store,
      IntakeIntent::Corridor,     IntakeIntent::Reject,
      IntakeIntent::ScoreBottom,  IntakeIntent::ScoreMiddle,
      IntakeIntent::ScoreTop,     IntakeIntent::SortNormal,
      IntakeIntent::SortInverted,
  };

  const DriveIntent unsaturated = normalizedArcadeDrive(0.6, 0.3);
  CHECK(unsaturated.left == 114);
  CHECK(unsaturated.right == 38);
  const DriveIntent saturated = normalizedArcadeDrive(0.8, 0.4);
  CHECK(saturated.left == 127);
  CHECK(saturated.right == 42);
  const DriveIntent reverse = normalizedArcadeDrive(-0.8, -0.4);
  CHECK(reverse.left == -127);
  CHECK(reverse.right == -42);

  const auto packedDrive = packDriveCommand(-127, 93);
  const auto unpackedDrive = unpackDriveCommand(packedDrive);
  CHECK(unpackedDrive.left == -127);
  CHECK(unpackedDrive.right == 93);
  const auto clampedDrive = unpackDriveCommand(packDriveCommand(-200, 200));
  CHECK(clampedDrive.left == -127);
  CHECK(clampedDrive.right == 127);

  Recorder held;
  CHECK(held.start(RobotIdentity::Small) == ResultCode::Ok);
  CHECK(held.event(intakeEvent(0, IntakeIntent::Store)) == ResultCode::Ok);
  CHECK(held.event({10, MechanismKind::Brooks, 1}) == ResultCode::Ok);
  CHECK(held.event(intakeEvent(20, IntakeIntent::Store)) ==
        ResultCode::DuplicateEvent);
  CHECK(held.event({30, MechanismKind::Brooks, 1}) ==
        ResultCode::DuplicateEvent);
  CHECK(held.event(intakeEvent(40, IntakeIntent::Store)) ==
        ResultCode::DuplicateEvent);
  CHECK(held.event(intakeEvent(60, IntakeIntent::Idle)) == ResultCode::Ok);
  CHECK(held.capture().eventCount == 3);
  CHECK(held.capture().events[2].value ==
        static_cast<std::int16_t>(IntakeIntent::Idle));

  Capture vocabulary = captureAt({
      routeFrame(0, 0, 0, 0, Direction::Forward),
      routeFrame(100, 0, 6, 0, Direction::Forward),
  });
  for (std::size_t index = 0; index < intakeIntents.size(); ++index) {
    vocabulary.events[vocabulary.eventCount++] = intakeEvent(
        static_cast<std::uint32_t>(index), intakeIntents[index]);
  }
  const MechanismEvent smallMechanisms[] = {
      {20, MechanismKind::Lever, 1},
      {21, MechanismKind::ScorerHeight, 1},
      {22, MechanismKind::Cart, 1},
      {23, MechanismKind::Trapdoor, 1},
      {24, MechanismKind::Brooks, 1},
      {25, MechanismKind::Arrow, 1},
  };
  const MechanismEvent bigMechanisms[] = {
      {30, MechanismKind::Cart, 0},
      {31, MechanismKind::Brooks, 0},
      {32, MechanismKind::Sem, 1},
  };
  for (const auto& event : smallMechanisms) {
    vocabulary.events[vocabulary.eventCount++] = event;
  }
  for (const auto& event : bigMechanisms) {
    vocabulary.events[vocabulary.eventCount++] = event;
  }

  const ProcessedRoute route = process(vocabulary);
  static EncodedRecording bytes{};
  CHECK(encode(vocabulary, route, 11, bytes) == ResultCode::Ok);
  static DecodedRecording decoded{};
  CHECK(decode(bytes.data.data(), bytes.size, RobotIdentity::Small, decoded) ==
        ResultCode::Ok);
  CHECK(decoded.capture.eventCount == vocabulary.eventCount);
  for (std::size_t index = 0; index < vocabulary.eventCount; ++index) {
    CHECK(decoded.capture.events[index].kind == vocabulary.events[index].kind);
    CHECK(decoded.capture.events[index].value ==
          vocabulary.events[index].value);
  }

  const IntakeIntent smallIntents[] = {
      IntakeIntent::Idle, IntakeIntent::Store, IntakeIntent::Corridor,
      IntakeIntent::Reject, IntakeIntent::ScoreBottom,
  };
  for (const auto intent : smallIntents) {
    CHECK(validateMechanism(RobotIdentity::Small, intakeEvent(0, intent)) ==
          ResultCode::Ok);
  }
  const IntakeIntent bigIntents[] = {
      IntakeIntent::Idle,        IntakeIntent::Store,
      IntakeIntent::ScoreBottom, IntakeIntent::ScoreMiddle,
      IntakeIntent::ScoreTop,    IntakeIntent::SortNormal,
      IntakeIntent::SortInverted,
  };
  for (const auto intent : bigIntents) {
    CHECK(validateMechanism(RobotIdentity::Big, intakeEvent(0, intent)) ==
          ResultCode::Ok);
  }
  for (const auto& event : smallMechanisms) {
    CHECK(validateMechanism(RobotIdentity::Small, event) == ResultCode::Ok);
  }
  for (const auto& event : bigMechanisms) {
    CHECK(validateMechanism(RobotIdentity::Big, event) == ResultCode::Ok);
  }
  CHECK(validateMechanism(RobotIdentity::Small,
                          {0, MechanismKind::Sem, 1}) ==
        ResultCode::WrongRobot);
  const MechanismKind smallOnlyKinds[] = {
      MechanismKind::Arrow, MechanismKind::ScorerHeight,
      MechanismKind::Trapdoor, MechanismKind::Lever,
  };
  for (const auto kind : smallOnlyKinds) {
    CHECK(validateMechanism(RobotIdentity::Big, {0, kind, 1}) ==
          ResultCode::WrongRobot);
  }
  CHECK(validateMechanism(RobotIdentity::Small,
                          {0, MechanismKind::Cart, 2}) ==
        ResultCode::CorruptFile);
  CHECK(validateMechanism(
            RobotIdentity::Small,
            {0, MechanismKind::IntakeMode,
             static_cast<std::int16_t>(IntakeIntent::SortInverted) + 1}) ==
        ResultCode::CorruptFile);
  CHECK(validateMechanism(
            RobotIdentity::Small,
            {0, static_cast<MechanismKind>(255), 0}) ==
        ResultCode::CorruptFile);

  CHECK(smallIntakeDecision(true, false, false, true) ==
        IntakeIntent::Store);
  CHECK(smallIntakeDecision(true, false, false, false) ==
        IntakeIntent::Corridor);
  CHECK(smallIntakeDecision(false, true, true, true) ==
        IntakeIntent::Reject);
  CHECK(smallIntakeDecision(false, false, true, true) ==
        IntakeIntent::ScoreBottom);
  CHECK(smallIntakeDecision(false, false, false, true) ==
        IntakeIntent::Idle);

  CHECK(bigIntakeDecision(true, false, false, false, true) ==
        IntakeIntent::Store);
  CHECK(bigIntakeDecision(false, true, false, false, true) ==
        IntakeIntent::ScoreBottom);
  CHECK(bigIntakeDecision(true, false, true, false, true) ==
        IntakeIntent::SortNormal);
  CHECK(bigIntakeDecision(false, true, false, true, true) ==
        IntakeIntent::SortInverted);
  CHECK(bigIntakeDecision(true, false, true, false, false) ==
        IntakeIntent::ScoreTop);
  CHECK(bigIntakeDecision(false, true, false, true, false) ==
        IntakeIntent::ScoreMiddle);
  CHECK(bigIntakeDecision(false, false, false, false, true) ==
        IntakeIntent::Idle);

  const auto smallIdle = intakeAdapterPlan(RobotIdentity::Small,
                                            IntakeIntent::Idle);
  CHECK(smallIdle.count == 1);
  CHECK(smallIdle.actions[0] == IntakeAdapterAction::Stop);
  const auto smallStore = intakeAdapterPlan(RobotIdentity::Small,
                                             IntakeIntent::Store);
  CHECK(smallStore.count == 1);
  CHECK(smallStore.actions[0] == IntakeAdapterAction::Store);
  const auto smallCorridor = intakeAdapterPlan(RobotIdentity::Small,
                                                IntakeIntent::Corridor);
  CHECK(smallCorridor.count == 1);
  CHECK(smallCorridor.actions[0] == IntakeAdapterAction::Corridor);
  const auto smallReject = intakeAdapterPlan(RobotIdentity::Small,
                                              IntakeIntent::Reject);
  CHECK(smallReject.count == 1);
  CHECK(smallReject.actions[0] == IntakeAdapterAction::Reject);
  const auto smallBottom = intakeAdapterPlan(RobotIdentity::Small,
                                              IntakeIntent::ScoreBottom);
  CHECK(smallBottom.count == 1);
  CHECK(smallBottom.actions[0] == IntakeAdapterAction::ScoreBottom);

  constexpr std::array<IntakeIntent, 5> directBigIntents = {
      IntakeIntent::Idle, IntakeIntent::Store, IntakeIntent::ScoreBottom,
      IntakeIntent::ScoreMiddle, IntakeIntent::ScoreTop,
  };
  constexpr std::array<IntakeAdapterAction, 5> directBigActions = {
      IntakeAdapterAction::Stop, IntakeAdapterAction::Store,
      IntakeAdapterAction::ScoreBottom, IntakeAdapterAction::ScoreMiddle,
      IntakeAdapterAction::ScoreTop,
  };
  for (std::size_t index = 0; index < directBigIntents.size(); ++index) {
    const auto plan = intakeAdapterPlan(RobotIdentity::Big,
                                        directBigIntents[index]);
    CHECK(plan.count == 2);
    CHECK(plan.actions[0] == IntakeAdapterAction::StopSortAndWait);
    CHECK(plan.actions[1] == directBigActions[index]);
  }
  const auto normalSort = intakeAdapterPlan(RobotIdentity::Big,
                                             IntakeIntent::SortNormal);
  CHECK(normalSort.count == 1);
  CHECK(normalSort.actions[0] == IntakeAdapterAction::SortNormal);
  const auto invertedSort = intakeAdapterPlan(RobotIdentity::Big,
                                               IntakeIntent::SortInverted);
  CHECK(invertedSort.count == 1);
  CHECK(invertedSort.actions[0] == IntakeAdapterAction::SortInverted);

  const MechanismCapturePlan inactiveCapture = planMechanismCapture(
      false, 1200, 1000, MechanismKind::Cart, 1);
  CHECK(!inactiveCapture.record);
  const MechanismCapturePlan activeCapture = planMechanismCapture(
      true, 1200, 1000, MechanismKind::Cart, 1);
  CHECK(activeCapture.record);
  CHECK(activeCapture.event.timeMs == 200);
  CHECK(activeCapture.event.kind == MechanismKind::Cart);
  CHECK(activeCapture.event.value == 1);

  const MechanismAdapterPlan intakePlan = planMechanismAdapter(
      RobotIdentity::Small, intakeEvent(0, IntakeIntent::Store));
  CHECK(intakePlan.result == ResultCode::Ok);
  CHECK(intakePlan.target == MechanismAdapterTarget::Intake);
  CHECK(intakePlan.value ==
        static_cast<std::int16_t>(IntakeIntent::Store));
  constexpr std::array<MechanismAdapterTarget, 6> smallTargets = {
      MechanismAdapterTarget::Lever, MechanismAdapterTarget::ScorerHeight,
      MechanismAdapterTarget::Cart, MechanismAdapterTarget::Trapdoor,
      MechanismAdapterTarget::Brooks, MechanismAdapterTarget::Arrow,
  };
  for (std::size_t index = 0; index < smallTargets.size(); ++index) {
    const MechanismAdapterPlan plan = planMechanismAdapter(
        RobotIdentity::Small, smallMechanisms[index]);
    CHECK(plan.result == ResultCode::Ok);
    CHECK(plan.target == smallTargets[index]);
    CHECK(plan.value == 1);
  }
  constexpr std::array<MechanismAdapterTarget, 3> bigTargets = {
      MechanismAdapterTarget::Cart, MechanismAdapterTarget::Brooks,
      MechanismAdapterTarget::Sem,
  };
  for (std::size_t index = 0; index < bigTargets.size(); ++index) {
    const MechanismAdapterPlan plan = planMechanismAdapter(
        RobotIdentity::Big, bigMechanisms[index]);
    CHECK(plan.result == ResultCode::Ok);
    CHECK(plan.target == bigTargets[index]);
    CHECK(plan.value == bigMechanisms[index].value);
  }
  const MechanismAdapterPlan invalidPlan = planMechanismAdapter(
      RobotIdentity::Small, {0, MechanismKind::Cart, 2});
  CHECK(invalidPlan.result == ResultCode::CorruptFile);
  CHECK(invalidPlan.target == MechanismAdapterTarget::None);
  const MechanismAdapterPlan wrongRobotPlan = planMechanismAdapter(
      RobotIdentity::Small, {0, MechanismKind::Sem, 1});
  CHECK(wrongRobotPlan.result == ResultCode::WrongRobot);
  CHECK(wrongRobotPlan.target == MechanismAdapterTarget::None);
}

int main() {
  playbackPolicyTests();
  playbackSchedulerTests();
  playbackProjectionTests();
  runtimePathSerializationTests();
  releaseRequestTests();
  recorderTests();
  capacityTests();
  processorTests();
  codecTests();
  storageTests();
  playbackServicePolicyTests();
  serviceStateTests();
  mechanismTests();
  std::cout << "shadow auton tests passed\n";
}
