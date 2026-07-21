#include "aon/shadow/recorder.hpp"

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

int main() {
  recorderTests();
  capacityTests();
  std::cout << "shadow auton tests passed\n";
}
