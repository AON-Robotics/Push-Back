#include "aon/shadow/recorder.hpp"

#include <algorithm>
#include <cmath>

namespace aon::shadow {
namespace {

bool hasValidPose(const RawSample& sample) {
  return sample.poseValid && std::isfinite(sample.x) &&
         std::isfinite(sample.y) && std::isfinite(sample.heading);
}

float headingDifference(float first, float second) {
  float difference = std::fmod(std::fabs(first - second), 360.0F);
  return std::min(difference, 360.0F - difference);
}

}  // namespace

Recorder::Recorder(RecorderLimits limits) : limits_(limits) {}

ResultCode Recorder::start(RobotIdentity robot) {
  if (recording_) return ResultCode::AlreadyRecording;

  capture_ = {};
  capture_.robot = robot;
  result_ = ResultCode::Ok;
  recording_ = true;
  invalidPoseCount_ = 0;
  poseJumpCount_ = 0;
  return result_;
}

ResultCode Recorder::sample(const RawSample& sampleValue) {
  if (!recording_) return ResultCode::NotRecording;

  if (capture_.sampleCount > 0) {
    const auto& previous = capture_.samples[capture_.sampleCount - 1];
    if (sampleValue.timeMs < previous.timeMs ||
        sampleValue.timeMs - previous.timeMs < kSamplePeriodMs) {
      return ResultCode::SampleTooSoon;
    }
  }

  if (sampleValue.timeMs > kMaximumDurationMs ||
      capture_.sampleCount == kMaximumSamples) {
    recording_ = false;
    result_ = ResultCode::CapacityReached;
    return result_;
  }

  const bool validPose = hasValidPose(sampleValue);
  bool poseJump = false;
  if (validPose && capture_.sampleCount > 0) {
    const auto& previous = capture_.samples[capture_.sampleCount - 1];
    if (hasValidPose(previous)) {
      const float xDifference = sampleValue.x - previous.x;
      const float yDifference = sampleValue.y - previous.y;
      poseJump = std::hypot(xDifference, yDifference) >
                     limits_.maximumPoseJumpInches ||
                 headingDifference(sampleValue.heading, previous.heading) >
                     limits_.maximumHeadingJumpDegrees;
    }
  }

  capture_.samples[capture_.sampleCount++] = sampleValue;
  capture_.durationMs = std::max(capture_.durationMs, sampleValue.timeMs);

  if (!validPose) {
    ++invalidPoseCount_;
    poseJumpCount_ = 0;
    if (invalidPoseCount_ >= limits_.invalidPoseSamples) {
      recording_ = false;
      result_ = ResultCode::InvalidPose;
      return result_;
    }
    return ResultCode::Ok;
  }

  invalidPoseCount_ = 0;
  if (poseJump) {
    ++poseJumpCount_;
    if (poseJumpCount_ >= limits_.poseJumpSamples) {
      recording_ = false;
      result_ = ResultCode::PoseJump;
      return result_;
    }
  } else {
    poseJumpCount_ = 0;
  }
  return ResultCode::Ok;
}

ResultCode Recorder::event(const MechanismEvent& eventValue) {
  if (!recording_) return ResultCode::NotRecording;

  for (std::size_t index = capture_.eventCount; index > 0; --index) {
    const auto& previous = capture_.events[index - 1];
    if (eventValue.kind == previous.kind) {
      if (eventValue.value == previous.value) {
        return ResultCode::DuplicateEvent;
      }
      break;
    }
  }

  if (eventValue.timeMs > kMaximumDurationMs ||
      capture_.eventCount == kMaximumEvents) {
    recording_ = false;
    result_ = ResultCode::CapacityReached;
    return result_;
  }

  capture_.events[capture_.eventCount++] = eventValue;
  capture_.durationMs = std::max(capture_.durationMs, eventValue.timeMs);
  return ResultCode::Ok;
}

ResultCode Recorder::stop() {
  if (!recording_) return ResultCode::NotRecording;

  recording_ = false;
  if (capture_.sampleCount == 0 && capture_.eventCount == 0) {
    result_ = ResultCode::EmptyRecording;
    return result_;
  }

  result_ = ResultCode::Ok;
  return result_;
}

bool Recorder::isRecording() const { return recording_; }

ResultCode Recorder::result() const { return result_; }

const Capture& Recorder::capture() const { return capture_; }

}  // namespace aon::shadow
