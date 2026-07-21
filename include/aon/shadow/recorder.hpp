#pragma once

#include "aon/shadow/types.hpp"

namespace aon::shadow {

struct RecorderLimits {
  std::uint32_t invalidPoseSamples = 3;
  std::uint32_t poseJumpSamples = 2;
  float maximumPoseJumpInches = 8.0F;
  float maximumHeadingJumpDegrees = 45.0F;
};

class Recorder {
 public:
  explicit Recorder(RecorderLimits limits = {});
  ResultCode start(RobotIdentity robot);
  ResultCode sample(const RawSample& sample);
  ResultCode event(const MechanismEvent& event);
  ResultCode stop();
  bool isRecording() const;
  ResultCode result() const;
  const Capture& capture() const;

 private:
  Capture capture_{};
  ResultCode result_ = ResultCode::NotRecording;
  bool recording_ = false;
  std::uint32_t invalidPoseCount_ = 0;
  std::uint32_t poseJumpCount_ = 0;
  RecorderLimits limits_{};
};

}  // namespace aon::shadow
