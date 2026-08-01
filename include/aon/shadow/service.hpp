#pragma once

#include "aon/shadow/player.hpp"
#include "aon/shadow/service-state.hpp"
#include "aon/shadow/storage.hpp"

#include <cstdint>
#include <functional>

namespace aon::shadow {

class Service {
 public:
  ResultCode beginRecording(std::uint8_t slot, bool overwriteConfirmed);
  ResultCode stopAndSave();
  ResultCode erase(std::uint8_t slot, bool confirmed);
  SlotSummary slot(std::uint8_t slot) const;
  Status status() const;
  ResultCode armPlayback(std::uint8_t slot, bool startConfirmed,
                         bool robotDisabled);
  ResultCode runArmedPlayback();
  void clearPlaybackArm();
  void pollRecorder();
  void cancel();

 private:
  Service() = default;
  friend Service& service();
};

using PlaybackRunner = std::function<ResultCode(
    const DecodedRecording&, const PlaybackPolicy&)>;

ResultCode loadAndRunPlayback(Storage& storage, std::uint8_t slot,
                              RobotIdentity robot,
                              DecodedRecording& snapshot,
                              const PlaybackRunner& runner);

ResultCode dispatchArmedPlayback(ServiceStateMachine& state, Storage& storage,
                                 std::uint8_t slot, RobotIdentity robot,
                                 DecodedRecording& snapshot,
                                 const PlaybackRunner& runner,
                                 std::uint32_t now = 0);

ResultCode authorizePlaybackArm(bool authorized, RobotIdentity activeRobot,
                                bool robotDisabled,
                                const SlotSummary& summary);
ResultCode playbackEligibility(bool authorized, RobotIdentity activeRobot,
                               bool robotDisabled,
                               const SlotSummary& summary, ServiceMode mode);

Service& service();

}  // namespace aon::shadow
