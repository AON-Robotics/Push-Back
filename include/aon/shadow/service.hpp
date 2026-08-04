#pragma once

#include "aon/shadow/player.hpp"
#include "aon/shadow/service-state.hpp"
#include "aon/shadow/storage.hpp"

#include <cstdint>
#include <functional>

namespace aon::shadow {

/**
 * @brief Process-lifetime coordinator for recording, storage, and playback.
 *
 * Slot numbers are one-based in [1, kSlotCount]. Methods return ResultCode for
 * invalid state, authorization, SD, codec, and motion failures. Recording is
 * polled from the dedicated recorder task; callers must not invoke mutating
 * operations concurrently unless the service state policy explicitly permits
 * that transition.
 */
class Service {
 public:
  /** Starts a capture in slot; existing data requires overwrite confirmation. */
  ResultCode beginRecording(std::uint8_t slot, bool overwriteConfirmed);
  /** Stops the active capture, processes it, and verifies the persisted copy. */
  ResultCode stopAndSave();
  /** Deletes both redundant files for slot when confirmed. */
  ResultCode erase(std::uint8_t slot, bool confirmed);
  /** Returns a by-value summary for a one-based slot. */
  SlotSummary slot(std::uint8_t slot) const;
  /** Returns the current service state by value. */
  Status status() const;
  /** Arms a valid slot for one playback while the robot is disabled. */
  ResultCode armPlayback(std::uint8_t slot, bool startConfirmed,
                         bool robotDisabled);
  /** Consumes the current arm and runs playback synchronously. */
  ResultCode runArmedPlayback();
  /** Invalidates any pending playback arm without deleting recordings. */
  void clearPlaybackArm();
  /** Captures one 20 ms sample when recording is active. */
  void pollRecorder();
  /** Cancels recording/playback and commands the hardware adapters to stop. */
  void cancel();

 private:
  Service() = default;
  friend Service& service();
};

/** Injectable synchronous playback boundary used by service tests. */
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

/** Returns the non-owning, process-lifetime Shadow service. */
Service& service();

}  // namespace aon::shadow
