#pragma once

#include "aon/shadow/types.hpp"

#include <cstdint>

namespace aon::shadow {

enum class ServiceMode : std::uint8_t {
  Idle,
  Recording,
  Processing,
  Saved,
  Invalid,
  Playing,
  Cancelled
};

struct Status {
  ServiceMode mode = ServiceMode::Idle;
  ResultCode result = ResultCode::Ok;
  std::uint8_t slot = 0;
  std::uint32_t changedAt = 0;
};

class ServiceStateMachine {
 public:
  ResultCode beginRecord(std::uint8_t slot, bool overwriteConfirmed,
                         std::uint32_t now = 0);
  ResultCode beginProcessing(std::uint32_t now = 0);
  ResultCode finishSave(ResultCode result, std::uint32_t now = 0);
  ResultCode authorizePlay(bool startConfirmed, bool robotDisabled,
                           bool slotValid) const;
  ResultCode armPlay(std::uint8_t slot, std::uint32_t now = 0);
  bool consumeArm(std::uint8_t slot);
  void cancel(std::uint32_t now = 0);
  Status status() const;

 private:
  Status status_{};
  std::uint8_t armedSlot_ = 0;
};

}  // namespace aon::shadow
