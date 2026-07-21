#include "aon/shadow/service-state.hpp"

namespace aon::shadow {
namespace {

bool validSlot(std::uint8_t slot) { return slot >= 1 && slot <= kSlotCount; }

}  // namespace

ResultCode ServiceStateMachine::beginRecord(std::uint8_t slot,
                                            bool overwriteConfirmed,
                                            std::uint32_t now) {
  if (!validSlot(slot)) return ResultCode::InvalidSlot;
  if (status_.mode == ServiceMode::Recording ||
      status_.mode == ServiceMode::Processing) {
    return ResultCode::AlreadyRecording;
  }
  if (status_.mode == ServiceMode::Saved && status_.slot == slot &&
      !overwriteConfirmed) {
    return ResultCode::UnsafeState;
  }
  status_ = {ServiceMode::Recording, ResultCode::Ok, slot, now};
  armedSlot_ = 0;
  return ResultCode::Ok;
}

ResultCode ServiceStateMachine::beginProcessing(std::uint32_t now) {
  if (status_.mode != ServiceMode::Recording) {
    return ResultCode::NotRecording;
  }
  status_.mode = ServiceMode::Processing;
  status_.result = ResultCode::Ok;
  status_.changedAt = now;
  return ResultCode::Ok;
}

ResultCode ServiceStateMachine::finishSave(ResultCode result,
                                           std::uint32_t now) {
  if (status_.mode != ServiceMode::Processing) {
    return ResultCode::NotRecording;
  }
  status_.mode = result == ResultCode::Ok ? ServiceMode::Saved
                                          : ServiceMode::Invalid;
  status_.result = result;
  status_.changedAt = now;
  return result;
}

ResultCode ServiceStateMachine::authorizePlay(bool startConfirmed,
                                              bool robotDisabled,
                                              bool slotValid) const {
  if (!startConfirmed || !robotDisabled ||
      status_.mode == ServiceMode::Recording ||
      status_.mode == ServiceMode::Processing) {
    return ResultCode::PlayLocked;
  }
  return slotValid ? ResultCode::Ok : ResultCode::EmptyRecording;
}

ResultCode ServiceStateMachine::armPlay(std::uint8_t slot,
                                        std::uint32_t now) {
  if (!validSlot(slot)) return ResultCode::InvalidSlot;
  if (status_.mode == ServiceMode::Recording ||
      status_.mode == ServiceMode::Processing) {
    return ResultCode::PlayLocked;
  }
  armedSlot_ = slot;
  status_ = {ServiceMode::Playing, ResultCode::Ok, slot, now};
  return ResultCode::Ok;
}

bool ServiceStateMachine::consumeArm(std::uint8_t slot) {
  if (armedSlot_ == 0 || armedSlot_ != slot) return false;
  armedSlot_ = 0;
  return true;
}

void ServiceStateMachine::cancel(std::uint32_t now) {
  armedSlot_ = 0;
  status_ = {ServiceMode::Cancelled, ResultCode::Cancelled, status_.slot, now};
}

Status ServiceStateMachine::status() const { return status_; }

}  // namespace aon::shadow
