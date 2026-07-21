#pragma once

#include "aon/shadow/service-state.hpp"
#include "aon/shadow/storage.hpp"

#include <cstdint>

namespace aon::shadow {

class Service {
 public:
  ResultCode beginRecording(std::uint8_t slot, bool overwriteConfirmed);
  ResultCode stopAndSave();
  ResultCode erase(std::uint8_t slot, bool confirmed);
  SlotSummary slot(std::uint8_t slot) const;
  Status status() const;
  void pollRecorder();
  void cancel();

 private:
  Service() = default;
  friend Service& service();
};

Service& service();

}  // namespace aon::shadow
