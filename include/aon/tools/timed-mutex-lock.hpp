#pragma once

#include <cstdint>

namespace aon {

/** Owns a mutex only when acquisition succeeds before the timeout. */
template <typename Mutex>
class TimedMutexLock {
 public:
  TimedMutexLock(Mutex& mutex, std::uint32_t timeoutMs) noexcept
      : mutex_(mutex), owns_(mutex_.take(timeoutMs)) {}

  ~TimedMutexLock() noexcept {
    if (owns_) (void)mutex_.give();
  }

  TimedMutexLock(const TimedMutexLock&) = delete;
  TimedMutexLock& operator=(const TimedMutexLock&) = delete;
  TimedMutexLock(TimedMutexLock&&) = delete;
  TimedMutexLock& operator=(TimedMutexLock&&) = delete;

  [[nodiscard]] bool ownsLock() const noexcept { return owns_; }

 private:
  Mutex& mutex_;
  bool owns_;
};

}  // namespace aon
