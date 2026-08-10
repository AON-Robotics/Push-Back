#pragma once

#include <cstdint>

namespace aon {

/**
 * @brief Acquires a borrowed mutex for at most `timeoutMs` milliseconds.
 * Destruction releases it exactly once when acquisition succeeded.
 */
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

  /** @brief True only while this guard owns the borrowed mutex. */
  [[nodiscard]] bool ownsLock() const noexcept { return owns_; }

 private:
  Mutex& mutex_;
  bool owns_;
};

}  // namespace aon
