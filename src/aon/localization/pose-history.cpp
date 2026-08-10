#include "aon/localization/pose-history.hpp"
#include "aon/time/monotonic.hpp"

#include <cmath>

namespace aon::localization {

PoseHistoryPushResult PoseHistory::push(TimedPose sample) noexcept {
  if (!std::isfinite(sample.pose.xInches) ||
      !std::isfinite(sample.pose.yInches) ||
      !std::isfinite(sample.pose.headingRadians)) {
    return PoseHistoryPushResult::Invalid;
  }
  if (size_ != 0 && !time::strictlyAfter(
                        sample.timestampMs,
                        logicalAt(size_ - 1).timestampMs)) {
    return PoseHistoryPushResult::OutOfOrder;
  }
  sample.pose.headingRadians = wrapRadians(sample.pose.headingRadians);
  samples_[next_] = sample;
  next_ = (next_ + 1) % kCapacity;
  if (size_ < kCapacity) ++size_;
  return PoseHistoryPushResult::Accepted;
}

PoseHistorySampleResult PoseHistory::sampleAt(
    std::uint32_t timestampMs, TimedPose& sample) const noexcept {
  if (size_ == 0) return PoseHistorySampleResult::Empty;
  const TimedPose& oldest = logicalAt(0);
  const TimedPose& newest = logicalAt(size_ - 1);
  if (timestampMs != oldest.timestampMs &&
      time::strictlyAfter(oldest.timestampMs, timestampMs)) {
    return PoseHistorySampleResult::TooOld;
  }
  if (timestampMs != newest.timestampMs &&
      time::strictlyAfter(timestampMs, newest.timestampMs)) {
    return PoseHistorySampleResult::Future;
  }

  for (std::size_t index = 0; index < size_; ++index) {
    const TimedPose& upper = logicalAt(index);
    if (timestampMs == upper.timestampMs) {
      sample = upper;
      return PoseHistorySampleResult::Exact;
    }
    const std::uint32_t queryOffset =
        time::elapsed(timestampMs, oldest.timestampMs);
    const std::uint32_t upperOffset =
        time::elapsed(upper.timestampMs, oldest.timestampMs);
    if (queryOffset < upperOffset) {
      const TimedPose& lower = logicalAt(index - 1);
      const double span = static_cast<double>(
          time::elapsed(upper.timestampMs, lower.timestampMs));
      const double ratio = static_cast<double>(
                               time::elapsed(timestampMs,
                                             lower.timestampMs)) /
                           span;
      sample.pose.xInches = lower.pose.xInches +
                            (upper.pose.xInches - lower.pose.xInches) * ratio;
      sample.pose.yInches = lower.pose.yInches +
                            (upper.pose.yInches - lower.pose.yInches) * ratio;
      sample.pose.headingRadians = wrapRadians(
          lower.pose.headingRadians +
          shortestAngleDelta(lower.pose.headingRadians,
                             upper.pose.headingRadians) * ratio);
      sample.timestampMs = timestampMs;
      return PoseHistorySampleResult::Interpolated;
    }
  }
  return PoseHistorySampleResult::Future;
}

void PoseHistory::clear() noexcept {
  size_ = 0;
  next_ = 0;
}

std::size_t PoseHistory::size() const noexcept { return size_; }

const TimedPose& PoseHistory::logicalAt(std::size_t index) const noexcept {
  const std::size_t oldest = size_ == kCapacity ? next_ : 0;
  return samples_[(oldest + index) % kCapacity];
}

}  // namespace aon::localization
