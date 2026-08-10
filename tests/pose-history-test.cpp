#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "aon/localization/pose-history.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

void historyInterpolatesPositionAndWrappedHeading() {
  using namespace aon::localization;
  PoseHistory history;
  CHECK(history.push({{0.0, 0.0, radians(170.0)}, 100}) ==
        PoseHistoryPushResult::Accepted);
  CHECK(history.push({{10.0, 20.0, radians(-170.0)}, 200}) ==
        PoseHistoryPushResult::Accepted);

  TimedPose sample;
  CHECK(history.sampleAt(150, sample) == PoseHistorySampleResult::Interpolated);
  CHECK(std::abs(sample.pose.xInches - 5.0) < 1e-9);
  CHECK(std::abs(sample.pose.yInches - 10.0) < 1e-9);
  CHECK(std::abs(std::abs(sample.pose.headingRadians) - kPi) < 1e-9);
  CHECK(sample.timestampMs == 150);
}

void historyRejectsInvalidOrderingAndOutOfRangeQueries() {
  using namespace aon::localization;
  PoseHistory history;
  TimedPose sample;
  CHECK(history.sampleAt(10, sample) == PoseHistorySampleResult::Empty);
  CHECK(history.push({{0.0, 0.0, 0.0}, 100}) ==
        PoseHistoryPushResult::Accepted);
  CHECK(history.push({{1.0, 0.0, 0.0}, 100}) ==
        PoseHistoryPushResult::OutOfOrder);
  CHECK(history.push({{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                      110}) == PoseHistoryPushResult::Invalid);
  CHECK(history.sampleAt(99, sample) == PoseHistorySampleResult::TooOld);
  CHECK(history.sampleAt(101, sample) == PoseHistorySampleResult::Future);
  CHECK(history.sampleAt(100, sample) == PoseHistorySampleResult::Exact);
}

void capacityEvictsOnlyTheOldestSamples() {
  using namespace aon::localization;
  PoseHistory history;
  for (std::uint32_t timestamp = 1;
       timestamp <= PoseHistory::kCapacity + 2; ++timestamp) {
    CHECK(history.push({{static_cast<double>(timestamp), 0.0, 0.0}, timestamp}) ==
          PoseHistoryPushResult::Accepted);
  }
  CHECK(history.size() == PoseHistory::kCapacity);
  TimedPose sample;
  CHECK(history.sampleAt(2, sample) == PoseHistorySampleResult::TooOld);
  CHECK(history.sampleAt(3, sample) == PoseHistorySampleResult::Exact);
  CHECK(sample.pose.xInches == 3.0);
}

void historyOrdersAndInterpolatesAcrossClockRollover() {
  using namespace aon::localization;
  constexpr std::uint32_t nearWrap =
      std::numeric_limits<std::uint32_t>::max() - 10U;
  PoseHistory history;
  CHECK(history.push({{0.0, 0.0, 0.0}, nearWrap}) ==
        PoseHistoryPushResult::Accepted);
  CHECK(history.push({{16.0, 0.0, 0.0}, 5U}) ==
        PoseHistoryPushResult::Accepted);

  TimedPose sample;
  CHECK(history.sampleAt(0U, sample) ==
        PoseHistorySampleResult::Interpolated);
  CHECK(std::abs(sample.pose.xInches - 11.0) < 1e-9);
}

}  // namespace

int main() {
  historyInterpolatesPositionAndWrappedHeading();
  historyRejectsInvalidOrderingAndOutOfRangeQueries();
  capacityEvictsOnlyTheOldestSamples();
  historyOrdersAndInterpolatesAcrossClockRollover();
  std::cout << "pose history tests passed\n";
  return 0;
}
