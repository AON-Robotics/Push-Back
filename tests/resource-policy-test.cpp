#include "aon/communication/pi-protocol.hpp"
#include "aon/config/localization-config.hpp"
#include "aon/core/task-start.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/navigation/path-planner.hpp"
#include "aon/odometry/diagnostics.hpp"
#include "aon/odometry/ekf.hpp"
#include "aon/odometry/sensor-measurements.hpp"
#include "aon/time/monotonic.hpp"
#include "aon/tools/timed-mutex-lock.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <type_traits>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

struct FakeMutex {
  bool result = false;
  std::uint32_t timeout = 0;
  int takeCalls = 0;
  int giveCalls = 0;

  bool take(std::uint32_t requestedTimeout) noexcept {
    timeout = requestedTimeout;
    ++takeCalls;
    return result;
  }

  bool give() noexcept {
    ++giveCalls;
    return true;
  }
};

using FakeLock = aon::TimedMutexLock<FakeMutex>;

static_assert(!std::is_copy_constructible_v<FakeLock>);
static_assert(!std::is_move_constructible_v<FakeLock>);

void timedMutexLockReleasesOnlyAnOwnedLock() {
  FakeMutex acquired{true};
  {
    FakeLock lock(acquired, 2U);
    CHECK(lock.ownsLock());
    CHECK(acquired.timeout == 2U);
  }
  CHECK(acquired.takeCalls == 1);
  CHECK(acquired.giveCalls == 1);

  FakeMutex rejected{false};
  {
    FakeLock lock(rejected, 7U);
    CHECK(!lock.ownsLock());
    CHECK(rejected.timeout == 7U);
  }
  CHECK(rejected.takeCalls == 1);
  CHECK(rejected.giveCalls == 0);
}

void taskStatePolicyRecognizesOnlyLiveProsStates() {
  using aon::core::TaskState;
  CHECK(aon::core::isLiveTaskState(
      static_cast<std::uint32_t>(TaskState::Running)));
  CHECK(aon::core::isLiveTaskState(
      static_cast<std::uint32_t>(TaskState::Ready)));
  CHECK(aon::core::isLiveTaskState(
      static_cast<std::uint32_t>(TaskState::Blocked)));
  CHECK(aon::core::isLiveTaskState(
      static_cast<std::uint32_t>(TaskState::Suspended)));
  CHECK(!aon::core::isLiveTaskState(
      static_cast<std::uint32_t>(TaskState::Deleted)));
  CHECK(!aon::core::isLiveTaskState(
      static_cast<std::uint32_t>(TaskState::Invalid)));
}

}  // namespace

static_assert(sizeof(aon::navigation::PathPlanner) <= 16U * 1024U);
static_assert(sizeof(aon::navigation::PathPlanner) >
              sizeof(aon::navigation::PathPlannerConfig));
static_assert(sizeof(aon::communication::FrameParser) <= 512U);
static_assert(sizeof(aon::navigation::DynamicObstacleMap) <= 4U * 1024U);
static_assert(sizeof(aon::localization::LocalizationDiagnostics) <= 1024U);
static_assert(aon::time::validInterval(1U));
static_assert(!aon::time::validInterval(
    static_cast<std::uint32_t>(
        std::numeric_limits<std::int32_t>::max())));

constexpr aon::localization::WheelDistances kDefaultWheels;
constexpr aon::localization::GpsMeasurement kDefaultGps;
constexpr aon::localization::EkfConfig kDefaultEkfConfig;
constexpr aon::localization::EstimatorPose kDefaultEstimatorPose;
constexpr aon::config::LocalizationConfig kDefaultLocalizationConfig;
static_assert(kDefaultWheels.leftInches == 0.0);
static_assert(!kDefaultWheels.leftValid);
static_assert(kDefaultGps.timestampMs == 0U);
static_assert(!kDefaultGps.fresh);
static_assert(kDefaultEkfConfig.initialPositionVariance == 0.0);
static_assert(kDefaultEstimatorPose.headingRadians == 0.0);
static_assert(!kDefaultLocalizationConfig.gps.enabled);
static_assert(!kDefaultLocalizationConfig.fusedLemLibAuthorized);

int main() {
  timedMutexLockReleasesOnlyAnOwnedLock();
  taskStatePolicyRecognizesOnlyLiveProsStates();
}
