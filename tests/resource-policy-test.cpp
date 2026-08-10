#include "aon/drivetrain/drivetrain.hpp"
#include "aon/communication/pi-protocol.hpp"
#include "aon/core/task-start.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/navigation/path-planner.hpp"
#include "aon/tools/timed-mutex-lock.hpp"
#include "aon/tools/gui/gui-debug.hpp"

#include <cstdlib>
#include <iostream>
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
  CHECK(aon::core::isLiveTaskState(pros::E_TASK_STATE_RUNNING));
  CHECK(aon::core::isLiveTaskState(pros::E_TASK_STATE_READY));
  CHECK(aon::core::isLiveTaskState(pros::E_TASK_STATE_BLOCKED));
  CHECK(aon::core::isLiveTaskState(pros::E_TASK_STATE_SUSPENDED));
  CHECK(!aon::core::isLiveTaskState(pros::E_TASK_STATE_DELETED));
  CHECK(!aon::core::isLiveTaskState(pros::E_TASK_STATE_INVALID));
}

}  // namespace

static_assert(std::has_virtual_destructor_v<aon::Gui>);
static_assert(std::has_virtual_destructor_v<aon::Drivetrain>);
static_assert(!std::is_copy_constructible_v<aon::Gui>);
static_assert(!std::is_move_constructible_v<aon::Gui>);
static_assert(!std::is_copy_constructible_v<aon::Drivetrain>);
static_assert(!std::is_move_constructible_v<aon::Drivetrain>);
static_assert(sizeof(aon::navigation::PathPlanner) <= 16U * 1024U);
static_assert(sizeof(aon::navigation::PathPlanner) >
              sizeof(aon::navigation::PathPlannerConfig));
static_assert(sizeof(aon::communication::FrameParser) <= 512U);
static_assert(sizeof(aon::navigation::DynamicObstacleMap) <= 4U * 1024U);

int main() {
  timedMutexLockReleasesOnlyAnOwnedLock();
  taskStatePolicyRecognizesOnlyLiveProsStates();
}
