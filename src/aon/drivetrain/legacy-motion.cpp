#include "aon/drivetrain/legacy-motion.hpp"

#include <memory>
#include <new>

#include "aon/config/robot-config.hpp"
#include "aon/globals.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/tools/timed-mutex-lock.hpp"

namespace aon::legacy_motion {

core::TaskStartResult prepare() {
  if (config::activeRobotConfig().localization.fusedLemLibAuthorized) {
    return lemlib_integration::startFusedLocalization();
  }

  static pros::Mutex taskMutex;
  static std::unique_ptr<pros::Task> odometryTask;

  {
    TimedMutexLock lock(taskMutex, 2U);
    if (!lock.ownsLock()) return core::TaskStartResult::Failed;
    if (odometryTask && core::isLiveTaskState(odometryTask->get_state())) {
      return core::TaskStartResult::AlreadyRunning;
    }
    odometryTask.reset();
    try {
      auto candidate = std::make_unique<pros::Task>(
          [] { drivetrain.runLocalizationLoop(); }, "Legacy Odometry");
      if (!core::isLiveTaskState(candidate->get_state())) {
        return core::TaskStartResult::Failed;
      }
      odometryTask = std::move(candidate);
    } catch (const std::bad_alloc&) {
      return core::TaskStartResult::Failed;
    }
  }

  // Native motion reads odometry immediately, so allow its task one update.
  pros::delay(50);
  return core::TaskStartResult::Started;
}

}  // namespace aon::legacy_motion
