#include "aon/drivetrain/legacy-motion.hpp"

#include <memory>

#include "aon/config/robot-config.hpp"
#include "aon/globals.hpp"
#include "aon/lemlib/chassis.hpp"

namespace aon::legacy_motion {

void prepare() {
  if (config::activeRobotConfig().localization.fusedLemLibAuthorized) {
    lemlib_integration::startFusedLocalization();
    return;
  }

  static pros::Mutex taskMutex;
  static std::unique_ptr<pros::Task> odometryTask;

  taskMutex.take();
  const bool startingTask = !odometryTask;
  if (startingTask) {
    odometryTask =
        std::make_unique<pros::Task>([] { drivetrain.runLocalizationLoop(); },
                                    "Legacy Odometry");
  }
  taskMutex.give();

  // Native motion reads odometry immediately, so allow its task one update.
  if (startingTask) pros::delay(50);
}

}  // namespace aon::legacy_motion
