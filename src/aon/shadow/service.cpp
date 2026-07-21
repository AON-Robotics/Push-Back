#include "aon/shadow/service.hpp"

#include "aon/constants.hpp"
#include "aon/globals.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/lemlib/drive-io.hpp"
#include "aon/shadow/processor.hpp"
#include "aon/shadow/recorder.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace aon::shadow {
namespace {

constexpr RobotIdentity kRobotIdentity =
#if USING_BIG_ROBOT
    RobotIdentity::Big;
#else
    RobotIdentity::Small;
#endif

class Lock {
 public:
  explicit Lock(pros::Mutex& mutex) : mutex_(mutex) { mutex_.take(); }
  ~Lock() { mutex_.give(); }
  Lock(const Lock&) = delete;
  Lock& operator=(const Lock&) = delete;

 private:
  pros::Mutex& mutex_;
};

struct ServiceStorage {
  pros::Mutex mutex;
  SdFileStore files;
  Storage storage{files};
  ServiceStateMachine state;
  Recorder recorder;
  Capture captureSnapshot{};
  ProcessedRoute routeSnapshot{};
  std::uint32_t startedAt = 0;
  bool ioBusy = false;
};

ServiceStorage& data() {
  static ServiceStorage storage;
  return storage;
}

std::int8_t axisValue(pros::controller_analog_e_t axis) {
  return static_cast<std::int8_t>(
      std::clamp(mainController.get_analog(axis), -127L, 127L));
}

Direction directionFor(std::int8_t left, std::int8_t right) {
  const int average = static_cast<int>(left) + static_cast<int>(right);
  if (average > 10) return Direction::Forward;
  if (average < -10) return Direction::Reverse;
  return Direction::Stopped;
}

}  // namespace

ResultCode Service::beginRecording(std::uint8_t slotValue,
                                   bool overwriteConfirmed) {
  if (pros::competition::is_disabled() ||
      pros::competition::is_autonomous()) {
    return ResultCode::UnsafeState;
  }
  auto& serviceData = data();
  if (slotValue < 1 || slotValue > kSlotCount) return ResultCode::InvalidSlot;

  if (!overwriteConfirmed) {
    {
      Lock lock(serviceData.mutex);
      if (serviceData.ioBusy) return ResultCode::UnsafeState;
      serviceData.ioBusy = true;
    }
    const SlotSummary current = serviceData.storage.inspect(slotValue,
                                                            kRobotIdentity);
    {
      Lock lock(serviceData.mutex);
      serviceData.ioBusy = false;
    }
    if (current.valid) return ResultCode::UnsafeState;
    if (current.result != ResultCode::EmptyRecording) return current.result;
  }

  Lock lock(serviceData.mutex);
  const ResultCode stateResult = serviceData.state.beginRecord(
      slotValue, overwriteConfirmed, pros::millis());
  if (stateResult != ResultCode::Ok) return stateResult;
  const ResultCode recorderResult = serviceData.recorder.start(kRobotIdentity);
  if (recorderResult != ResultCode::Ok) {
    serviceData.state.cancel(pros::millis());
    return recorderResult;
  }
  serviceData.startedAt = pros::millis();
  return ResultCode::Ok;
}

ResultCode Service::stopAndSave() {
  auto& serviceData = data();
  std::uint8_t targetSlot = 0;
  {
    Lock lock(serviceData.mutex);
    if (serviceData.state.status().mode != ServiceMode::Recording) {
      return ResultCode::NotRecording;
    }
    const ResultCode stopResult = serviceData.recorder.stop();
    targetSlot = serviceData.state.status().slot;
    serviceData.state.beginProcessing(pros::millis());
    if (stopResult != ResultCode::Ok) {
      serviceData.state.finishSave(stopResult, pros::millis());
      return stopResult;
    }
    serviceData.captureSnapshot = serviceData.recorder.capture();
    if (serviceData.ioBusy) {
      serviceData.state.finishSave(ResultCode::UnsafeState, pros::millis());
      return ResultCode::UnsafeState;
    }
    serviceData.ioBusy = true;
  }

  ResultCode saveResult =
      process(serviceData.captureSnapshot, serviceData.routeSnapshot);
  if (saveResult == ResultCode::Ok) {
    saveResult = serviceData.storage.save(targetSlot, kRobotIdentity,
                                          serviceData.captureSnapshot,
                                          serviceData.routeSnapshot);
  }
  {
    Lock lock(serviceData.mutex);
    serviceData.ioBusy = false;
    serviceData.state.finishSave(saveResult, pros::millis());
  }
  return saveResult;
}

ResultCode Service::erase(std::uint8_t slotValue, bool confirmed) {
  if (slotValue < 1 || slotValue > kSlotCount) return ResultCode::InvalidSlot;
  if (!confirmed) return ResultCode::UnsafeState;
  auto& serviceData = data();
  {
    Lock lock(serviceData.mutex);
    const auto mode = serviceData.state.status().mode;
    if (serviceData.ioBusy || mode == ServiceMode::Recording ||
        mode == ServiceMode::Processing) {
      return ResultCode::UnsafeState;
    }
    serviceData.ioBusy = true;
  }
  const ResultCode result = serviceData.storage.erase(slotValue);
  {
    Lock lock(serviceData.mutex);
    serviceData.ioBusy = false;
  }
  return result;
}

SlotSummary Service::slot(std::uint8_t slotValue) const {
  SlotSummary summary{};
  if (slotValue < 1 || slotValue > kSlotCount) {
    summary.result = ResultCode::InvalidSlot;
    return summary;
  }
  auto& serviceData = data();
  {
    Lock lock(serviceData.mutex);
    if (serviceData.ioBusy) {
      summary.result = ResultCode::UnsafeState;
      return summary;
    }
    serviceData.ioBusy = true;
  }
  summary = serviceData.storage.inspect(slotValue, kRobotIdentity);
  {
    Lock lock(serviceData.mutex);
    serviceData.ioBusy = false;
  }
  return summary;
}

Status Service::status() const {
  auto& serviceData = data();
  Lock lock(serviceData.mutex);
  return serviceData.state.status();
}

void Service::pollRecorder() {
  auto& serviceData = data();
  std::uint32_t elapsed = 0;
  {
    Lock lock(serviceData.mutex);
    if (serviceData.state.status().mode != ServiceMode::Recording) return;
    elapsed = pros::millis() - serviceData.startedAt;
  }
  if (pros::competition::is_disabled() ||
      pros::competition::is_autonomous() || elapsed >= kMaximumDurationMs) {
    stopAndSave();
    return;
  }

  const lemlib::Pose pose = aon::lemlib_integration::chassis().getPose();
  const auto command = aon::lemlib_integration::effectiveDriveCommand();
  RawSample sample{};
  sample.timeMs = elapsed;
  sample.x = static_cast<float>(pose.x);
  sample.y = static_cast<float>(pose.y);
  sample.heading = static_cast<float>(pose.theta);
  sample.leftX = axisValue(ANALOG_LEFT_X);
  sample.leftY = axisValue(ANALOG_LEFT_Y);
  sample.rightX = axisValue(ANALOG_RIGHT_X);
  sample.rightY = axisValue(ANALOG_RIGHT_Y);
  sample.leftCommand = command.left;
  sample.rightCommand = command.right;
  sample.direction = directionFor(command.left, command.right);
  sample.poseValid = std::isfinite(pose.x) && std::isfinite(pose.y) &&
                     std::isfinite(pose.theta);

  Lock lock(serviceData.mutex);
  if (serviceData.state.status().mode != ServiceMode::Recording) return;
  const ResultCode result = serviceData.recorder.sample(sample);
  if (result != ResultCode::Ok && result != ResultCode::SampleTooSoon) {
    serviceData.state.beginProcessing(pros::millis());
    serviceData.state.finishSave(result, pros::millis());
  }
}

void Service::cancel() {
  auto& serviceData = data();
  Lock lock(serviceData.mutex);
  if (serviceData.recorder.isRecording()) serviceData.recorder.stop();
  serviceData.state.cancel(pros::millis());
}

Service& service() {
  static Service instance;
  return instance;
}

}  // namespace aon::shadow
