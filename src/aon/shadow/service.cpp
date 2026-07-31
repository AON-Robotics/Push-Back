#include "aon/shadow/service.hpp"

#include "aon/constants.hpp"
#include "aon/globals.hpp"
#include "aon/lemlib/chassis.hpp"
#include "aon/lemlib/drive-io.hpp"
#include "aon/shadow/processor.hpp"
#include "aon/shadow/recorder.hpp"
#include "aon/shadow/mechanisms.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <array>
#include <atomic>
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
  ProcessorWorkspace processorWorkspace{};
  std::array<SlotSummary, kSlotCount> cachedSlots{};
  std::uint32_t startedAt = 0;
  bool ioBusy = false;
  std::uint32_t nextPendingStart = 0;
  std::uint32_t pendingStartOperation = 0;
  std::atomic<bool> recordingActive{false};
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

ResultCode startRecordingLocked(ServiceStorage& serviceData,
                                std::uint8_t slotValue,
                                bool overwriteConfirmed) {
  const ResultCode stateResult = serviceData.state.beginRecord(
      slotValue, overwriteConfirmed, pros::millis());
  if (stateResult != ResultCode::Ok) return stateResult;
  const ResultCode recorderResult = serviceData.recorder.start(kRobotIdentity);
  if (recorderResult != ResultCode::Ok) {
    serviceData.state.cancel(pros::millis());
    return recorderResult;
  }
  serviceData.startedAt = pros::millis();
  serviceData.recordingActive.store(true);
  return ResultCode::Ok;
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
    std::uint32_t pendingOperation = 0;
    std::uint32_t stateRevision = 0;
    {
      Lock lock(serviceData.mutex);
      if (serviceData.ioBusy) return ResultCode::UnsafeState;
      const auto mode = serviceData.state.status().mode;
      if (mode == ServiceMode::Recording || mode == ServiceMode::Processing) {
        return ResultCode::AlreadyRecording;
      }
      serviceData.ioBusy = true;
      ++serviceData.nextPendingStart;
      if (serviceData.nextPendingStart == 0) ++serviceData.nextPendingStart;
      pendingOperation = serviceData.nextPendingStart;
      serviceData.pendingStartOperation = pendingOperation;
      stateRevision = serviceData.state.revision();
    }
    const SlotSummary current = serviceData.storage.inspect(slotValue,
                                                            kRobotIdentity);
    {
      Lock lock(serviceData.mutex);
      if (serviceData.pendingStartOperation != pendingOperation) {
        return ResultCode::Cancelled;
      }
      serviceData.pendingStartOperation = 0;
      serviceData.ioBusy = false;
      serviceData.cachedSlots[slotValue - 1] = current;
      const bool driverControl = !pros::competition::is_disabled() &&
                                 !pros::competition::is_autonomous();
      const ResultCode revalidated = serviceData.state.revalidatePendingStart(
          stateRevision, driverControl);
      if (revalidated != ResultCode::Ok) return revalidated;
      if (current.valid) return ResultCode::UnsafeState;
      if (current.result != ResultCode::EmptyRecording) return current.result;
      return startRecordingLocked(serviceData, slotValue, false);
    }
  }

  Lock lock(serviceData.mutex);
  if (serviceData.ioBusy) return ResultCode::UnsafeState;
  const bool driverControl = !pros::competition::is_disabled() &&
                             !pros::competition::is_autonomous();
  const ResultCode revalidated =
      serviceData.state.revalidateImmediateStart(driverControl);
  if (revalidated != ResultCode::Ok) return revalidated;
  return startRecordingLocked(serviceData, slotValue, true);
}

ResultCode Service::stopAndSave() {
  auto& serviceData = data();
  std::uint8_t targetSlot = 0;
  std::uint32_t operation = 0;
  {
    Lock lock(serviceData.mutex);
    if (serviceData.state.status().mode != ServiceMode::Recording) {
      return ResultCode::NotRecording;
    }
    if (serviceData.ioBusy) return ResultCode::UnsafeState;
    serviceData.ioBusy = true;
    serviceData.recordingActive.store(false);
    operation = serviceData.state.recordingSession();
    const ResultCode stopResult = serviceData.recorder.stop();
    targetSlot = serviceData.state.status().slot;
    serviceData.state.beginProcessing(pros::millis());
    if (stopResult != ResultCode::Ok) {
      serviceData.ioBusy = false;
      serviceData.state.finishSave(stopResult, pros::millis(), operation);
      return stopResult;
    }
    serviceData.captureSnapshot = serviceData.recorder.capture();
  }

  ResultCode saveResult = process(serviceData.captureSnapshot,
                                  serviceData.routeSnapshot,
                                  serviceData.processorWorkspace);
  if (saveResult == ResultCode::Ok) {
    saveResult = serviceData.storage.save(targetSlot, kRobotIdentity,
                                          serviceData.captureSnapshot,
                                          serviceData.routeSnapshot);
  }
  SlotSummary savedSummary{};
  if (saveResult == ResultCode::Ok) {
    savedSummary = serviceData.storage.inspect(targetSlot, kRobotIdentity);
  }
  {
    Lock lock(serviceData.mutex);
    serviceData.ioBusy = false;
    const ResultCode completion = serviceData.state.finishSave(
        saveResult, pros::millis(), operation);
    if (completion == ResultCode::Cancelled) return completion;
    if (saveResult == ResultCode::Ok) {
      serviceData.cachedSlots[targetSlot - 1] = savedSummary;
    }
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
    if (result == ResultCode::Ok) {
      serviceData.cachedSlots[slotValue - 1] = {};
    }
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
    const auto mode = serviceData.state.status().mode;
    if (serviceData.ioBusy || mode == ServiceMode::Recording ||
        mode == ServiceMode::Processing) {
      return serviceData.cachedSlots[slotValue - 1];
    }
    serviceData.ioBusy = true;
  }
  summary = serviceData.storage.inspect(slotValue, kRobotIdentity);
  {
    Lock lock(serviceData.mutex);
    serviceData.ioBusy = false;
    serviceData.cachedSlots[slotValue - 1] = summary;
  }
  return summary;
}

Status Service::status() const {
  auto& serviceData = data();
  Lock lock(serviceData.mutex);
  return serviceData.state.status();
}

void captureMechanism(MechanismKind kind, std::int16_t value) {
  auto& serviceData = data();
  if (!serviceData.recordingActive.load()) return;
  Lock lock(serviceData.mutex);
  const MechanismCapturePlan plan = planMechanismCapture(
      serviceData.state.status().mode == ServiceMode::Recording,
      pros::millis(), serviceData.startedAt, kind, value);
  if (!plan.record) return;

  const ResultCode result = serviceData.recorder.event(plan.event);
  if (result != ResultCode::Ok && result != ResultCode::DuplicateEvent) {
    serviceData.recordingActive.store(false);
    serviceData.state.beginProcessing(pros::millis());
    serviceData.state.finishSave(result, pros::millis());
  }
}

void Service::pollRecorder() {
  auto& serviceData = data();
  std::uint32_t elapsed = 0;
  std::uint32_t session = 0;
  {
    Lock lock(serviceData.mutex);
    if (serviceData.state.status().mode != ServiceMode::Recording) return;
    elapsed = pros::millis() - serviceData.startedAt;
    session = serviceData.state.recordingSession();
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
  if (!serviceData.state.acceptsSample(session)) return;
  const ResultCode result = serviceData.recorder.sample(sample);
  if (result != ResultCode::Ok && result != ResultCode::SampleTooSoon) {
    serviceData.recordingActive.store(false);
    serviceData.state.beginProcessing(pros::millis());
    serviceData.state.finishSave(result, pros::millis());
  }
}

void Service::cancel() {
  auto& serviceData = data();
  serviceData.recordingActive.store(false);
  Lock lock(serviceData.mutex);
  if (serviceData.recorder.isRecording()) serviceData.recorder.stop();
  serviceData.state.cancel(pros::millis());
}

Service& service() {
  static Service instance;
  return instance;
}

}  // namespace aon::shadow
