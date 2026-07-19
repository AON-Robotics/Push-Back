# Motor-Encoder Odometry Fallback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a conservative, latched motor-encoder fallback that can be selected on the brain before autonomous or activated after a confirmed LemLib tracking failure, then retry one supported motion at reduced speed.

**Architecture:** Keep route code behind `aon::auton::Actions`. Split pure sample evaluation and fallback geometry from PROS-facing drive I/O so the safety decisions can be host-tested. The LemLib integration continues to construct every drivetrain device once; encoder control begins only after LemLib cancellation and a confirmed stop.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, GNU Make/ARM toolchain, MinGW g++ host tests, Git.

## Global Constraints

- Preserve native Kevin Loader and Kevin Park and their lazy legacy odometry path.
- Keep `AUTO FALLBACK` as the non-persisted startup default; `FORCE ENCODERS` must be selected again after every brain restart.
- Automatic fallback latches until brain restart and never returns to tracking during that process lifetime.
- Automatic detection must use direct invalid data, impossible jumps, or tracking-versus-motor disagreement; target error, wheel slip, blocking, and ordinary timeout are not odometry faults.
- Cancel LemLib, command zero output, and wait for drivetrain settling before encoder control starts.
- Retry an interrupted supported action at most once and cap fallback output at 60 percent of the requested maximum.
- Fail safely for `followPath`; do not guess a replacement path.
- A failed required action must prevent dependent route actions and mark the routine failed.
- Do not edit vendored PROS, LemLib, LVGL, fmt, or JSON sources.
- Complete the existing migration Task 1 physical gate before enabling automatic fallback in a competition route.
- Clean-build both robot configurations at each robot-code checkpoint and restore `USING_BIG_ROBOT false` before committing.

## Execution Status

- Tasks 1 through 6 are implemented through checkpoint `f316e98`.
- Checkpoint `db084c4` hardens action ownership, emergency cancellation,
  feedback validation, and fault confirmation before physical tests.
- Automatic fallback remains unauthorized in both robot configurations; the
  brain shows `AUTO LOCKED` for the normal tracking mode.
- Forced encoder testing is separately unauthorized, so `AUTO LOCKED` cannot
  enter encoder mode before the baseline measurements are recorded.
- Task 7 is blocked at its first physical gate. Record the existing AUT3 LemLib
  12-inch test and a separately rebooted native Kevin fallback before adding or
  exposing forced-encoder validation routines.
- Multi-action encoder pose continuity remains unimplemented. Keep automatic
  fallback and dependent encoder route segments locked until a dead-reckoned
  diagnostic pose is implemented and host-tested.

---

## File Structure

- `include/aon/auton/motion-health.hpp`: pure sensor sample types, thresholds, failure reasons, and health-monitor interface.
- `src/aon/auton/motion-health.cpp`: pure consecutive-invalid, impossible-jump, and frozen-tracking state machine.
- `include/aon/auton/fallback-geometry.hpp`: pure relative fallback target calculations.
- `src/aon/auton/fallback-geometry.cpp`: target bearing, signed distance, heading normalization, and motor-degree conversion.
- `tests/motion-fallback-test.cpp`: dependency-free host test runner for both pure modules.
- `include/aon/lemlib/drive-io.hpp`: narrow PROS-facing drivetrain sample and command interface.
- `src/aon/lemlib/chassis.cpp`: reuse the single motor/sensor instances for LemLib and drive I/O.
- `include/aon/auton/fallback-status.hpp`: process-lifetime mode and diagnostic snapshot API.
- `src/aon/auton/fallback-status.cpp`: mutex-protected startup selection, autonomous lock, and fault latch.
- `include/aon/auton/encoder-motion.hpp`: encoder controller request/result interface.
- `src/aon/auton/encoder-motion.cpp`: bounded relative distance and turn loops.
- `include/aon/auton/actions.hpp`: structured `MotionResult` return contract.
- `src/aon/auton/actions.cpp`: asynchronous LemLib monitoring, safe transition, retry, and logging.
- `include/aon/config/robot-config.hpp`: per-robot fallback conversion and health thresholds.
- `src/aon/config/robot-config.cpp`: explicit small/big initial values.
- `include/aon/tools/gui/gui.hpp`: brain fallback selection state and touch hook.
- `include/aon/tools/gui/ui/gui-layout.hpp`: fallback mode button geometry.
- `src/aon/tools/gui/gui.cpp`: selection handling and autonomous lock behavior.
- `src/aon/tools/gui/ui/gui-displays.cpp`: selected/active mode and fault reason display.
- `src/aon/auton/lemlib-routines.cpp`: propagate required motion failures.
- `src/aon/auton/routine-selectors.cpp`: preserve result propagation into routine status.
- `docs/CURRENT_HANDOFF.md`: record checkpoints and physical gates.

---

### Task 1: Pure Motion Health Monitor

**Files:**
- Create: `include/aon/auton/motion-health.hpp`
- Create: `src/aon/auton/motion-health.cpp`
- Create: `tests/motion-fallback-test.cpp`

**Interfaces:**
- Produces: `aon::auton::MotionSample`, `HealthThresholds`, `MotionIntent`, `MotionFailureReason`, and `MotionHealthMonitor::observe(const MotionSample&, MotionIntent)`.
- Consumes: only C++ standard-library headers; this task must compile without PROS or LemLib.

- [ ] **Step 1: Add failing tests for transient invalid samples and confirmed invalid devices**

Create a test runner with a local `CHECK` macro and these first cases:

```cpp
#include "aon/auton/motion-health.hpp"

#include <cstdlib>
#include <iostream>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

using aon::auton::MotionFailureReason;
using aon::auton::MotionHealthMonitor;
using aon::auton::MotionIntent;
using aon::auton::MotionSample;

MotionSample healthy(std::uint32_t timeMs) {
  MotionSample sample{};
  sample.timeMs = timeMs;
  sample.poseValid = true;
  sample.leftMotorValid = true;
  sample.rightMotorValid = true;
  sample.leftTrackingValid = true;
  sample.rightTrackingValid = true;
  sample.backTrackingValid = true;
  sample.imuValid = true;
  return sample;
}

void testInvalidSamplesRequireConfirmation() {
  MotionHealthMonitor monitor({});
  auto sample = healthy(0);
  monitor.reset(sample);
  sample = healthy(20);
  sample.leftTrackingValid = false;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);
  sample = healthy(40);
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);
  sample.leftTrackingValid = false;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);
  sample.timeMs = 60;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::None);
  sample.timeMs = 80;
  CHECK(monitor.observe(sample, MotionIntent::Linear) ==
        MotionFailureReason::DeviceInvalid);
}
```

- [ ] **Step 2: Run the host test to verify it fails**

Run:

```powershell
New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\motion-fallback-test.cpp src\aon\auton\motion-health.cpp -o bin\host-tests\motion-fallback-test.exe
```

Expected: compilation fails because `aon/auton/motion-health.hpp` does not exist.

- [ ] **Step 3: Define the pure health interface**

Add these exact public types:

```cpp
#pragma once

#include <cstdint>

namespace aon::auton {

enum class MotionIntent { Idle, Linear, Turn };

enum class MotionFailureReason {
  None,
  DeviceInvalid,
  NonFinitePose,
  ImpossiblePoseJump,
  FrozenTracking,
  Timeout,
  Cancelled,
  Unsupported,
  RetryFailed,
};

struct MotionSample {
  std::uint32_t timeMs = 0;
  double poseX = 0.0;
  double poseY = 0.0;
  double poseHeading = 0.0;
  double leftMotorDegrees = 0.0;
  double rightMotorDegrees = 0.0;
  double leftTrackingInches = 0.0;
  double rightTrackingInches = 0.0;
  double backTrackingInches = 0.0;
  double imuDegrees = 0.0;
  bool poseValid = false;
  bool leftMotorValid = false;
  bool rightMotorValid = false;
  bool leftTrackingValid = false;
  bool rightTrackingValid = false;
  bool backTrackingValid = false;
  bool imuValid = false;
};

struct HealthThresholds {
  std::uint32_t invalidConfirmationSamples = 3;
  std::uint32_t impossibleJumpConfirmationSamples = 2;
  std::uint32_t frozenTrackingDwellMs = 300;
  double frozenMotorDeltaDegrees = 15.0;
  double trackingMovementEpsilonInches = 0.02;
  double maxPoseJumpInchesPerSample = 8.0;
  double maxHeadingJumpDegreesPerSample = 45.0;
};

class MotionHealthMonitor {
 public:
  explicit MotionHealthMonitor(HealthThresholds thresholds);
  void reset(const MotionSample& sample);
  MotionFailureReason observe(const MotionSample& sample, MotionIntent intent);
  const MotionSample& lastTrustedSample() const;

 private:
  HealthThresholds thresholds_;
  MotionSample previous_{};
  MotionSample trusted_{};
  MotionSample trackingBaseline_{};
  std::uint32_t invalidCount_ = 0;
  std::uint32_t impossibleJumpCount_ = 0;
  bool initialized_ = false;
};

const char* motionFailureName(MotionFailureReason reason);

}  // namespace aon::auton
```

- [ ] **Step 4: Implement consecutive confirmation and trusted-sample handling**

Implement `observe` in this order:

```cpp
const bool finitePose = sample.poseValid && std::isfinite(sample.poseX) &&
                        std::isfinite(sample.poseY) &&
                        std::isfinite(sample.poseHeading);
const bool devicesValid = sample.leftMotorValid && sample.rightMotorValid &&
                          sample.leftTrackingValid &&
                          sample.rightTrackingValid &&
                          sample.backTrackingValid && sample.imuValid;

if (!finitePose || !devicesValid) {
  ++invalidCount_;
  if (invalidCount_ >= thresholds_.invalidConfirmationSamples) {
    return finitePose ? MotionFailureReason::DeviceInvalid
                      : MotionFailureReason::NonFinitePose;
  }
  previous_ = sample;
  return MotionFailureReason::None;
}
invalidCount_ = 0;
```

Then calculate pose distance with `std::hypot`, normalize heading delta with
`std::remainder(delta, 360.0)`, and require two consecutive impossible jumps.
Update `trusted_` only after every active check passes.

- [ ] **Step 5: Add failing frozen-sensor, blocked-drive, and impossible-jump tests**

Append tests that build 20 ms samples and verify:

```cpp
void testFrozenTrackingRequiresMotorMovementAndDwell();
void testBlockedDriveDoesNotLookLikeFrozenTracking();
void testImpossiblePoseJumpRequiresConfirmation();
```

For the frozen case, advance `leftMotorDegrees` and `rightMotorDegrees` by 2
degrees per sample for 320 ms while leaving both longitudinal tracking values
unchanged; expect `FrozenTracking`. For the blocked case leave motor and tracking
values unchanged for 500 ms; expect `None`. For the jump case use two successive
9-inch pose jumps with the default 8-inch threshold; expect a fault only on the
second jump.

- [ ] **Step 6: Implement frozen-tracking disagreement**

While intent is `Linear` or `Turn`, reset `trackingBaseline_` whenever either
longitudinal tracking wheel changes by at least
`trackingMovementEpsilonInches`. Otherwise, after
`frozenTrackingDwellMs`, compare each motor side with its baseline. Return
`FrozenTracking` only if a motor side exceeded `frozenMotorDeltaDegrees` while
its corresponding tracking wheel remained below the epsilon. Do not perform
this check for `Idle`.

- [ ] **Step 7: Run all host tests**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\motion-fallback-test.cpp src\aon\auton\motion-health.cpp -o bin\host-tests\motion-fallback-test.exe
& .\bin\host-tests\motion-fallback-test.exe
```

Expected: exit code 0 and `motion fallback tests passed`.

- [ ] **Step 8: Commit the pure monitor**

```powershell
git add -- include/aon/auton/motion-health.hpp src/aon/auton/motion-health.cpp tests/motion-fallback-test.cpp
git commit -m "Add testable odometry health monitor"
```

---

### Task 2: Pure Fallback Geometry and Robot Threshold Configuration

**Files:**
- Create: `include/aon/auton/fallback-geometry.hpp`
- Create: `src/aon/auton/fallback-geometry.cpp`
- Modify: `tests/motion-fallback-test.cpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`

**Interfaces:**
- Consumes: `MotionSample` and `HealthThresholds` from Task 1.
- Produces: `FallbackGeometry`, `pointFallback(...)`, `poseFallback(...)`, `headingFallback(...)`, and per-robot `FallbackConfig`.

- [ ] **Step 1: Add failing geometry tests**

Add cases with heading zero defined as +Y:

```cpp
CHECK(pointFallback({0, 0, 0}, 0, 12).distanceInches == 12);
CHECK(pointFallback({0, 0, 0}, 12, 0).turnDegrees == 90);
CHECK(pointFallback({0, 0, 180}, 0, 12).turnDegrees == -180);
CHECK(headingFallback(350, 10) == 20);
CHECK(headingFallback(10, 350) == -20);
CHECK(motorDegreesForDistance(12, 2.75, 0.75) > 0);
```

Use a tolerance helper of `1e-6` instead of direct equality for calculated
floating-point values.

- [ ] **Step 2: Run the host build and verify missing symbols**

Run the Task 1 host compile command with
`src\aon\auton\fallback-geometry.cpp` added. Expected: compilation fails because
the geometry header and functions do not exist.

- [ ] **Step 3: Define and implement fallback geometry**

Use this API:

```cpp
struct TrustedPose { double x; double y; double heading; };
struct FallbackGeometry {
  double turnDegrees;
  double distanceInches;
  double finalTurnDegrees;
};

FallbackGeometry pointFallback(TrustedPose start, double targetX,
                               double targetY);
FallbackGeometry poseFallback(TrustedPose start, double targetX,
                              double targetY, double targetHeading);
double headingFallback(double currentHeading, double targetHeading);
double motorDegreesForDistance(double distanceInches,
                               double driveWheelDiameter,
                               double wheelRevolutionsPerMotorRevolution);
```

Calculate bearing with `atan2(targetX - start.x, targetY - start.y)` so zero
degrees remains +Y. Normalize every turn with `std::remainder(value, 360.0)`.
Convert distance using:

```cpp
return distanceInches /
       (M_PI * driveWheelDiameter * wheelRevolutionsPerMotorRevolution) *
       360.0;
```

- [ ] **Step 4: Add fallback configuration to each robot**

Append to `LemLibDriveConfig`:

```cpp
struct FallbackConfig {
  double wheelRevolutionsPerMotorRevolution;
  double distanceKp;
  double turnKp;
  int minimumOutput;
  int maximumOutputPercent;
  std::uint32_t settleMs;
  std::uint32_t transitionAllowanceMs;
  aon::auton::HealthThresholds health;
};
```

Include `aon/auton/motion-health.hpp` from `robot-config.hpp`. Initialize both
robots explicitly with their existing `MOTOR_TO_DRIVE_RATIO`, conservative
starting gains `distanceKp = 4.0`, `turnKp = 1.2`, `minimumOutput = 18`,
`maximumOutputPercent = 60`, `settleMs = 100`, and
`transitionAllowanceMs = 250`. Copy the Task 1 default health thresholds
explicitly into both configurations so later tuning changes are robot-specific.

- [ ] **Step 5: Run host tests and both compile configurations**

Run the host test. Then build small robot. Temporarily change only
`#define USING_BIG_ROBOT false` to `true`, clean-build big robot, restore
`false`, and clean-build small robot again.

Expected: all builds succeed; the only permitted warning is the existing
vendored `json.hpp` `std::is_pod` deprecation.

- [ ] **Step 6: Commit geometry and configuration**

```powershell
git add -- include/aon/auton/fallback-geometry.hpp src/aon/auton/fallback-geometry.cpp tests/motion-fallback-test.cpp include/aon/config/robot-config.hpp src/aon/config/robot-config.cpp include/aon/constants.hpp
git commit -m "Define encoder fallback geometry and thresholds"
```

---

### Task 3: Single-Owner Drive Sampling and Commands

**Files:**
- Create: `include/aon/lemlib/drive-io.hpp`
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `include/aon/lemlib/chassis.hpp`

**Interfaces:**
- Produces: `DriveSensorSample sampleDriveSensors()`, `commandTank(int, int)`, `stopDrive()`, and `setDriveBrakeMode(pros::motor_brake_mode_e)`.
- Consumes: the exact motor groups, rotation sensors, and IMU already used by `chassis()`; it must not construct duplicate devices.

- [ ] **Step 1: Add the drive I/O data contract**

```cpp
struct DriveSensorSample {
  double leftMotorDegrees;
  double rightMotorDegrees;
  double leftTrackingInches;
  double rightTrackingInches;
  double backTrackingInches;
  double imuDegrees;
  bool leftMotorValid;
  bool rightMotorValid;
  bool leftTrackingValid;
  bool rightTrackingValid;
  bool backTrackingValid;
  bool imuValid;
};

DriveSensorSample sampleDriveSensors();
void commandTank(int left, int right);
void stopDrive();
void setDriveBrakeMode(pros::motor_brake_mode_e brakeMode);
```

- [ ] **Step 2: Refactor chassis-local devices into shared private accessors**

Inside `chassis.cpp`, add private functions `leftMotors()`, `rightMotors()`,
`leftEncoder()`, `rightEncoder()`, `backEncoder()`, and `imu()`. Each returns one
function-local static instance configured from `activeRobotConfig()`. Update
`chassis()` and tracking-wheel construction to reference these accessors. Remove
the old duplicate locals from `chassis()`.

- [ ] **Step 3: Implement validity-aware sampling**

Average `get_position_all()` values only when the vector is non-empty and every
entry is finite and not `PROS_ERR_F`. Treat a rotation position of `PROS_ERR` as
invalid. Treat IMU rotation as invalid when non-finite, equal to `PROS_ERR_F`,
or `get_status()` equals `pros::ImuStatus::error`. Reuse the existing
`trackingInches` conversion for valid rotation samples.

- [ ] **Step 4: Implement bounded tank commands**

Clamp both commands to `[-127, 127]`, call `MotorGroup::move`, and provide a
zero-output `stopDrive`. `setDriveBrakeMode` updates both motor groups. Do not
call these command functions from normal LemLib motion in this task.

- [ ] **Step 5: Clean-build both robot configurations and restore small robot**

Expected: both builds pass without new warnings. Confirm with `rg` that each
configured motor and sensor port is constructed only in the new private
accessor. Rewire the calibration-only `trackingSample()` helper to those same
accessors; do not retain duplicate sensor objects.

- [ ] **Step 6: Commit drive I/O**

```powershell
git add -- include/aon/lemlib/drive-io.hpp include/aon/lemlib/chassis.hpp src/aon/lemlib/chassis.cpp include/aon/constants.hpp
git commit -m "Expose single-owner drivetrain sampling"
```

---

### Task 4: Latched Mode Status and Encoder Controller

**Files:**
- Create: `include/aon/auton/fallback-status.hpp`
- Create: `src/aon/auton/fallback-status.cpp`
- Create: `include/aon/auton/encoder-motion.hpp`
- Create: `src/aon/auton/encoder-motion.cpp`
- Modify: `tests/motion-fallback-test.cpp`

**Interfaces:**
- Produces: `MotionMode`, `FallbackStatusSnapshot`, `selectForcedEncoder(bool)`, `lockFallbackSelection()`, `latchFallbackFault(...)`, `fallbackStatus()`, and `EncoderMotionController`.
- Consumes: Task 2 geometry/config and Task 3 drive I/O.

- [ ] **Step 1: Define mode and thread-safe status API**

```cpp
enum class MotionMode { Tracking, ForcedEncoder, FaultedEncoder };

struct FallbackStatusSnapshot {
  MotionMode mode;
  MotionFailureReason reason;
  bool selectionLocked;
  std::array<char, 48> actionName;
  std::uint32_t changedAt;
};

bool selectForcedEncoder(bool forced);
void lockFallbackSelection();
void latchFallbackFault(MotionFailureReason reason, const char* actionName);
FallbackStatusSnapshot fallbackStatus();
const char* motionModeName(MotionMode mode);
```

Store the action name in a fixed-size `std::array<char, 48>` internally so a
route's temporary string cannot dangle. Protect all shared state with one
`pros::Mutex`. Reject selection changes after locking and reject every attempt
to leave `FaultedEncoder`.

- [ ] **Step 2: Define encoder request/result types**

```cpp
struct EncoderMotionRequest {
  const char* name;
  FallbackGeometry geometry;
  int requestedMaxOutput;
  std::uint32_t timeoutMs;
  bool imuAllowed;
};

struct EncoderMotionResult {
  bool succeeded;
  MotionFailureReason reason;
  double distanceErrorInches;
  double headingErrorDegrees;
};

class EncoderMotionController {
 public:
  EncoderMotionResult execute(const EncoderMotionRequest& request);
  void cancel();
};
```

- [ ] **Step 3: Add failing pure tests for output caps and timeout budgeting**

Add these helpers to `fallback-geometry.hpp/.cpp` and test them without linking
the PROS-facing encoder controller:

```cpp
int cappedFallbackOutput(int requested, int configuredPercent);
std::uint32_t fallbackBudget(std::uint32_t startedAt,
                             std::uint32_t now,
                             std::uint32_t originalTimeout,
                             std::uint32_t transitionAllowance);
```

Verify `cappedFallbackOutput(100, 60) == 60`, negative magnitude is handled,
and a motion with 700 ms used from a 1000 ms timeout plus 250 ms transition gets
550 ms, never underflows, and never becomes unbounded.

- [ ] **Step 4: Implement bounded relative distance control**

Snapshot both motor positions, calculate average signed motor delta, convert it
to inches with Task 2 configuration, and run a 20 ms proportional loop. Clamp
to the capped output and apply `minimumOutput` only outside a 0.5-inch finish
band. Require error inside 0.5 inch for `settleMs`. On invalid motor feedback,
cancel, timeout, or budget exhaustion, call `stopDrive()` and return failure.

- [ ] **Step 5: Implement bounded turn control with IMU degradation**

Use IMU delta when the starting and current samples are valid. If IMU becomes
invalid, continue from the last valid heading using differential wheel travel:

```cpp
deltaRadians = (rightDistanceInches - leftDistanceInches) / trackWidth;
deltaDegrees = deltaRadians * 180.0 / M_PI;
```

Normalize error, clamp output, command `(-output, output)`, and require error
inside 2 degrees for `settleMs`. A motor feedback failure still aborts.

- [ ] **Step 6: Sequence point, pose, and heading requests**

Execute initial turn, relative drive, and optional final turn from
`FallbackGeometry`, sharing one absolute deadline. Skip segments already inside
their finish band. Stop and return immediately on the first failed segment.

- [ ] **Step 7: Run host tests and both robot builds**

Expected: host tests pass, both robot configurations compile, and small robot is
restored.

- [ ] **Step 8: Commit encoder control**

```powershell
git add -- include/aon/auton/fallback-status.hpp src/aon/auton/fallback-status.cpp include/aon/auton/encoder-motion.hpp src/aon/auton/encoder-motion.cpp tests/motion-fallback-test.cpp include/aon/constants.hpp
git commit -m "Add latched encoder motion fallback"
```

---

### Task 5: Resilient Autonomous Actions

**Files:**
- Modify: `include/aon/auton/actions.hpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: `src/aon/auton/lemlib-routines.cpp`
- Modify: `src/aon/lemlib/chassis.cpp`

**Interfaces:**
- Produces: `MotionResult` from every drivetrain action and one monitored retry path.
- Consumes: Tasks 1 through 4.

- [ ] **Step 1: Define the result contract and change required signatures**

```cpp
struct MotionResult {
  bool succeeded;
  MotionMode mode;
  bool fallbackUsed;
  MotionFailureReason reason;
};
```

Return `MotionResult` from `moveToPoint`, `moveToPose`, `turnToHeading`,
`arcadeFor`, and `followPath`. Keep `setPose`, `cancelMotion`, and `stop` as
`void`. Add a private `runMonitored(...)` implementation in `actions.cpp`; do
not expose LemLib callbacks in the public header.

- [ ] **Step 2: Start LemLib operations asynchronously and sample at 20 ms**

For tracking mode, call LemLib with `async = true`. While
`chassis().isInMotion()`, combine `getPose()` with `sampleDriveSensors()` into a
`MotionSample`, infer `MotionIntent`, and call the health monitor. Preserve the
existing overall timeout as the hard deadline even if LemLib remains active.

- [ ] **Step 3: Implement the safe transition order**

On a confirmed fault, execute exactly:

```cpp
robotChassis.cancelAllMotions();
stopDrive();
setDriveBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
pros::delay(config.fallback.settleMs);
latchFallbackFault(reason, name);
```

Sample motor velocity or position during the settle interval and abort with
`RetryFailed` if movement does not decrease. Only then construct fallback
geometry from `monitor.lastTrustedSample()` and call `EncoderMotionController`.

- [ ] **Step 4: Enforce one retry and unsupported path behavior**

Tracking mode gets at most one encoder attempt after a confirmed fault. Forced
mode executes only one encoder attempt. `followPath` returns `Unsupported` in
either encoder mode and cancels safely if a fault occurs while following. A
normal LemLib timeout returns `Timeout` without latching fallback.

- [ ] **Step 5: Make cancellation dominate retry**

Add one action-ownership flag protected by a mutex or atomic. `cancelMotion()`
sets cancellation first, cancels LemLib, cancels the encoder controller, and
stops drive output. `runMonitored` checks cancellation before latching or
retrying and returns `Cancelled`.

- [ ] **Step 6: Expand stable action logging**

Finish records must include:

```text
AUTON_FINISH operation=<op> name=<name> succeeded=<0|1> mode=<mode> fallback=<0|1> reason=<reason> x=<x> y=<y> heading=<h> time=<ms>
```

Transition records must include action, reason, trusted pose, target, tracking
values, motor values, and timestamp under the stable prefix `ODOM_FALLBACK`.

- [ ] **Step 7: Propagate failures through current LemLib routines**

Replace unconditional success after required motion with checks:

```cpp
const auto result = routine.moveToPoint(...);
if (!result.succeeded) {
  routine.stop();
  aon::auton::mechanisms::stopAll();
  return 0;
}
```

Apply this to forward validation, turn characterization, JerryIO path test, and
every required movement in the staged loader experiment. Do not run later
mechanism actions after a required drivetrain failure.

- [ ] **Step 8: Run host tests and clean-build both configurations**

Expected: host tests and both builds pass, small robot is restored, and `rg`
finds no ignored `MotionResult` in `src/aon/auton/lemlib-routines.cpp` or LemLib
test helpers.

- [ ] **Step 9: Commit resilient actions**

```powershell
git add -- include/aon/auton/actions.hpp src/aon/auton/actions.cpp src/aon/auton/lemlib-routines.cpp src/aon/lemlib/chassis.cpp include/aon/constants.hpp
git commit -m "Retry failed odometry motions with encoders"
```

---

### Task 6: Brain Selection and Diagnostics

**Files:**
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `include/aon/tools/gui/ui/gui-layout.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`
- Modify: `src/aon/tools/gui/ui/gui-displays.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`

**Interfaces:**
- Consumes: Task 4 fallback status and Task 5 results.
- Produces: visible `AUTO FALLBACK` / `FORCE ENCODERS` selection and active-mode diagnostics.

- [ ] **Step 1: Add one mode button to the autonomous hub**

Define a button that fits above the alliance buttons without overlapping BACK
or the selected-auton title. Its dynamic label is drawn from the current status:

```cpp
ui::Button modeButton = fallbackModeBtn;
modeButton.label = fallbackStatus().mode == MotionMode::ForcedEncoder
                       ? "FORCE ENCODERS"
                       : "AUTO FALLBACK";
modeButton.bg = fallbackStatus().mode == MotionMode::ForcedEncoder
                    ? COLOR_ORANGE
                    : COLOR_DARK_GREEN;
modeButton.draw(pros::E_TEXT_SMALL);
```

- [ ] **Step 2: Toggle only before autonomous lock**

In `handleAutonMenuTouch`, call `selectForcedEncoder(...)` when the mode button
is hit, then redraw. If selection is locked or mode is faulted, ignore the touch
and keep the diagnostic display. Do not call `saveAutonSelection`; the mode must
not be stored on SD.

- [ ] **Step 3: Lock selection at autonomous dispatch**

Call `lockFallbackSelection()` at the start of the existing `runRoutine`
wrapper, immediately before invoking its LemLib routine. Do not call it from
`runNativeRoutine` or `runDrivetrainTest`; native Kevin routines remain native
and must not accidentally start encoder fallback.

- [ ] **Step 4: Display active mode and reason**

Add a compact line on the main and autonomous screens:

```text
ODOM: TRACKING
ODOM: FORCED ENCODER
ODOM: FAULT ENCODER - FROZEN
```

Redraw the main screen when mode or failure reason changes, in addition to the
existing routine-state change detection. Keep the routine status line visible.

- [ ] **Step 5: Verify routine failure reaches GUI status**

Confirm `runRoutine` still passes `result != 0` to `finishRoutine`. Add no GUI
special case that turns a failed fallback into completed status.

- [ ] **Step 6: Clean-build both configurations and inspect GUI coordinates**

Expected: both builds pass and button hitboxes do not overlap. Restore small
robot. This is a compile/layout checkpoint, not permission to field-run.

- [ ] **Step 7: Commit brain control**

```powershell
git add -- include/aon/tools/gui/gui.hpp include/aon/tools/gui/ui/gui-layout.hpp src/aon/tools/gui/gui.cpp src/aon/tools/gui/ui/gui-displays.cpp src/aon/auton/routine-selectors.cpp include/aon/constants.hpp
git commit -m "Add brain control for encoder fallback"
```

---

### Task 7: Validation Routines, Handoff, and Physical Gates

**Files:**
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/lemlib-routines.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `docs/CURRENT_HANDOFF.md`
- Modify: `docs/superpowers/plans/2026-07-17-motor-encoder-odometry-fallback.md`

**Interfaces:**
- Produces: short forced-encoder forward/reverse/turn tests and explicit physical gate records.
- Consumes: all earlier tasks.

- [ ] **Step 1: Preserve the existing migration gate before fallback tests**

Restart the brain, run AUT3 `TEST LemLib 12in`, and record physical distance plus
final LemLib X/Y/heading. Restart again, run one native Kevin fallback, and
record GUI, drivetrain, and mechanism behavior. If this gate fails, stop and use
the diagnosing-bugs workflow; do not enable fallback tests.

- [ ] **Step 2: Add reduced-speed forced-encoder primitive routines**

Add separate entry points for 6-inch forward, 6-inch reverse, +45-degree turn,
and -45-degree turn. The operator selects `FORCE ENCODERS` on the brain before
dispatch. Each routine executes one `Actions` operation with requested maximum
output 30 and a 2500 ms timeout, logs the `MotionResult`, stops, and returns
`result.succeeded ? 1 : 0`. If the mode is not forced, the test returns failure
without moving.

- [ ] **Step 3: Expose only one unvalidated fallback test through AUT3**

Begin with `TEST Encoder 6in`. Do not expose turn tests until forward and reverse
both pass. Preserve native Kevin options in AUT1/AUT2. Commit each newly exposed
primitive separately after its preceding physical result is recorded.

- [ ] **Step 4: Bench-test forced mode with wheels clear**

Verify the brain defaults to `AUTO FALLBACK`, toggles to `FORCE ENCODERS`, does
not retain the forced selection after restart, caps output, stops at timeout,
and shows the correct terminal/brain status. Record motor deltas and measured
movement direction.

- [ ] **Step 5: Test forced primitives in clear floor space**

Run forward, reverse, clockwise, and counterclockwise separately with an
emergency stop available. Record five runs per primitive: physical distance or
angle, motor deltas, final diagnostic estimate, battery state, and drift.
Change only one controller gain or conversion category per calibration commit.

- [ ] **Step 6: Test automatic transition with a controlled fault**

With wheels clear first, disconnect one tracking sensor, start a short tracking
motion, and verify three invalid samples, LemLib cancellation, zero command,
settle delay, one reduced-speed retry, and a latched fault display. Repeat once
in clear floor space only after the wheels-clear sequence is correct.

- [ ] **Step 7: Prove false-positive protections**

Briefly block the drivetrain so tracking and motor encoders remain still. Verify
the action times out without `ODOM_FALLBACK`. Inject or observe a single bad
sample and verify no transition. Do not intentionally create uncontrolled wheel
slip near field structures.

- [ ] **Step 8: Final verification**

Run host tests, `git diff --check`, clean-build small, clean-build big, restore
small, and clean-build small again. Expected: clean builds with only the known
vendored warning. Confirm `USING_BIG_ROBOT false` before committing.

- [ ] **Step 9: Update the handoff**

Record the latest completed commit, exact physical results, active AUT3 test,
whether automatic fallback is authorized for competition routes, and the next
physical gate. Do not claim field validation from compilation.

- [ ] **Step 10: Commit and synchronize the validated checkpoint**

```powershell
git add -- include/aon/auton/routines.hpp src/aon/auton/lemlib-routines.cpp src/aon/auton/routine-selectors.cpp include/aon/tools/gui/gui.hpp docs/CURRENT_HANDOFF.md docs/superpowers/plans/2026-07-17-motor-encoder-odometry-fallback.md include/aon/constants.hpp
git commit -m "Validate motor encoder fallback primitives"
git push origin Testing
git status --short --branch
```

Expected: clean `Testing` synchronized with `origin/Testing`.
