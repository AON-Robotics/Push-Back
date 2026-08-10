# Localization and EKF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the legacy raw-odometry weaknesses with a deterministic, allocation-free three-wheel estimator and lightweight three-state EKF, then make the existing LemLib motion stack consume the fused pose after an explicit integration and physical-validation gate.

**Architecture:** Platform-independent localization math lives behind fixed-size value interfaces and is host-tested before PROS code changes. `aon::Odometry` becomes the single coherent estimator and sensor adapter; LemLib remains the motion controller and receives fused pose through its existing pose seam only after its internal wheel-pose writer is proven disabled. GPS is fully supported but remains disabled until its port and mounting geometry are measured.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, fixed-size explicit 3x3 equations, MinGW g++ host tests, GNU Make/ARM toolchain, PowerShell, Git.

## Global Constraints

- Preserve the existing project structure and coding style where practical.
- Do not rewrite unrelated systems or modify autonomous route geometry or mechanism behavior.
- Keep `aon::auton::Actions` as the autonomous motion interface.
- Do not edit vendored PROS, LemLib, LVGL, fmt, JSON, or firmware files.
- Avoid dynamic allocation, streams, formatted output, and general matrix libraries in the localization loop.
- Public positions are inches and headings are degrees; internal estimator angles are radians.
- The field frame is `+X` right, `+Y` forward at zero heading, with clockwise-positive heading.
- Make one behavior, calibration category, integration stage, or filter stage per commit.
- Run localization host tests at every math checkpoint and both robot builds at every PROS-facing checkpoint.
- Restore `USING_BIG_ROBOT false` and leave the worktree clean before every commit.
- Preserve the known big-robot right-tracker reversal disagreement until physical calibration resolves it.
- Keep GPS disabled until its valid smart port, mounting offsets, mounting heading, and field alignment are supplied.
- Keep fused-LemLib mode unauthorized until the existing baseline gate and this plan's integration gate are recorded on the real robot.
- Never claim that EKF fusion improves localization without repeated measured results.
- Preserve unexpected user or concurrent-agent changes and stage only paths named by the current task.

---

## Target File Structure

```text
include/aon/odometry/
  odometry.hpp                 PROS adapter and stable public interface
  sensor-measurements.hpp      raw/validated measurements and angle helpers
  pose-estimator.hpp           three-wheel kinematics and SE(2) propagation
  ekf.hpp                      fixed-size three-state EKF
  diagnostics.hpp              fixed-size diagnostic snapshot and rejection enums

src/aon/
  odometry.cpp                 sensor acquisition, reset, publication, timing
  pose-estimator.cpp           platform-independent raw motion model
  ekf.cpp                      platform-independent prediction/updates

tests/
  localization-math-test.cpp   host behavior tests
  localization-integration-test.cpp configuration and source-boundary checks

tools/
  test-localization.ps1        deterministic host compile/run entry point

docs/testing/
  localization-benchmark.md    physical procedure and result tables
```

`include/aon/config/robot-config.hpp` owns robot-specific localization values.
`src/aon/config/robot-config.cpp` supplies values for both configurations.
`src/aon/lemlib/chassis.cpp` continues owning LemLib motion adaptation and is
the only code allowed to publish AON fused pose into LemLib.

## Relationship to the Broader Navigation Roadmap

The concurrently approved
`docs/superpowers/specs/2026-08-10-fused-localization-lidar-navigation-design.md`
describes a later LiDAR/Pi navigation program. This plan is the bounded V5
localization-core project underneath that roadmap:

- it intentionally implements the user-requested minimum `[x, y, theta]` state;
- it does not add velocity, delayed-observation history, LiDAR, serial protocol,
  obstacle planning, or a replacement path follower;
- its fixed measurement and diagnostic types may later be adapted by a broader
  `LocalizationManager` without exposing PROS devices or mutable EKF state;
- it does not claim LemLib consumes fused pose merely because `setPose` exists;
  Tasks 10-11 require proof before enabling or documenting that behavior;
- if the LemLib integration gate fails, the tested three-state estimator remains
  useful as the raw/fused foundation for the later injected-pose follower.

This keeps the current request maintainable and prevents the future roadmap's
six-state/LiDAR scope from entering the V5 odometry correction checkpoint.

---

### Task 1: Establish the Localization Host-Test Harness and Angle Contract

**Files:**
- Create: `include/aon/odometry/sensor-measurements.hpp`
- Create: `tests/localization-math-test.cpp`
- Create: `tools/test-localization.ps1`

**Interfaces:**
- Produces: `aon::localization::radians(double)`, `degrees(double)`, `wrapRadians(double)`, `shortestAngleDelta(double, double)`, `sinc(double)`, and fixed-size measurement types.
- Consumes: only the C++ standard library; this checkpoint must compile without PROS or LemLib.

- [ ] **Step 1: Write failing angle and unit tests**

Create the test harness with the repository's existing `CHECK` macro style and
these exact behavioral cases:

```cpp
checkNear(wrapRadians(radians(359.0)), radians(-1.0));
checkNear(shortestAngleDelta(radians(359.0), radians(1.0)), radians(2.0));
checkNear(shortestAngleDelta(radians(-179.0), radians(179.0)), radians(-2.0));
checkNear(degrees(radians(90.0)), 90.0);
checkNear(sinc(0.0), 1.0);
checkNear(sinc(1e-9), 1.0, 1e-12);
```

Define the required types in the test before they exist:

```cpp
WheelDistances wheels{1.0, 2.0, 3.0, true, true, true};
ImuMeasurement imu{radians(10.0), true};
GpsMeasurement gps{12.0, 24.0, radians(90.0), 1.5,
                   true, true, true, 100U};
CHECK(wheels.backValid && imu.valid && gps.fresh);
```

- [ ] **Step 2: Run the test and verify RED**

Create `tools/test-localization.ps1` with compiler discovery used by existing
host plans:

```powershell
$ErrorActionPreference = 'Stop'
$compiler = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe'
if (-not (Test-Path -LiteralPath $compiler)) {
  $compiler = (Get-Command g++ -ErrorAction Stop).Source
}
New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
& $compiler -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\localization-math-test.cpp `
  -o bin\host-tests\localization-math-test.exe
if ($LASTEXITCODE -ne 0) { throw 'localization host compile failed' }
& '.\bin\host-tests\localization-math-test.exe'
if ($LASTEXITCODE -ne 0) { throw 'localization host tests failed' }
```

Run: `& tools\test-localization.ps1`

Expected: FAIL because `aon/odometry/sensor-measurements.hpp` and its symbols do
not exist.

- [ ] **Step 3: Implement the minimal fixed-size measurement layer**

Add these public types and helpers:

```cpp
namespace aon::localization {
inline constexpr double kPi = 3.14159265358979323846;

struct WheelDistances {
  double leftInches;
  double rightInches;
  double backInches;
  bool leftValid;
  bool rightValid;
  bool backValid;
};

struct ImuMeasurement {
  double headingRadians;
  bool valid;
};

struct GpsMeasurement {
  double xInches;
  double yInches;
  double headingRadians;
  double positionErrorInches;
  bool positionValid;
  bool headingValid;
  bool fresh;
  std::uint32_t timestampMs;
};

inline double radians(double degrees) noexcept;
inline double degrees(double radians) noexcept;
inline double wrapRadians(double angle) noexcept;
inline double shortestAngleDelta(double from, double to) noexcept;
inline double sinc(double value) noexcept;
}
```

Use `std::remainder(angle, 2*kPi)` for wrapping and the series
`1 - z*z/6 + z*z*z*z/120` when `abs(z) < 1e-4`. Define these
small helpers inline in the header so Task 1 remains a one-file math module.

- [ ] **Step 4: Run tests and verify GREEN**

Run: `& tools\test-localization.ps1`

Expected: `localization math tests passed`.

- [ ] **Step 5: Commit the angle and measurement contract**

```powershell
git add -- include/aon/odometry/sensor-measurements.hpp `
  tests/localization-math-test.cpp tools/test-localization.ps1
git diff --cached --check
git commit -m "test(localization): define angle and measurement contract"
```

---

### Task 2: Implement Correct Three-Wheel Pose Propagation

**Files:**
- Create: `include/aon/odometry/pose-estimator.hpp`
- Create: `src/aon/pose-estimator.cpp`
- Modify: `tests/localization-math-test.cpp`
- Modify: `tools/test-localization.ps1`

**Interfaces:**
- Consumes: Task 1 angle helpers.
- Produces: `TrackingGeometry`, `WheelDeltas`, `LocalMotion`, `EstimatorPose`, `localMotion()`, and `propagatePose()`.

- [ ] **Step 1: Add failing geometry and motion tests**

Use this verified test geometry rather than robot constants:

```cpp
const TrackingGeometry geometry{-4.0, 4.0, -3.0};
```

Add tests for:

```cpp
checkMotion(localMotion({0, 0, 0, true, true, true}, geometry), 0, 0, 0);
checkMotion(localMotion({12, 12, 0, true, true, true}, geometry), 0, 12, 0);
checkMotion(localMotion({-12, -12, 0, true, true, true}, geometry), 0, -12, 0);

const double quarterTurn = kPi / 2.0;
checkMotion(localMotion({4*quarterTurn, -4*quarterTurn,
                         -3*quarterTurn, true, true, true}, geometry),
            0, 0, quarterTurn);

checkMotion(localMotion({0, 0, 5, true, true, true}, geometry), 5, 0, 0);
```

Test field propagation:

```cpp
checkPose(propagatePose({0, 0, 0}, {0, 12, 0, true}), 0, 12, 0);
checkPose(propagatePose({0, 0, kPi/2}, {0, 12, 0, true}), 12, 0, kPi/2);
checkPose(propagatePose({0, 0, 0}, {6, 0, 0, true}), 6, 0, 0);
```

For a constant-radius clockwise quarter arc with center forward velocity and
`dForward = radius*dTheta`, check the exact endpoint against the equation in the
design spec. Add one combined lateral/forward/rotation case calculated directly
from the same closed-form equation.

- [ ] **Step 2: Run tests and verify RED**

Add `src\aon\pose-estimator.cpp` to the PowerShell compile command.

Expected: FAIL because the pose-estimator interface does not exist.

- [ ] **Step 3: Implement the pure motion model**

Declare:

```cpp
struct TrackingGeometry {
  double leftOffsetInches;
  double rightOffsetInches;
  double backOffsetInches;
};

using WheelDeltas = WheelDistances;
struct LocalMotion {
  double rightInches;
  double forwardInches;
  double headingRadians;
  bool lateralValid;
};
struct EstimatorPose { double xInches, yInches, headingRadians; };

LocalMotion localMotion(WheelDeltas wheels,
                        TrackingGeometry geometry) noexcept;
EstimatorPose propagatePose(EstimatorPose pose,
                            LocalMotion motion) noexcept;
```

Return a non-finite/invalid result if either longitudinal wheel is invalid, the
offset separation is non-finite, or `rightOffset-leftOffset` is near zero. When
the back wheel is invalid, set right motion to zero and `lateralValid=false`;
do not invent lateral displacement. Implement the exact equations from the
approved design.

- [ ] **Step 4: Run tests and verify GREEN**

Run: `& tools\test-localization.ps1`

Expected: all zero, straight, reverse, rotation, arc, lateral, and combined
motion tests pass.

- [ ] **Step 5: Commit the corrected raw motion model**

```powershell
git add -- include/aon/odometry/pose-estimator.hpp `
  src/aon/pose-estimator.cpp tests/localization-math-test.cpp `
  tools/test-localization.ps1
git diff --cached --check
git commit -m "feat(localization): add three-wheel pose propagation"
```

---

### Task 3: Add EKF Prediction and Fixed-Size Covariance

**Files:**
- Create: `include/aon/odometry/ekf.hpp`
- Create: `src/aon/ekf.cpp`
- Modify: `tests/localization-math-test.cpp`
- Modify: `tools/test-localization.ps1`

**Interfaces:**
- Consumes: `EstimatorPose`, `LocalMotion`, `propagatePose()`.
- Produces: `Ekf`, `EkfConfig`, `CovarianceDiagonal`, `reset()`, `predict()`, `pose()`, and `covarianceDiagonal()`.

- [ ] **Step 1: Write failing prediction and covariance tests**

Define configuration using variances, not standard deviations:

```cpp
EkfConfig config{
  .initialPositionVariance = 4.0,
  .initialHeadingVariance = square(radians(5.0)),
  .stationaryPositionVariance = 1e-6,
  .stationaryHeadingVariance = 1e-8,
  .positionVariancePerInch = 0.01,
  .headingVariancePerRadian = 0.01,
  .imuHeadingVariance = square(radians(2.0)),
  .gpsPositionVariance = 4.0,
  .gpsHeadingVariance = square(radians(8.0)),
  .singularityTolerance = 1e-12,
};
```

Add tests that:

- reset produces the requested pose and initial diagonal;
- zero prediction leaves pose unchanged and increases covariance by the floor;
- straight prediction advances `+Y` at heading zero;
- a combined prediction matches `propagatePose()`;
- covariance remains finite, symmetric, and has a non-negative diagonal for
  10,000 alternating small motions;
- a non-finite input returns `false` and preserves the last state.

- [ ] **Step 2: Run tests and verify RED**

Add `src\aon\ekf.cpp` to `tools/test-localization.ps1`.

Expected: FAIL because `Ekf` is undefined.

- [ ] **Step 3: Implement explicit 3x3 prediction**

Use fixed storage:

```cpp
using Matrix3 = std::array<std::array<double, 3>, 3>;

class Ekf {
 public:
  explicit Ekf(EkfConfig config) noexcept;
  void reset(EstimatorPose pose) noexcept;
  bool predict(LocalMotion motion) noexcept;
  EstimatorPose pose() const noexcept;
  Matrix3 covariance() const noexcept;
  CovarianceDiagonal covarianceDiagonal() const noexcept;
 private:
  EkfConfig config_;
  EstimatorPose state_{};
  Matrix3 covariance_{};
};
```

Implement `F*P*F^T` with bounded loops over the compile-time dimension three.
Set `Qxx=Qyy=stationaryPositionVariance +
positionVariancePerInch*hypot(dRight,dForward)` and
`Qtheta=stationaryHeadingVariance +
headingVariancePerRadian*abs(dTheta)`. Normalize heading and validate the entire
candidate state/covariance before committing it.

- [ ] **Step 4: Run tests and verify GREEN**

Run: `& tools\test-localization.ps1`

Expected: prediction and long stationary/motion covariance tests pass.

- [ ] **Step 5: Commit EKF prediction**

```powershell
git add -- include/aon/odometry/ekf.hpp src/aon/ekf.cpp `
  tests/localization-math-test.cpp tools/test-localization.ps1
git diff --cached --check
git commit -m "feat(localization): add EKF pose prediction"
```

---

### Task 4: Add Joseph-Form IMU Heading Correction

**Files:**
- Modify: `include/aon/odometry/ekf.hpp`
- Modify: `src/aon/ekf.cpp`
- Modify: `tests/localization-math-test.cpp`

**Interfaces:**
- Consumes: Task 3 predicted state and covariance.
- Produces: `bool Ekf::updateImuHeading(double headingRadians) noexcept`.

- [ ] **Step 1: Write failing IMU update tests**

Add tests that:

- a `359` degree prediction updated with `1` degree moves through the two-degree
  shortest innovation rather than across 358 degrees;
- `-179` updated with `179` uses a negative two-degree innovation;
- low `imuHeadingVariance` moves heading more than a high-noise configuration;
- repeated stationary IMU measurements reduce heading covariance but keep it
  finite and non-negative;
- a NaN measurement and a near-singular innovation return `false` without
  changing state or covariance.

- [ ] **Step 2: Run tests and verify RED**

Expected: FAIL because `updateImuHeading` does not exist.

- [ ] **Step 3: Implement wrapped scalar correction and Joseph covariance**

Calculate:

```text
innovation = shortestAngleDelta(state.theta, measurement)
S = P[2][2] + Rimu
K[i] = P[i][2] / S
state[i] += K[i] * innovation
```

Build `A=I-KH`, then calculate `A*P*A^T + K*R*K^T`. Normalize heading,
symmetrize mirrored covariance entries by averaging, and clamp only diagonal
values in `[-singularityTolerance, 0)` to zero. Validate the candidate before
publishing it.

- [ ] **Step 4: Run tests and verify GREEN**

Run: `& tools\test-localization.ps1`

Expected: all wrap, confidence, and stationary covariance tests pass.

- [ ] **Step 5: Commit IMU correction**

```powershell
git add -- include/aon/odometry/ekf.hpp src/aon/ekf.cpp `
  tests/localization-math-test.cpp
git diff --cached --check
git commit -m "feat(localization): fuse IMU heading with Joseph update"
```

---

### Task 5: Add GPS Preprocessing, Outlier Gates, and EKF Updates

**Files:**
- Modify: `include/aon/odometry/sensor-measurements.hpp`
- Modify: `include/aon/odometry/ekf.hpp`
- Modify: `src/aon/ekf.cpp`
- Modify: `tests/localization-math-test.cpp`

**Interfaces:**
- Produces: `GpsValidationConfig`, `GpsRejectionReason`, `GpsGate`, `Ekf::updateGpsPosition()`, and `Ekf::updateGpsHeading()`.
- Consumes: GPS values already converted to inches and radians.

- [ ] **Step 1: Write failing GPS gate tests**

Create a deterministic gate configuration in tests:

```cpp
GpsValidationConfig gateConfig{
  .minimumXInches = -72.0,
  .maximumXInches = 72.0,
  .minimumYInches = -72.0,
  .maximumYInches = 72.0,
  .maximumReportedErrorInches = 6.0,
  .maximumJumpInches = 18.0,
  .maximumHeadingJumpRadians = radians(45.0),
  .minimumSamplePeriodMs = 50,
  .maximumPositionNis = 9.21,
  .maximumHeadingNis = 6.63,
};
```

Test typed rejection for non-finite values, stale timestamps, excessive sensor
error, field bounds, impossible position jump, and impossible heading jump.
Test that temporary loss does not invalidate the last fused state and that a
later valid sample is accepted.

- [ ] **Step 2: Write failing GPS filter tests**

Add tests that:

- valid GPS X/Y moves a drifted estimate toward the measurement without
  overwriting it exactly;
- a large normalized innovation is rejected;
- a valid correction reduces `Pxx` and `Pyy`;
- GPS heading uses wrapped innovation;
- disabling GPS heading leaves theta unchanged;
- a singular two-by-two innovation matrix is rejected safely.

- [ ] **Step 3: Run tests and verify RED**

Expected: FAIL because GPS gate/update interfaces do not exist.

- [ ] **Step 4: Implement basic gates and explicit GPS equations**

`GpsGate` retains only the last accepted sample and timestamp. Basic gates run
before the filter. `Ekf::updateGpsPosition` computes the explicit symmetric 2x2
inverse:

```text
det = S00*S11 - S01*S10
invS = [ S11 -S01; -S10 S00 ] / det
K = P*H^T*invS
```

Calculate position NIS as `innovation^T*invS*innovation` before applying the
Joseph update. GPS heading reuses the scalar Joseph implementation with its own
noise and NIS limit. Do not share IMU noise or acceptance state.

- [ ] **Step 5: Run tests and verify GREEN**

Run: `& tools\test-localization.ps1`

Expected: GPS loss, stale sample, outlier, drift correction, and covariance
tests pass.

- [ ] **Step 6: Commit GPS math without enabling hardware**

```powershell
git add -- include/aon/odometry/sensor-measurements.hpp `
  include/aon/odometry/ekf.hpp src/aon/ekf.cpp `
  tests/localization-math-test.cpp
git diff --cached --check
git commit -m "feat(localization): add gated GPS corrections"
```

---

### Task 6: Put Geometry, Noise, Timing, and GPS Policy in Robot Configuration

**Files:**
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `tests/hardware-map-test.cpp`
- Create: `include/aon/odometry/diagnostics.hpp`

**Interfaces:**
- Produces: `LocalizationConfig RobotConfig::localization` for both robots and fixed diagnostic enums/types.
- Consumes: Tasks 1-5 configuration types.

- [ ] **Step 1: Add failing configuration assertions**

In `tests/hardware-map-test.cpp`, assert for both configurations:

```cpp
CHECK(config.localization.loopPeriodMs == 10U);
CHECK(config.localization.geometry.leftOffsetInches < 0.0);
CHECK(config.localization.geometry.rightOffsetInches > 0.0);
CHECK(config.localization.geometry.backOffsetInches < 0.0);
CHECK(config.localization.trackingWheelDiameterInches > 0.0);
CHECK(!config.localization.fusedLemLibAuthorized);
CHECK(!config.localization.gps.enabled);
CHECK(config.localization.gps.port == 0);
CHECK(config.localization.ekf.imuHeadingVariance > 0.0);
```

Also assert that big-robot right reversal remains unchanged and still produces
the existing mismatch result.

- [ ] **Step 2: Run the hardware-map host test and verify RED**

Use the existing hardware-map compile command from
`docs/superpowers/plans/2026-08-03-hardware-map-consistency.md`.

Expected: FAIL because `RobotConfig::localization` does not exist.

- [ ] **Step 3: Add explicit localization configuration**

Declare:

```cpp
struct GpsHardwareConfig {
  bool enabled;
  std::int8_t port;
  double xOffsetMeters;
  double yOffsetMeters;
  double headingOffsetDegrees;
  bool headingUpdateEnabled;
  GpsValidationConfig validation;
};

struct LocalizationConfig {
  TrackingGeometry geometry;
  double trackingWheelDiameterInches;
  std::uint32_t loopPeriodMs;
  EkfConfig ekf;
  GpsHardwareConfig gps;
  bool fusedLemLibAuthorized;
};
```

Move the signed LemLib tracking offsets into this shared configuration source;
have `LemLibDriveConfig` refer to the same values rather than introducing a
second set. Use existing nominal diameters and offsets, clearly commented as
unverified. Supply conservative positive starting variances in configuration,
not source-local constants. Set GPS disabled/port zero and fused LemLib
authorization false for both robots.

- [ ] **Step 4: Add fixed diagnostics vocabulary**

Define `LocalizationDiagnostics` with raw/fused pose, wheel/IMU/GPS samples,
`GpsRejectionReason`, covariance diagonal, `dtSeconds`, execution microseconds,
deadline misses, sensor error counters, numerical rejection count, and reset
count. Use only fixed-size scalars and enums.

- [ ] **Step 5: Run host tests and verify GREEN**

Run both `tools/test-localization.ps1` and the hardware-map test executable.

Expected: configuration assertions pass and the big reversal mismatch remains
reported.

- [ ] **Step 6: Commit configuration separately from runtime behavior**

```powershell
git add -- include/aon/config/robot-config.hpp src/aon/config/robot-config.cpp `
  include/aon/odometry/diagnostics.hpp tests/hardware-map-test.cpp
git diff --cached --check
git commit -m "config(localization): expose geometry noise and sensor policy"
```

---

### Task 7: Refactor `aon::Odometry` into One Coherent Estimator

**Files:**
- Modify: `include/aon/odometry/odometry.hpp`
- Modify: `src/aon/odometry.cpp`
- Modify: `include/aon/drivetrain/drivetrain.hpp`
- Modify: `include/aon/drivetrain/differential-drive.hpp`
- Modify: `include/aon/drivetrain/h-drive.hpp`
- Modify: `include/aon/drivetrain/mecanum.hpp`
- Modify: `include/aon/drivetrain/x-drive.hpp`
- Modify: `src/aon/core/hardware.cpp`
- Modify: `src/aon/drivetrain/legacy-motion.cpp`
- Create: `tests/localization-integration-test.cpp`

**Interfaces:**
- Produces: coherent `Odometry::getPose()`, `rawOdometryPose()`, `resetPose()`, `update()`, `getDiagnostics()`, and boot-only `calibrateImu()`.
- Removes: `ENCODER`, `GYRO`, `changeWeb`, the state-dropping copy constructor, separate position/orientation mutexes, and blocking `gpsPosition()`.

- [ ] **Step 1: Add source-boundary and public-interface tests**

Make `tests/localization-integration-test.cpp` read the header/source text and
assert:

- public `getPose`, `rawOdometryPose`, `resetPose`, `update`, and
  `getDiagnostics` declarations exist;
- `changeWeb`, `GYRO_CONFIDENCE`, `pros::delay(3000)`, `pros::delay(2000)`, and
  `gpsPosition()` no longer occur in odometry files;
- exactly one `pros::Mutex` protects the published estimator snapshot;
- the back encoder is sampled by update code;
- `getPose()` is not assembled through `getX()`, `getY()`, and `getDegrees()`.

Compile/run this test as a second executable from `tools/test-localization.ps1`.

- [ ] **Step 2: Run integration test and verify RED**

Expected: FAIL on the existing legacy interface and source patterns.

- [ ] **Step 3: Replace legacy state with fixed estimator state**

Change the constructor to:

```cpp
Odometry(const config::LocalizationConfig& config,
         std::int8_t leftPort, std::int8_t rightPort,
         std::int8_t backPort, std::int8_t imuPort);
```

The class owns the three rotations and IMU, an `std::optional<pros::Gps>` that
is emplaced only when GPS configuration is enabled, wheel baselines, IMU field
offset, raw pose, `Ekf`, `GpsGate`, one published snapshot, one generation
counter, and one mutex. Optional GPS construction performs no dynamic
allocation. Delete the copy constructor. `update()`:

1. records start time;
2. reads each device once;
3. validates PROS error sentinels/status;
4. converts cumulative wheel values to inches and calculates increments;
5. derives `LocalMotion`;
6. propagates raw pose;
7. predicts EKF from wheel motion;
8. applies IMU update when valid;
9. samples/fuses fresh GPS only when enabled;
10. builds diagnostics locally;
11. takes one mutex and publishes only if the reset generation still matches;
12. records execution time/deadline counters without printing.

At the beginning of an update, briefly lock and copy the estimator state,
baselines, and generation into local fixed-size values. Perform sensor reads and
math without the lock. On publication, reacquire the same mutex and discard the
candidate if a concurrent reset incremented the generation. This prevents an
older update from overwriting a reset while keeping device calls and EKF math
outside the getter critical section.

Keep `getX/Y/Degrees` as compatibility wrappers around one `getPose()` call.
Replace `SetPosition`, `setDegrees`, and `setRadians` callers with `resetPose`;
do not retain partial-state setters on `Odometry`.

- [ ] **Step 4: Make reset nonblocking and coherent**

`resetPose` reads the current sensors before taking the snapshot mutex. It then
takes the mutex, increments the generation, installs those wheel baselines and
IMU rotation, and computes:

```text
imuFieldOffset = requestedHeadingRadians - currentImuRotationRadians
```

then resets raw pose, EKF, GPS acceptance history, diagnostics, and publishes
the complete requested pose. It does not tare, calibrate, or sleep.

- [ ] **Step 5: Remove duplicate Odometry ownership from drivetrain**

Change drivetrain constructors from `std::unique_ptr<Odometry>` to a non-owning
`Odometry&`. Store `Odometry& odometry`, remove the stale `Drivetrain::pose`
member, and make:

```cpp
Pose getPose() { return odometry.getPose(); }
double getX() { return getPose().x; }
double getY() { return getPose().y; }
double getTheta() { return getPose().theta; }
void resetPose(double x, double y, double theta) {
  odometry.resetPose(x, y, theta);
}
```

If compatibility `setX/Y/Theta` remains required, each takes one pose snapshot
and calls `resetPose` with the other two fields unchanged. Update `Hardware` to
pass its one `odometry` object by reference. Native lifecycle starts one task
that calls `update()` on a deadline grid; repeated prepare calls remain safe.

- [ ] **Step 6: Run host checks and both builds**

Run:

```powershell
& tools\test-localization.ps1
```

Then build the active small configuration with the configured PROS executable.
Temporarily switch `USING_BIG_ROBOT` to true using `apply_patch`, clean-build,
restore it to false with `apply_patch`, and clean-build again. Do not use
`git checkout`, `git restore`, or a generated rewrite to restore the header.

Expected: both builds pass; final worktree contains `USING_BIG_ROBOT false`.

- [ ] **Step 7: Commit the coherent raw estimator**

Stage only the files listed in this task, inspect the staged diff, then:

```powershell
git commit -m "refactor(localization): publish coherent raw and fused poses"
```

---

### Task 8: Centralize Physical Tracking Sensors Without Enabling Fusion

**Files:**
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `include/aon/odometry/odometry.hpp`
- Modify: `src/aon/core/hardware.cpp`
- Modify: `tests/hardware-map-test.cpp`
- Modify: `tests/localization-integration-test.cpp`

**Interfaces:**
- Produces: one physical left/right/back/IMU owner shared by AON localization,
  LemLib tracking adapters, calibration reporting, and motion-health sampling.
- Preserves: existing LemLib odometry behavior while `fusedLemLibAuthorized` is false.

- [ ] **Step 1: Add failing ownership assertions**

Assert by source inspection that `pros::Rotation` and `pros::Imu` for tracking
are constructed in one production module only, and that `chassis.cpp` obtains
references rather than constructing duplicate static devices. Keep motor-group
ownership assertions separate because this task does not move drive motors.

- [ ] **Step 2: Run test and verify RED**

Expected: FAIL because `Odometry` and `chassis.cpp` both construct sensor
wrappers over the same ports.

- [ ] **Step 3: Expose non-owning sensor access from the hardware-owned estimator**

Keep physical tracking devices owned by `Hardware::odometry` during the legacy
migration. Add narrow accessors returning `pros::Rotation&` and `pros::Imu&`
only for the LemLib integration adapter. `chassis.cpp` constructs its existing
`lemlib::TrackingWheel` objects from those references. Calibration and
`sampleDriveSensors()` use the same references.

Do not start the AON update loop in normal LemLib mode yet. With authorization
false, `initializeChassis()` must execute the existing LemLib calibration and
pose behavior exactly as before.

- [ ] **Step 4: Run both builds and existing motion tests**

Run localization, hardware-map, motion-fallback, and LemLib validation host
tests. Clean-build both robot configurations and restore small.

- [ ] **Step 5: Commit sensor ownership separately**

```powershell
git add -- src/aon/lemlib/chassis.cpp include/aon/odometry/odometry.hpp `
  src/aon/core/hardware.cpp tests/hardware-map-test.cpp `
  tests/localization-integration-test.cpp
git diff --cached --check
git commit -m "refactor(localization): share one tracking sensor owner"
```

---

### Task 9: Add Deterministic Scheduling and Opt-In Diagnostics

**Files:**
- Modify: `include/aon/odometry/odometry.hpp`
- Modify: `src/aon/odometry.cpp`
- Modify: `src/aon/drivetrain/legacy-motion.cpp`
- Modify: `include/aon/tools/logging.hpp`
- Modify or Create: `src/aon/tools/logging.cpp` only if the existing logger has a source implementation
- Modify: `tests/localization-integration-test.cpp`

**Interfaces:**
- Produces: `runLocalizationLoop()`, deadline metrics, and explicit snapshot CSV formatting outside the update loop.
- Consumes: fixed `LocalizationDiagnostics` from Task 6.

- [ ] **Step 1: Add failing scheduling and no-spam assertions**

Assert that the localization loop uses `pros::delay_until` with the configured
period, `update()` contains no `pros::delay`, and neither `update()` nor the held
snapshot lock contains `printf`, `cout`, LCD output, controller output, or file
I/O.

- [ ] **Step 2: Run test and verify RED**

Expected: FAIL until deadline scheduling replaces the legacy relative loop.

- [ ] **Step 3: Implement one idempotent task owner**

Move the never-returning loop out of `Odometry::initialize()`. Provide an
idempotent localization task start function with one process-lifetime task. Its
loop keeps a `std::uint32_t wake = pros::millis()`, calls `update()`, records a
miss when execution passes the next release, and calls:

```cpp
pros::delay_until(&wake, config.loopPeriodMs);
```

`legacy_motion::prepare()` ensures this task is running instead of starting a
second estimator.

- [ ] **Step 4: Add explicit diagnostic formatting**

Add a caller-driven function that accepts a previously copied diagnostic
snapshot and writes one CSV header or row. It must never be called from
`update()`. Fields are:

```text
time_ms,raw_x,raw_y,raw_theta,fused_x,fused_y,fused_theta,
imu_theta,imu_valid,gps_x,gps_y,gps_theta,gps_valid,gps_reason,
p_x,p_y,p_theta,dt_s,execution_us,deadline_misses
```

Do not open `/usd` files here. Direct SD ownership remains with the existing SD
architecture and is deferred unless a later explicit logger adapter is needed.

- [ ] **Step 5: Run tests and both builds**

Expected: source-boundary checks, host math, and both configurations pass.

- [ ] **Step 6: Commit deterministic runtime behavior**

```powershell
git add -- include/aon/odometry/odometry.hpp src/aon/odometry.cpp `
  src/aon/drivetrain/legacy-motion.cpp include/aon/tools/logging.hpp `
  src/aon/tools/logging.cpp tests/localization-integration-test.cpp
git diff --cached --check
git commit -m "feat(localization): schedule deterministic diagnostic updates"
```

Omit `src/aon/tools/logging.cpp` from staging if the existing header-only logger
remains the appropriate project pattern.

---

### Task 10: Build the LemLib Fused-Pose Adapter Behind Authorization

**Files:**
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `include/aon/lemlib/chassis.hpp`
- Modify: `src/aon/core/robot.cpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: `tests/localization-integration-test.cpp`
- Modify: `tests/lemlib-validation-test.cpp` if its fake interface needs pose-source visibility

**Interfaces:**
- Produces: authorized fused mode in which AON is the only wheel/IMU pose writer and LemLib motion consumes `Odometry::getPose()` through `Chassis::setPose()`.
- Preserves: current LemLib sensor ownership and odometry path when authorization is false.

- [ ] **Step 1: Add failing mode-boundary tests**

Assert that:

- configuration false selects the existing LemLib `OdomSensors` set;
- configuration true supplies null LemLib odometry sensors and calibrates LemLib
  with `calibrate(false)` after AON performs the one blocking boot IMU reset;
- only `chassis.cpp` calls LemLib `setPose` from the localization publisher;
- `Actions::setPose` resets AON first and then publishes the same snapshot to
  LemLib without taring;
- normal mode and fused mode cannot start two AON localization tasks.

- [ ] **Step 2: Run tests and verify RED**

Expected: FAIL because fused mode does not exist.

- [ ] **Step 3: Implement the gated construction paths**

At static chassis construction, choose once from immutable configuration:

```text
authorization false -> current 3 TrackingWheel + IMU OdomSensors
authorization true  -> OdomSensors(nullptr, nullptr, nullptr, nullptr, nullptr)
```

In authorized mode:

1. call `odometry.calibrateImu()` once at boot and check success;
2. call `chassis.calibrate(false)` so LemLib does not recalibrate the IMU;
3. reset AON and LemLib to the same initial pose;
4. start the one AON deadline task;
5. after each AON update, copy one fused snapshot and call LemLib `setPose`;
6. keep route bodies and LemLib controllers unchanged.

The publisher must never call back into `Odometry` while holding the Odometry
snapshot mutex. AON publishes internally, releases its mutex, then the adapter
copies and publishes to LemLib.

- [ ] **Step 4: Keep authorization false and run the regression matrix**

Run all localization, hardware, motion-fallback, LemLib validation, Shadow, and
route tests. Clean-build both configurations. Confirm the false path has the
same LemLib sensor configuration and startup calls as before.

- [ ] **Step 5: Commit the inactive fused adapter**

```powershell
git add -- src/aon/lemlib/chassis.cpp include/aon/lemlib/chassis.hpp `
  src/aon/core/robot.cpp src/aon/auton/actions.cpp `
  tests/localization-integration-test.cpp tests/lemlib-validation-test.cpp
git diff --cached --check
git commit -m "feat(localization): add gated LemLib fused pose adapter"
```

Do not change `fusedLemLibAuthorized` in this commit.

---

### Task 11: Physical Baseline and Fused Integration Gate

**Files:**
- Modify: `docs/CURRENT_HANDOFF.md`
- Create: `docs/testing/localization-benchmark.md`
- Modify: `src/aon/config/robot-config.cpp` only after measurements pass

**Interfaces:**
- Produces: recorded baseline, measured sensor signs/geometry, proof of one pose writer, and authorization decision.
- Consumes: Tasks 1-10 with fused mode still false.

- [ ] **Step 1: Record the repository's existing pending baseline first**

On a fresh small-robot boot, run the currently required LemLib figure-eight and
one native Kevin fallback exactly as specified by `docs/CURRENT_HANDOFF.md`.
Record completion, final pose, crossover behavior, GUI behavior, mechanisms,
and cancellation. A failure stops this task; do not enable fused mode.

- [ ] **Step 2: Measure tracking signs and offsets without changing constants**

Use the existing sensor and full-turn calibration modes. For forward 24 inches,
right/left tracking distances must have the configured forward sign and the back
wheel should remain near zero. For one clockwise and one counterclockwise full
turn, record left/right/back distance and IMU rotation. Calculate signed offsets
from `wheelDelta / imuDeltaRadians` in both directions and average only results
whose magnitudes and signs repeat.

- [ ] **Step 3: Enable fused mode on an isolated validation build**

Change only small-robot `fusedLemLibAuthorized` to true. Upload and perform, in
order:

1. stationary 30 seconds;
2. runtime reset repeated five times;
3. forward and reverse 12 inches at reduced speed;
4. clockwise and counterclockwise 90-degree turns;
5. drive-turn-drive;
6. controller-X cancellation.

Log raw/fused pose, covariance, task deadline misses, and direct sensor values.
Confirm that LemLib pose matches the fused snapshot at every sampled point and
does not move while AON publication is paused in a diagnostic build. Any
independent LemLib pose drift proves a second writer and fails the gate.

- [ ] **Step 4: Decide authorization from evidence**

Pass criteria:

- no second pose writer is observed;
- stationary fused displacement remains within the measured sensor noise band;
- all five resets complete without IMU calibration or multi-second blocking;
- direction conventions match `+X` right, `+Y` forward, clockwise positive;
- no non-finite covariance or sensor state occurs;
- cancellation and existing action monitoring still stop motion;
- update deadline misses are zero during isolated primitives or are explained
  and corrected before continuing.

If any criterion fails, restore authorization false, commit only the recorded
failure report if useful, and stop for redesign. Do not compensate with guessed
signs, inflated noise, or repeated `setPose` calls.

- [ ] **Step 5: Commit measured geometry and authorization separately**

First commit each proven geometry/sign category independently. After rerunning
the gate with final measured configuration, commit authorization and records:

```powershell
git add -- src/aon/config/robot-config.cpp docs/CURRENT_HANDOFF.md `
  docs/testing/localization-benchmark.md
git diff --cached --check
git commit -m "config(localization): authorize measured small robot fusion"
```

Repeat the complete measurement and authorization procedure separately for the
big robot. Never copy small-robot values into the big configuration.

---

### Task 12: Configure and Validate GPS Without Automatic Trust

**Files:**
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `docs/testing/localization-benchmark.md`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Produces: measured optional GPS hardware configuration and position fusion.
- Consumes: physically authorized wheel/IMU fused mode from Task 11.

- [ ] **Step 1: Measure and record GPS installation**

Record smart port, sensor position relative to tracking center in meters,
mounting heading offset, field origin, and initial robot pose. Confirm PROS
reported error and heading behavior at multiple fixed field locations. If no GPS
is installed, record that fact and leave `enabled=false`; the task is complete
without inventing a port.

- [ ] **Step 2: Enable GPS position only**

Set the measured port/offsets and `gps.enabled=true`, but keep
`headingUpdateEnabled=false`. Use broad, explicitly recorded initial gates based
on stationary and manual-relocation measurements. Do not derive bounds from
autonomous targets alone.

- [ ] **Step 3: Validate loss, outliers, and recovery**

Run stationary, straight, square, temporary sensor obstruction/disconnection,
large relocation while disabled, and correction-after-dead-reckoning-drift
tests. Verify every accepted/rejected sample and rejection reason. GPS loss must
not stop wheel/IMU localization or make covariance non-finite.

- [ ] **Step 4: Tune noise from repeated residuals**

Use at least twenty stationary and repeated-path samples to calculate position
residual variance. Update only `gpsPositionVariance` and one gate category per
checkpoint. Rerun host tests, both builds, and the physical subset after each
change.

- [ ] **Step 5: Evaluate GPS heading separately**

Compare at least five runs with GPS heading disabled and enabled. Authorize it
only if mean or worst heading error and repeatability improve without additional
outlier-induced corrections. Otherwise leave it implemented but disabled and
record the result.

- [ ] **Step 6: Commit measured GPS configuration**

```powershell
git add -- src/aon/config/robot-config.cpp `
  docs/testing/localization-benchmark.md docs/CURRENT_HANDOFF.md
git diff --cached --check
git commit -m "config(localization): apply measured GPS fusion settings"
```

---

### Task 13: Final Regression, Benchmark Comparison, and Documentation

**Files:**
- Modify: `README.md`
- Modify: `docs/CURRENT_HANDOFF.md`
- Modify: `docs/testing/localization-benchmark.md`
- Modify: `include/aon/odometry/odometry.hpp` comments only if final behavior differs from its documented contract

**Interfaces:**
- Documents the stable localization interface, tuning values, limitations, and measured A/B/C results.

- [ ] **Step 1: Run the complete host suite**

Run every existing host test plus `tools/test-localization.ps1`, with warnings
treated as errors. Run PowerShell source/architecture checks. Fix only
regressions caused by localization changes.

- [ ] **Step 2: Clean-build both configurations**

Clean-build small, clean-build big, restore small, and clean-build small again.
Run `git diff --check` and confirm no generated binaries are staged.

- [ ] **Step 3: Execute the repeatable A/B/C benchmark**

For each robot and each available mode, run at least five trials of:

- straight 72-inch forward and reverse;
- clockwise and counterclockwise 90-degree turns;
- square path;
- constant-radius arc;
- figure eight;
- return-to-start;
- lateral and combined motion on the H-drive.

Compare:

```text
A: preserved LemLib/current baseline
B: corrected raw three-wheel pose
C: EKF fused pose
```

Record per-run final X error, Y error, heading error, duration, deadline misses,
GPS accepted/rejected counts, and covariance diagonal. Calculate mean, standard
deviation, maximum absolute error, and repeatability range.

- [ ] **Step 4: Document the result without overstating it**

Update README and handoff with the public interface, coordinate/unit contract,
enabled sensors, measured geometry, noise settings, known limitations, reset
behavior, and exact validation status. State that EKF improves localization only
if the C results outperform A and B in the recorded metrics.

- [ ] **Step 5: Commit final verified documentation**

```powershell
git add -- README.md docs/CURRENT_HANDOFF.md `
  docs/testing/localization-benchmark.md `
  include/aon/odometry/odometry.hpp
git diff --cached --check
git commit -m "docs(localization): record fused pose validation results"
```

---

## Final Acceptance Checklist

- [ ] Three-wheel signs, offsets, and units are explicit and physically measured.
- [ ] Lateral/back-wheel motion participates in raw and fused propagation.
- [ ] Angle crossings never create false large deltas.
- [ ] Raw and fused poses use one coordinate convention.
- [ ] Pose snapshots contain mutually consistent X, Y, and heading.
- [ ] Runtime reset does not tare, calibrate, or sleep.
- [ ] One deadline-scheduled task owns localization updates.
- [ ] No high-frequency dynamic allocation or logging exists.
- [ ] EKF prediction and all measurement updates use fixed-size equations.
- [ ] Joseph-form covariance remains finite, symmetric, and non-negative.
- [ ] IMU and GPS failures degrade cleanly without corrupting pose.
- [ ] GPS never overwrites pose and is disabled without measured hardware data.
- [ ] LemLib has no competing pose writer in fused mode.
- [ ] Existing autonomous route bodies remain unchanged.
- [ ] Motion cancellation, health monitoring, Shadow recording, and native compatibility still work.
- [ ] All host tests and both robot builds pass.
- [ ] Physical benchmark results, assumptions, and tuning values are recorded.
- [ ] No claim of EKF improvement exceeds the measurements.
