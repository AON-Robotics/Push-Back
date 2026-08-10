# Fused Localization and LiDAR Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a host-tested, fixed-capacity known-field localization and obstacle-aware navigation stack in which the V5 owns fused pose and drivetrain safety while a Raspberry Pi converts 2D LiDAR scans into compact corrections, obstacles, and routes.

**Architecture:** Pure C++17 localization, field, navigation, protocol, and LiDAR math remain independent of PROS. A thin V5 adapter samples the already configured tracking wheels and IMU, while a disabled-by-default runtime facade publishes the authoritative estimate. Existing LemLib and native routes remain unchanged until physical gates authorize route-by-route migration.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, fixed-size matrices and containers, MinGW host tests, GNU Make/ARM V5 toolchain, Raspberry Pi serial/LiDAR adapter boundary.

## Global Constraints

- Use inches, seconds, and radians internally; display adapters alone may expose degrees.
- Positive Y is forward at heading zero, positive X is robot-right, and positive heading is clockwise.
- Do not allocate dynamically in V5 update, parser, planner, or follower loops.
- The V5 remains the only drivetrain safety authority; Pi messages never command motors or mechanisms directly.
- Preserve all current autonomous authorization and physical-validation gates.
- Keep new competition execution disabled until host, build, mounting, calibration, and physical validation gates pass.
- Reject non-finite, stale, malformed, oversized, statistically inconsistent, or out-of-bounds data explicitly.
- Follow red-green-refactor for every production behavior and make one focused commit per task.

## File Structure

- `include/aon/localization/types.hpp`: units, state, covariance, sensor sample, observation, diagnostics, and status contracts.
- `include/aon/localization/geometry.hpp`: angle wrapping and SE(2) integration helpers.
- `include/aon/localization/tracking-odometry.hpp`, `src/aon/localization/tracking-odometry.cpp`: cumulative tracking-sensor sampling to body motion increments.
- `include/aon/localization/state-estimator.hpp`, `src/aon/localization/state-estimator.cpp`: fixed 6-state EKF prediction, correction, gating, and delayed-observation history.
- `include/aon/localization/confidence.hpp`, `src/aon/localization/confidence.cpp`: covariance-derived confidence and recovery policy.
- `include/aon/localization/manager.hpp`, `src/aon/localization/manager.cpp`: coherent authoritative snapshots and reset semantics.
- `include/aon/field/geometry.hpp`: bounded field geometry primitives and collision queries.
- `include/aon/field/field-map.hpp`, `src/aon/field/field-map.cpp`: immutable known-map validation and lookup.
- `include/aon/field/push-back-field.hpp`, `src/aon/field/push-back-field.cpp`: Push Back field boundary and verified permanent geometry.
- `include/aon/localization/wall-observation.hpp`, `src/aon/localization/wall-observation.cpp`: field-wall range observations with geometry-derived covariance.
- `include/aon/navigation/dynamic-obstacles.hpp`, `src/aon/navigation/dynamic-obstacles.cpp`: fixed-capacity obstacle tracking and expiry.
- `include/aon/navigation/path-planner.hpp`, `src/aon/navigation/path-planner.cpp`: bounded visibility graph and A*.
- `include/aon/navigation/replanner.hpp`, `src/aon/navigation/replanner.cpp`: route-corridor invalidation and rate limiting.
- `include/aon/navigation/path-follower.hpp`, `src/aon/navigation/path-follower.cpp`: fused-pose nonlinear follower core.
- `include/aon/communication/pi-protocol.hpp`, `src/aon/communication/pi-protocol.cpp`: framed binary protocol and bounded streaming parser.
- `include/aon/lidar/scan-processor.hpp`, `src/aon/lidar/scan-processor.cpp`: model-independent scan filtering, wall fitting, and clustering.
- `include/aon/localization/runtime.hpp`, `src/aon/localization/runtime.cpp`: disabled-by-default PROS sampling and telemetry adapter.
- `tests/localization-test.cpp`, `tests/field-model-test.cpp`, `tests/navigation-test.cpp`, `tests/pi-protocol-test.cpp`, `tests/lidar-scan-test.cpp`: deterministic host suites.
- `tools/run-host-tests.ps1`: one command that builds and runs existing and new host suites.

---

### Task 1: Localization value types and SE(2) geometry

**Files:**
- Create: `include/aon/localization/types.hpp`
- Create: `include/aon/localization/geometry.hpp`
- Create: `tests/localization-test.cpp`

**Interfaces:**
- Produces: `Pose2D`, `Velocity2D`, `StateVector6`, `Covariance6`, `SensorSample`, `LocalizationObservation`, `LocalizationEstimate`, `wrapRadians(double)`, and `integrateBodyIncrement(Pose2D, BodyIncrement)`.

- [ ] **Step 1: Write failing value and geometry tests**

```cpp
checkNear(wrapRadians(3.0 * kPi), -kPi);
checkNear(wrapRadians(-3.0 * kPi), -kPi);
const Pose2D straight = integrateBodyIncrement({0, 0, 0}, {0, 12, 0});
CHECK(nearPose(straight, {0, 12, 0}));
const Pose2D arc = integrateBodyIncrement({0, 0, 0}, {0, 10, kPi / 2});
CHECK(std::isfinite(arc.xInches));
CHECK(std::isfinite(arc.yInches));
```

- [ ] **Step 2: Verify RED**

Run the documented MinGW compiler with `-std=c++17 -Wall -Wextra -Werror -Iinclude tests\localization-test.cpp`; expect missing localization headers.

- [ ] **Step 3: Implement fixed value types and stable integration**

Use `std::array<double, 6>` and `std::array<double, 36>` for state/covariance. Implement the small-angle `sinc`/`cosc` limits without heap allocation and reject non-finite increments through an explicit validity result.

- [ ] **Step 4: Verify GREEN and commit**

Run `bin\host-tests\localization-test.exe`; expect `localization tests passed`. Commit `feat(localization): add state and SE2 geometry types`.

### Task 2: Three-wheel odometry sampler

**Files:**
- Create: `include/aon/localization/tracking-odometry.hpp`
- Create: `src/aon/localization/tracking-odometry.cpp`
- Modify: `tests/localization-test.cpp`

**Interfaces:**
- Consumes: cumulative left/right/back distances, IMU heading, timestamps, and `TrackingGeometry`.
- Produces: `TrackingOdometry::update(const SensorSample&) -> OdometryUpdate` and atomic `reset(const SensorSample&)` baselines.

- [ ] **Step 1: Add failing tests for straight, rotation, lateral, curve, wrap, disconnect, and reset**

```cpp
TrackingOdometry odom({1.125, 1.125, 1.572});
CHECK(odom.reset(sampleAt(0, 0, 0, 350_deg)) == SampleResult::Accepted);
const auto update = odom.update(sampleAt(20, 2, 2, 0, 10_deg));
CHECK(update.accepted);
CHECK(update.increment.forwardInches > 1.9);
CHECK(std::abs(update.increment.headingRadians - radians(20)) < 1e-6);
```

- [ ] **Step 2: Verify RED, implement the minimum kinematics, and verify GREEN**

Compute rotation from wrapped IMU delta when valid, fall back to left/right geometry only when policy permits, and compensate all three tracking-wheel offsets. Reject non-monotonic timestamps and invalid sensor combinations.

- [ ] **Step 3: Commit**

Commit `feat(localization): add three-wheel odometry sampler`.

### Task 3: EKF prediction and covariance growth

**Files:**
- Create: `include/aon/localization/state-estimator.hpp`
- Create: `src/aon/localization/state-estimator.cpp`
- Modify: `tests/localization-test.cpp`

**Interfaces:**
- Produces: `StateEstimator::reset`, `predict(const OdometryUpdate&)`, `estimate()`, and fixed `EstimatorNoise` configuration.

- [ ] **Step 1: Add failing prediction tests**

```cpp
StateEstimator estimator(defaultNoise());
estimator.reset({0, 0, 0}, initialCovariance(), 0);
CHECK(estimator.predict(straightUpdate(20, 1.0)) == EstimatorResult::Accepted);
CHECK(estimator.estimate().pose.yInches > 0.99);
CHECK(estimator.estimate().velocity.vyInchesPerSecond > 49.0);
CHECK(estimator.estimate().covariance(0, 0) > initialXVariance);
```

- [ ] **Step 2: Verify RED, implement fixed 6-by-6 predict math, and verify GREEN**

Use explicit fixed-array matrix operations, actual bounded `dt`, a kinematic Jacobian, symmetry restoration, diagonal floors, and translation/rotation/time-dependent process noise.

- [ ] **Step 3: Commit**

Commit `feat(localization): add EKF prediction and covariance`.

### Task 4: Observation correction, innovation gating, and delayed history

**Files:**
- Modify: `include/aon/localization/state-estimator.hpp`
- Modify: `src/aon/localization/state-estimator.cpp`
- Modify: `tests/localization-test.cpp`

**Interfaces:**
- Produces: `correct(const LocalizationObservation&) -> CorrectionResult`, `CorrectionDiagnostics`, observation sequence tracking, and a fixed-capacity prediction history.

- [ ] **Step 1: Add failing tests for position, heading, pose, wall-axis, outlier, duplicate, stale, and delayed corrections**

```cpp
const auto accepted = estimator.correct(positionObservation(100, 0.5, 11.5));
CHECK(accepted.status == CorrectionStatus::Accepted);
CHECK(accepted.normalizedInnovationSquared < accepted.gateThreshold);
CHECK(estimator.estimate().covariance(0, 0) < beforeVariance);
CHECK(estimator.correct(positionObservation(100, 500, 500)).status ==
      CorrectionStatus::Duplicate);
```

- [ ] **Step 2: Verify RED, implement scalar/2D/3D corrections and bounded replay, then verify GREEN**

Normalize heading innovations, invert only matrices up to 3-by-3, use configured chi-square gates, apply Joseph-form covariance updates, and reject observations older than retained history.

- [ ] **Step 3: Commit**

Commit `feat(localization): gate and replay corrective observations`.

### Task 5: Confidence, recovery, and authoritative manager

**Files:**
- Create: `include/aon/localization/confidence.hpp`
- Create: `src/aon/localization/confidence.cpp`
- Create: `include/aon/localization/manager.hpp`
- Create: `src/aon/localization/manager.cpp`
- Modify: `tests/localization-test.cpp`

**Interfaces:**
- Produces: `localizationConfidence(const LocalizationEstimate&)`, `RecoveryMonitor::observe`, `LocalizationManager::update`, `reset`, and coherent `snapshot()`.

- [ ] **Step 1: Add failing policy and displacement-recovery tests**

```cpp
CHECK(classifyConfidence(tightEstimate, policy) == ConfidenceLevel::Normal);
CHECK(classifyConfidence(wideEstimate, policy) == ConfidenceLevel::StopRequired);
CHECK(!recovery.observe(firstConsistentLargeCorrection).allowLargeCorrection);
CHECK(recovery.observe(thirdConsistentLargeCorrection).allowLargeCorrection);
```

- [ ] **Step 2: Verify RED, implement covariance-derived thresholds and repeated-consistency recovery, then verify GREEN**

Snapshots must be copied under a short critical section; callers never receive mutable estimator references.

- [ ] **Step 3: Commit and run the full localization suite**

Commit `feat(localization): expose confidence and recovery policy`.

### Task 6: Static field geometry and Push Back map

**Files:**
- Create: `include/aon/field/geometry.hpp`
- Create: `include/aon/field/field-map.hpp`
- Create: `src/aon/field/field-map.cpp`
- Create: `include/aon/field/push-back-field.hpp`
- Create: `src/aon/field/push-back-field.cpp`
- Create: `tests/field-model-test.cpp`

**Interfaces:**
- Produces: `Segment`, `Circle`, `Rectangle`, `NamedPose`, `FieldMap`, collision/distance queries, and `pushBackField()`.

- [ ] **Step 1: Verify current official field dimensions and encode failing validation tests**

```cpp
const FieldMap& field = pushBackField();
CHECK(field.validate() == FieldMapIssue::None);
CHECK(field.contains({0, 0}, 0.0));
CHECK(!field.contains({80, 0}, 0.0));
CHECK(field.wallCount() == 4);
```

- [ ] **Step 2: Verify RED, implement bounded immutable geometry and only dimension-verified permanent features, then verify GREEN**

Do not infer scoring geometry from photographs. Record the official manual revision in the field source comment and keep uncertain game objects out of collision geometry.

- [ ] **Step 3: Commit**

Commit `feat(field): add validated Push Back field model`.

### Task 7: Wall observations

**Files:**
- Create: `include/aon/localization/wall-observation.hpp`
- Create: `src/aon/localization/wall-observation.cpp`
- Modify: `tests/field-model-test.cpp`

**Interfaces:**
- Produces: `makeWallObservation(const WallRangeMeasurement&, const FieldMap&, const LocalizationEstimate&)`.

- [ ] **Step 1: Add failing tests for valid perpendicular wall sighting, shallow angle, wrong wall, range bounds, and covariance scaling**

```cpp
const auto observation = makeWallObservation(frontWallHit(24.0), field, estimate);
CHECK(observation.status == WallObservationStatus::Accepted);
CHECK(observation.observation.kind == ObservationKind::AxisPosition);
CHECK(shallowWallHit.status == WallObservationStatus::PoorGeometry);
```

- [ ] **Step 2: Verify RED, implement ray/segment geometry and quality-derived variance, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(localization): derive corrections from known walls`.

### Task 8: Dynamic obstacle tracking

**Files:**
- Create: `include/aon/navigation/dynamic-obstacles.hpp`
- Create: `src/aon/navigation/dynamic-obstacles.cpp`
- Create: `tests/navigation-test.cpp`

**Interfaces:**
- Produces: `DynamicObstacleMap<Capacity>::update`, `expire`, `snapshot`, and bounded circle/rectangle obstacles.

- [ ] **Step 1: Add failing tests for association, velocity, confidence, capacity replacement, static rejection, and expiry**

```cpp
DynamicObstacleMap<8> obstacles(config);
CHECK(obstacles.update(circleDetection(1, 0, 100)) == ObstacleResult::Inserted);
CHECK(obstacles.update(circleDetection(2, 0, 120)) == ObstacleResult::Updated);
obstacles.expire(1000);
CHECK(obstacles.size() == 0);
```

- [ ] **Step 2: Verify RED, implement deterministic nearest-valid association and expiry, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(navigation): add bounded dynamic obstacle map`.

### Task 9: Visibility-graph A* planner

**Files:**
- Create: `include/aon/navigation/path-planner.hpp`
- Create: `src/aon/navigation/path-planner.cpp`
- Modify: `tests/navigation-test.cpp`

**Interfaces:**
- Produces: fixed-capacity `Path`, `PlanRequest`, `PlanResult`, and `PathPlanner::plan`.

- [ ] **Step 1: Add failing direct, detour, unreachable, inflated-footprint, bounds, capacity, and deterministic-route tests**

```cpp
const PlanResult direct = planner.plan({start, goal, {}, robotRadius});
CHECK(direct.status == PlanStatus::Success);
CHECK(direct.path.size == 2);
const PlanResult detour = planner.plan(requestWithBlockingRectangle());
CHECK(detour.status == PlanStatus::Success);
CHECK(pathHasClearance(detour.path, obstacles, robotRadius));
```

- [ ] **Step 2: Verify RED, implement bounded visible nodes/edges and A* with deterministic tie-breaking, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(navigation): add bounded visibility graph planner`.

### Task 10: Replanning policy

**Files:**
- Create: `include/aon/navigation/replanner.hpp`
- Create: `src/aon/navigation/replanner.cpp`
- Modify: `tests/navigation-test.cpp`

**Interfaces:**
- Produces: `Replanner::evaluate(const RouteState&, const WorldRevision&, std::uint32_t)` and structured `ReplanReason`.

- [ ] **Step 1: Add failing tests for irrelevant changes, corridor intersection, goal change, pose correction, blocked progress, and rate limit**

```cpp
CHECK(replanner.evaluate(clearRevision, route, 100).reason == ReplanReason::None);
CHECK(replanner.evaluate(blockedRevision, route, 200).reason ==
      ReplanReason::RouteObstructed);
CHECK(replanner.evaluate(blockedRevision, route, 210).reason ==
      ReplanReason::RateLimited);
```

- [ ] **Step 2: Verify RED, implement remaining-corridor checks and event-based throttling, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(navigation): trigger meaningful route replanning`.

### Task 11: Fused-pose path follower core

**Files:**
- Create: `include/aon/navigation/path-follower.hpp`
- Create: `src/aon/navigation/path-follower.cpp`
- Modify: `tests/navigation-test.cpp`

**Interfaces:**
- Consumes: `LocalizationEstimate`, active `Path`, follower limits, elapsed time, cancellation and confidence state.
- Produces: bounded `DriveCommand`, progress, completion, blocked, confidence-stop, and route-replaced statuses.

- [ ] **Step 1: Add failing convergence, saturation, acceleration, cross-track, final-heading, cancellation, confidence, timeout, and route-replacement tests**

```cpp
const FollowerOutput output = follower.update(path, estimate, 0.02);
CHECK(std::abs(output.left) <= limits.maxCommand);
CHECK(std::abs(output.right) <= limits.maxCommand);
CHECK(cancelled.status == FollowerStatus::Cancelled);
CHECK(lowConfidence.status == FollowerStatus::LocalizationUnsafe);
```

- [ ] **Step 2: Verify RED, implement nonlinear pose control and fixed waypoint progression, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(navigation): add fused-pose path follower`.

### Task 12: Versioned Pi protocol

**Files:**
- Create: `include/aon/communication/pi-protocol.hpp`
- Create: `src/aon/communication/pi-protocol.cpp`
- Create: `tests/pi-protocol-test.cpp`

**Interfaces:**
- Produces: bounded `encodeFrame`, streaming `FrameParser::consume`, typed payload codecs, heartbeat/link-health tracker, CRC-32, and protocol version 1.

- [ ] **Step 1: Add failing round-trip, fragmentation, concatenation, corruption, oversize, unknown-version, non-finite, replay, wrap, and resynchronization tests**

```cpp
const EncodedFrame frame = encodeFrame(heartbeat(7, 100));
for (std::uint8_t byte : frame.bytes()) parser.consume(byte);
CHECK(parser.take(message));
CHECK(message.sequence == 7);
CHECK(corruptFrameResult == ParseResult::CrcMismatch);
```

- [ ] **Step 2: Verify RED, implement endian-explicit framing and typed validation, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(protocol): add bounded V5 Pi message framing`.

### Task 13: Model-independent LiDAR scan processing

**Files:**
- Create: `include/aon/lidar/scan-processor.hpp`
- Create: `src/aon/lidar/scan-processor.cpp`
- Create: `tests/lidar-scan-test.cpp`

**Interfaces:**
- Consumes: bounded polar scans, LiDAR mounting transform, V5 pose snapshot, and `FieldMap`.
- Produces: qualified wall observations and bounded obstacle detections; hardware driver remains injected.

- [ ] **Step 1: Add failing tests from synthetic scans for mount transform, range filtering, wall fitting, transparent-wall dropout, covariance, static removal, clustering, and capacity**

```cpp
const ScanResult result = processor.process(syntheticNorthWallScan(), pose);
CHECK(result.wallObservationCount == 1);
CHECK(result.obstacleCount == 0);
const ScanResult blocked = processor.process(scanWithRobotCluster(), pose);
CHECK(blocked.obstacleCount == 1);
```

- [ ] **Step 2: Verify RED, implement bounded line fitting and Euclidean clustering without external dependencies, then verify GREEN**

- [ ] **Step 3: Commit**

Commit `feat(lidar): extract wall corrections and obstacles`.

### Task 14: V5 runtime adapter, telemetry, and inactive integration

**Files:**
- Create: `include/aon/localization/runtime.hpp`
- Create: `src/aon/localization/runtime.cpp`
- Modify: `include/aon/lemlib/drive-io.hpp`
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `src/aon/core/robot.cpp`
- Create: `tests/localization-runtime-policy-test.cpp`

**Interfaces:**
- Produces: `LocalizationRuntimeConfig`, compile-time/runtime logging controls, fixed-rate sampling, authoritative `localization()` facade, and explicit `fusedNavigationAuthorized = false` configuration.

- [ ] **Step 1: Add failing host policy tests proving both robots keep fused navigation unauthorized**

```cpp
CHECK(!smallConfig.fusedNavigationAuthorized);
CHECK(!bigConfig.fusedNavigationAuthorized);
CHECK(runtimePolicy(false, stalePi).action == RuntimeAction::DoNotStart);
```

- [ ] **Step 2: Verify RED, extend sensor sampling and add the gated runtime adapter, then verify GREEN**

The task may publish estimates and diagnostics, but it must not replace current LemLib pose or command motors while authorization is false.

- [ ] **Step 3: Clean-build small and big configurations and restore small**

Require both ARM builds to exit zero. Restore `#define USING_BIG_ROBOT false` before commit.

- [ ] **Step 4: Commit**

Commit `feat(localization): integrate gated V5 runtime`.

### Task 15: Unified verification tooling and deployment documentation

**Files:**
- Create: `tools/run-host-tests.ps1`
- Create: `docs/localization/LIDAR_MOUNTING_AND_CALIBRATION.md`
- Create: `docs/localization/V5_PI_PROTOCOL.md`
- Modify: `README.md`

**Interfaces:**
- Produces: a fail-fast host suite command and explicit physical gates for LiDAR model selection, rigid mounting, transform measurement, serial wiring, time synchronization, wall reflectivity, covariance tuning, recovery, and route authorization.

- [ ] **Step 1: Write the host-test runner and prove it reports failures**

Temporarily invoke the runner with a deliberately invalid compiler path and confirm a nonzero exit; restore the documented compiler discovery before continuing.

- [ ] **Step 2: Run every existing and new host suite**

Run `powershell -ExecutionPolicy Bypass -File tools\run-host-tests.ps1`; require all suites to compile with warnings as errors and exit zero.

- [ ] **Step 3: Clean-build both robot configurations and restore small**

Use the configured PROS 4.2.2 toolchain. Require exit zero for both configurations and verify `USING_BIG_ROBOT false`, `fusedNavigationAuthorized == false`, and existing experimental route gates remain false.

- [ ] **Step 4: Verify scope and documentation**

Run `git diff --check`, inspect `git status --short`, and map every design requirement to a completed task. Document unimplemented hardware facts as physical-gate inputs, not software placeholders.

- [ ] **Step 5: Commit**

Commit `docs: add localization deployment and verification workflow`.

## Physical Activation Gate

Software completion does not authorize competition use. Before changing
`fusedNavigationAuthorized`, the team must record:

1. Exact LiDAR model, electrical interface, scan rate, range limits, and driver.
2. Legal inspection under the active VEX U manual and Q&A.
3. Rigid mounting and measured robot-to-LiDAR X/Y/yaw transform.
4. Static test scans against every relevant field-wall material.
5. Serial throughput, timestamp offset, jitter, dropout, and reconnect results.
6. Tracking-wheel/IMU calibration and stationary covariance logs.
7. Push, slip, and displacement recovery tests with emergency stop available.
8. Obstacle detection and route replanning at reduced speed.
9. Route-by-route autonomous validation for each robot configuration.

Only a separate reviewed checkpoint may enable fused navigation after these
measurements pass.
