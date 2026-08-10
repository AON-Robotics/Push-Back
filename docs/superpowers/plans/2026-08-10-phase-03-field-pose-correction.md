# Phase 3 Field Pose Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct accumulated pose drift using timestamped observations of known field geometry while updating only the dimensions each observation truly constrains.

**Architecture:** A pure field model and observation projector convert sensor-specific measurements into partial x/y/heading observations with covariance. The Phase 2 estimator owns innovation testing and correction; adapters never overwrite odometry. The committed LiDAR design may later provide observations through this same interface, but navigation/planning is not part of this phase.

**Tech Stack:** C++17, fixed field geometry, distance/vision/GPS adapters, Phase 2 EKF, host simulation/replay.

## Global Constraints

- Phase 2 pose estimate and quality must be validated before corrections can affect controllers.
- Do not force every observation to update x, y, and heading.
- Reject stale, non-finite, impossible, geometrically ambiguous, and high-innovation corrections.
- Store sensor extrinsics, field geometry version, variance, and thresholds in robot configuration.
- Corrections remain observe-only until repeated physical comparison shows improvement.
- LiDAR/Pi inputs are untrusted external observations with sequence, age, covariance, and protocol validation.

---

## File Structure

- `include/aon/field/geometry.hpp`: static walls, goals, landmarks, and field version.
- `include/aon/estimation/field-observation.hpp`: partial dimension mask, values, variance, source, timestamp.
- `include/aon/estimation/observation-projector.hpp` and `.cpp`: wall/landmark projection.
- `src/aon/estimation/distance-observation-adapter.cpp`: PROS distance sensor adapter.
- `src/aon/estimation/vision-observation-adapter.cpp`: landmark adapter.
- `tests/field-observation-test.cpp`: geometry, projection, and rejection.

### Task 1: Static Field Model and Partial Observation Contract

**Files:**
- Create field/observation headers and `tests/field-observation-test.cpp`.

**Interfaces:**
- Produces: `DimensionMask`, `FieldObservation`, `ObservationType`, `ObservationSource`, `FieldGeometry`, `WallSegment`, and `Landmark`.

- [ ] **Step 1:** Test X-only, Y-only, heading-only, XY, and invalid empty masks; field bounds; unique IDs; finite geometry; and version digest stability.
- [ ] **Step 2:** Implement constexpr/fixed-array geometry with inches and documented frame.
- [ ] **Step 3:** Run tests, commit `feat(field): define observable field geometry`, and push.

### Task 2: Wall Distance Projection

**Files:**
- Create projector header/source.
- Modify field observation tests.

**Interfaces:**
- Produces: `projectWallDistance(PoseEstimate, SensorExtrinsics, DistanceReading, FieldGeometry)` returning accepted observation or typed rejection.

- [ ] **Step 1:** Test sensor translation/rotation, facing each wall, X/Y-only result, oblique ambiguity, out-of-range, no intersection, stale pose/read, poor pose quality, and variance propagation.
- [ ] **Step 2:** Implement ray/segment intersection and axis observability without modifying the estimate.
- [ ] **Step 3:** Add configurable incidence-angle and range gates; ambiguous intersections reject.
- [ ] **Step 4:** Run tests, commit `feat(estimation): project wall observations`, and push.

### Task 3: Landmark and External Observation Adapters

**Files:**
- Create vision adapter and protocol-neutral external observation decoder.
- Modify tests.

**Interfaces:**
- Produces: bearing/position observations from known landmark IDs and a validated `ExternalLocalizationFrame` compatible with the Pi/LiDAR design.

- [ ] **Step 1:** Test known/unknown landmark, bearing wrap, wrong field version, stale/duplicate/out-of-order sequence, invalid covariance, impossible bounds, checksum/protocol mismatch, and reconnect.
- [ ] **Step 2:** Implement decoding to `FieldObservation`; never expose external pose as authoritative assignment.
- [ ] **Step 3:** Run tests, commit vision and external adapters separately, and push.

### Task 4: Estimator Innovation and Telemetry

**Files:**
- Modify: Phase 2 estimator files.
- Modify: health and telemetry event adapters.
- Create: `tests/field-correction-replay-test.cpp`.

**Interfaces:**
- Consumes: `FieldObservation` dimension mask/value/variance.
- Produces: `CorrectionResult` with accepted/rejected dimensions, normalized innovation, threshold, source, and reason.

- [ ] **Step 1:** Test partial covariance updates, cross-covariance effects, per-dimension thresholds, accepted/rejected logs, repeated same timestamp, delayed observation, and numerical guard.
- [ ] **Step 2:** Implement dimension-specific EKF update using Joseph covariance form.
- [ ] **Step 3:** Publish bounded correction events; no direct terminal or SD call.
- [ ] **Step 4:** Replay traces with drift and outliers; verify accepted corrections reduce error and rejected corrections leave state unchanged.
- [ ] **Step 5:** Commit `feat(estimation): apply partial field corrections` and push.

### Task 5: Observe-Only Physical Calibration

**Files:**
- Create: `docs/testing/2026-08-10-field-correction-checklist.md`
- Create robot-specific extrinsic measurement records.
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: measured sensor mounting pose, known wall/landmark positions, repeated routes.
- Produces: reviewed extrinsics, noise, innovation thresholds, and observe-only comparison.

- [ ] **Step 1:** Measure extrinsics and field landmarks; record tools, uncertainty, robot configuration, and exact commit.
- [ ] **Step 2:** Collect stationary and moving observations at varied ranges/angles, including deliberate invalid geometry.
- [ ] **Step 3:** Compare estimator with corrections disabled versus calculated-but-not-applied over repeated routes.
- [ ] **Step 4:** Commit raw observations and proposed thresholds; stop for review.

### Task 6: Progressive Correction Authorization

**Files:**
- Modify robot-specific target configuration.
- Modify physical checklist and handoff.

**Interfaces:**
- Produces: separate enable flags per observation source and robot, default false until its gate passes.

- [ ] **Step 1:** Enable one wall/axis/source at conservative variance in a supervised commit.
- [ ] **Step 2:** Repeat drift, outlier, cancellation, and route tests; verify no physically impossible correction is accepted.
- [ ] **Step 3:** Require repeated improvement in mean and worst-case error without new failures before retaining authorization.
- [ ] **Step 4:** Commit each source authorization and measured result separately; keep all untested sources false.

