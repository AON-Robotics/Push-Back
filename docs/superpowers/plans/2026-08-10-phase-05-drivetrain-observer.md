# Phase 5 Drivetrain Observer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect persistent drivetrain slip, blockage, external push, stall, collision, and sensor disagreement from synchronized command, motor, pose, and IMU evidence.

**Architecture:** A pure observer consumes immutable `DrivetrainSample` values and publishes a typed observation with state, confidence, evidence, freshness, and timestamps. It never commands motors; controller, constraint, health, and autonomous policies decide how to respond.

**Tech Stack:** C++17 fixed-size state machines, central sensor snapshots, Phase 2 pose quality, PROS motor telemetry adapters, host trace tests.

## Global Constraints

- Do not trigger confirmed faults from one sample.
- Do not classify a stationary blocked robot as a tracking-sensor failure.
- Do not let the observer modify PID state, motor commands, or autonomous flow.
- Use measured `dt` and timestamp freshness, not an assumed exact 20 ms loop.
- Keep thresholds robot-specific and inactive for controller response until physical fault-injection gates pass.
- Bound histories with persistence counters/windows; no dynamic trace storage in the observer.

---

## File Structure

- `include/aon/observers/drivetrain-types.hpp`: sample, state, evidence, observation.
- `include/aon/observers/drivetrain-observer.hpp` and `.cpp`: pure state machines.
- `src/aon/observers/drivetrain-sampler.cpp`: synchronized adapter.
- `tests/drivetrain-observer-test.cpp`: synthetic scenarios and numerical edges.
- `tests/fixtures/drivetrain/`: versioned recorded traces added only after physical capture.

### Task 1: Sample and Observation Contracts

**Files:**
- Create type header and `tests/drivetrain-observer-test.cpp`.

**Interfaces:**
- Produces: `DrivetrainCondition`, `DrivetrainEvidence`, `DrivetrainSample`, `DrivetrainObservation`, and `DrivetrainObserverConfig`.

- [ ] **Step 1:** Test default invalid state, finite fields, independent validity/freshness, quality propagation, state names, fixed evidence capacity, and trivial copyability.
- [ ] **Step 2:** Implement value-only types with explicit units in field names/comments and no PROS dependencies.
- [ ] **Step 3:** Run warnings-as-errors tests, commit `feat(observers): define drivetrain observations`, and push.

### Task 2: Slip and Sensor-Disagreement State Machines

**Files:**
- Create observer header/source.
- Modify observer tests.

**Interfaces:**
- Produces: `DrivetrainObserver::reset`, `observe(sample)`, and `snapshot`.

- [ ] **Step 1:** Test normal acceleration, wheel speed without pose speed, pose speed without matching wheel speed, left/right disagreement, poor pose quality, stale pose, transient spike, persistence entry, hysteretic exit, and timestamp gaps.
- [ ] **Step 2:** Implement `Normal -> SuspectedSlip -> Slipping` and sensor-disagreement transitions using configured evidence duration and clear duration.
- [ ] **Step 3:** Suppress pose-based classification when pose quality/age is insufficient and report degraded observability rather than false slip.
- [ ] **Step 4:** Run tests, commit `feat(observers): detect persistent drivetrain slip`, and push.

### Task 3: Blocked, Stall, Collision, and External-Push Detection

**Files:**
- Modify observer source/tests.

**Interfaces:**
- Consumes: effective command, requested/measured wheel velocity, current, position delta, pose velocity, and IMU acceleration.
- Produces: mutually prioritized condition plus all supporting evidence bits.

- [ ] **Step 1:** Test high command/high current/low motion stall, commanded motion with low current blockage ambiguity, collision acceleration impulse with command, external push without command, normal braking, mechanism contact, single motor stall, invalid current, and recovery.
- [ ] **Step 2:** Implement separate persistence/hysteresis for suspected/confirmed stall and blocked states; collision impulse requires corroborating velocity change.
- [ ] **Step 3:** Define deterministic priority when several conditions overlap and retain secondary evidence.
- [ ] **Step 4:** Run tests, commit `feat(observers): classify drivetrain faults`, and push.

### Task 4: Central Drivetrain Sampler

**Files:**
- Create: `src/aon/observers/drivetrain-sampler.cpp`
- Modify central sensor snapshot and effective-command publication.
- Modify runtime/health adapters.

**Interfaces:**
- Consumes: one motor/IMU/pose snapshot and one atomic effective command/reference snapshot.
- Produces: one timestamp-coherent observer sample at the configured rate.

- [ ] **Step 1:** Add fake-adapter tests for unit conversion, sample age, sequence mismatch, and partial device failure.
- [ ] **Step 2:** Read motor telemetry once per central sample; do not add independent per-consumer polling.
- [ ] **Step 3:** Instrument sampling and observer execution; publish immutable result to health/telemetry.
- [ ] **Step 4:** Run host suites, dual builds, stack/memory review, commit `feat(observers): sample drivetrain health`, and push.

### Task 5: Observe-Only Physical Fault Matrix

**Files:**
- Create: `docs/testing/2026-08-10-drivetrain-observer-checklist.md`
- Add recorded trace fixtures after capture.
- Modify handoff.

**Interfaces:**
- Produces: measured thresholds and false-positive/negative evidence; no controller response authorization.

- [ ] **Step 1:** Record normal straight, turn, acceleration, braking, path curvature, and mechanism-contact runs.
- [ ] **Step 2:** Safely induce wheels-off-ground slip, soft blockage, controlled push, individual motor disconnect where approved, and low-speed collision.
- [ ] **Step 3:** Label traces with ground truth, robot, battery, surface, load, exact commit, and observer output.
- [ ] **Step 4:** Tune using multiple traces, run replay regressions, and commit datasets/config proposals separately.

### Task 6: Progressive Consumer Policies

**Files:**
- Modify health adapter, later constraint manager, and later executive policy files.
- Create: `tests/drivetrain-response-policy-test.cpp`.

**Interfaces:**
- Produces: pure response recommendations `Observe`, `LimitAcceleration`, `InhibitIntegral`, `Abort`, or `RequestRecovery`.

- [ ] **Step 1:** Test every condition/quality/risk combination and stale observation.
- [ ] **Step 2:** Enable health reporting first, conservative constraints second, autonomous abort/recovery last.
- [ ] **Step 3:** Validate each response physically with cancellation and stopped outputs.
- [ ] **Step 4:** Commit each consumer authorization separately; observer remains command-free.

