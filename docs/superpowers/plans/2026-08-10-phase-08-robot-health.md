# Phase 8 Robot Health Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish one immutable, specific, freshness-aware health snapshot for autonomous, driver, GUI, telemetry, and diagnostics.

**Architecture:** Producers report bounded subsystem snapshots; the health manager performs pure aggregation and never polls or commands hardware. Stable fault codes and severity rules are shared across phases, while producer-specific details remain in their own modules.

**Tech Stack:** C++17, fixed arrays, PROS sensor adapters, host GCC, Brain GUI, existing Shadow/motion status.

## Global Constraints

- Do not reduce health to one Boolean.
- Health observation never commands motors or performs SD I/O.
- Fault arrays, names, and histories are bounded and allocation-free.
- Use timestamps and expected age; do not infer freshness from task existence.
- Critical faults fail risky autonomous preconditions, but warning display must not flood match loops.
- Preserve existing motion, fallback, Shadow, and sorter result types through adapters.

---

## File Structure

- `include/aon/health/fault.hpp`: stable IDs, severity, and records.
- `include/aon/health/snapshot.hpp`: subsystem and robot health snapshots.
- `include/aon/health/manager.hpp` and `src/aon/health/manager.cpp`: pure aggregation.
- `src/aon/health/pros-producers.cpp`: low-rate battery/device/storage adapters.
- `tests/health-manager-test.cpp`: aggregation, freshness, latching, and capacity.
- `src/aon/tools/gui/`: low-rate fault display.

### Task 1: Stable Fault and Health Value Types

**Files:**
- Create: `include/aon/health/fault.hpp`
- Create: `include/aon/health/snapshot.hpp`
- Create: `tests/health-manager-test.cpp`

**Interfaces:**
- Produces: `SubsystemId`, `FaultCode`, `FaultSeverity`, `HealthStatus`, `FaultRecord`, `SubsystemHealthSnapshot`, and `RobotHealthSnapshot`.

- [ ] **Step 1:** Test stable numeric codes, ordering from Ready through Critical, fixed-capacity insertion, duplicate-code coalescing, saturation, and trivial copyability.
- [ ] **Step 2:** Confirm compilation fails, then implement value-only headers without PROS or strings.
- [ ] **Step 3:** Add `faultName(FaultCode)` returning process-lifetime string literals for formatting outside producers.
- [ ] **Step 4:** Run warnings-as-errors test, commit `feat(health): define structured health snapshots`, and push.

### Task 2: Pure Aggregation, Freshness, and Latching

**Files:**
- Create: `include/aon/health/manager.hpp`
- Create: `src/aon/health/manager.cpp`
- Modify: `tests/health-manager-test.cpp`

**Interfaces:**
- Produces: `HealthManager::update(const HealthInputs&, uint32_t now)` and `RobotHealthSnapshot snapshot() const`.
- Consumes: battery, localization, sensors, storage, timing, drivetrain, and mechanism producer snapshots.

- [ ] **Step 1:** Test healthy input, stale producer, warning aggregation, critical dominance, fault activation/clear, latched fault reset policy, duplicate faults, timestamp wrap, missing required producer, and maximum fault capacity.
- [ ] **Step 2:** Implement deterministic aggregation with configuration-owned maximum ages and latching policies.
- [ ] **Step 3:** Preserve first-observed time and count occurrences; update last-observed time only when active evidence arrives.
- [ ] **Step 4:** Run tests, commit `feat(health): aggregate subsystem health`, and push.

### Task 3: Adapt Existing Status Producers

**Files:**
- Modify: `src/aon/auton/fallback-status.cpp`
- Modify: `src/aon/auton/status.cpp`
- Modify: `src/aon/shadow/service.cpp`
- Modify: `src/aon/intake.cpp`
- Create: `src/aon/health/existing-status-adapters.cpp`
- Test: existing motion and Shadow suites plus `health-manager-test.cpp`.

**Interfaces:**
- Consumes: `FallbackStatusSnapshot`, `RoutineStatus`, Shadow `Status`, sorter fault/transition state.
- Produces: subsystem health values without changing original public results.

- [ ] **Step 1:** Add adapter tests for every existing failure/result enum value and freshness.
- [ ] **Step 2:** Implement exhaustive switches; compiler warnings fail on unhandled enum additions.
- [ ] **Step 3:** Ensure adapters read one immutable producer snapshot and perform no hardware call.
- [ ] **Step 4:** Run focused suites and dual builds, commit `feat(health): adapt existing subsystem status`, and push.

### Task 4: Low-Rate PROS Health Producers

**Files:**
- Create: `include/aon/health/pros-producers.hpp`
- Create: `src/aon/health/pros-producers.cpp`
- Modify: robot lifecycle/composition files.
- Create: `tests/health-threshold-test.cpp`

**Interfaces:**
- Produces: battery voltage/sag, motor temperature/current, sensor connectivity/freshness, SD presence/error, and task timing health at configured low rates.

- [ ] **Step 1:** Extract pure threshold policies and test boundary, hysteresis, stale, invalid, and recovery cases.
- [ ] **Step 2:** Add PROS sampling outside high-frequency controllers; group devices by update period and reuse central sensor snapshots.
- [ ] **Step 3:** Do not allocate vectors each health cycle when a fixed or cached adapter is available; measure unavoidable API cost.
- [ ] **Step 4:** Instrument producer execution, run tests and dual builds, commit per producer group, and push.

### Task 5: Autonomous Preconditions and Diagnostics

**Files:**
- Create: `include/aon/health/preconditions.hpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: GUI display files.
- Create: `tests/health-precondition-test.cpp`

**Interfaces:**
- Produces: pure `checkPrecondition(OperationRisk, RobotHealthSnapshot)` returning allowed/degraded/rejected plus specific `FaultCode`.

- [ ] **Step 1:** Test low/medium/high-risk operations against localization, drivetrain, mechanism, battery, storage, and timing states.
- [ ] **Step 2:** Insert precondition checks before ownership acquisition for newly migrated actions; do not retroactively block proven native routes without a physical gate.
- [ ] **Step 3:** Display concise highest-severity faults at low rate and retain a scrollable debug list.
- [ ] **Step 4:** Run tests, dual builds, and fault-injection bench checks; commit action and GUI changes separately.

### Task 6: Physical Health Fault Matrix

**Files:**
- Create: `docs/testing/2026-08-10-robot-health-checklist.md`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: both robot configurations and safe fault-injection procedures.
- Produces: measured fault detection/recovery latency and false-positive observations.

- [ ] **Step 1:** Test low battery/sag under load, disconnected or stale sensors, invalid IMU, SD absence/write failure, Shadow fault, timing miss, drivetrain fault, and mechanism fault where safe.
- [ ] **Step 2:** Verify specific Brain text, telemetry code, autonomous precondition, and safe output behavior.
- [ ] **Step 3:** Record thresholds requiring tuning; never tune from one sample.
- [ ] **Step 4:** Commit measurements separately and keep unvalidated high-risk preconditions disabled.

