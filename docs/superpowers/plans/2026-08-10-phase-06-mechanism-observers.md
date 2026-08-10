# Phase 6 Mechanism Observers and Jam Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detect mechanism jams, stalls, failed transitions, timeouts, and sensor disagreements, then run strictly bounded recovery sequences that always end safely.

**Architecture:** A reusable pure observer compares command intent, motor/sensor snapshots, and transition deadlines. Each concrete mechanism supplies expectations and a nonblocking command adapter; recovery policy is separate from detection and owns a fixed attempt budget.

**Tech Stack:** C++17 state machines, PROS motor/sensor adapters, existing Intake/Orbit/Piston mechanisms, host fake traces, health/executive integration.

## Global Constraints

- Preserve the big sorter mutex, request acknowledgement, and fail-safe ownership.
- Do not create two writers for intake, elevator, judge, scorer, Orbit, or pistons.
- No infinite recovery loop; every retry, pulse, transition, and wait has an absolute deadline.
- Manual driver override cancels assistance/recovery predictably and receives ownership.
- Recovery exhaustion commands the mechanism-specific safe state and reports `MechanismFailure`.
- Migrate one mechanism and one robot at a time with physical validation.

---

## File Structure

- `include/aon/observers/mechanism-types.hpp`: generic intent/sample/observation.
- `include/aon/observers/mechanism-observer.hpp` and `.cpp`: pure detector.
- `include/aon/mechanisms/recovery.hpp` and `.cpp`: bounded recovery state machine.
- target mechanism adapters under `apps/small-robot/` and `apps/big-robot/`.
- `tests/mechanism-observer-test.cpp`, `mechanism-recovery-test.cpp`.

### Task 1: Reusable Mechanism Observation Core

**Files:**
- Create generic headers/source and observer test.

**Interfaces:**
- Produces: `MechanismId`, `MechanismIntent`, `MechanismSample`, `MechanismCondition`, `MechanismObservation`, and `MechanismObserverConfig`.

- [ ] **Step 1:** Test idle, transitioning, reached target, slow motion, no motion, excessive current, sensor disagreement, unexpected object, stale/invalid sample, timeout, transient recovery, and hysteresis.
- [ ] **Step 2:** Implement fixed state and evidence counters with measured `dt`; no motor commands.
- [ ] **Step 3:** Run tests, commit `feat(observers): add mechanism health core`, and push.

### Task 2: Bounded Recovery Scheduler

**Files:**
- Create recovery header/source and recovery test.

**Interfaces:**
- Produces: `RecoveryStep {command, durationMs, verification}`, `RecoveryPlan`, `RecoveryState`, and `RecoveryController::update(now, observation)`.

- [ ] **Step 1:** Test stop/reverse/stop/retry/verify success, cancellation at every step, timeout, stale observation, command rejection, maximum attempts, timestamp wrap, and safe-stop callback count.
- [ ] **Step 2:** Implement a fixed maximum step/attempt plan with absolute deadlines and no sleeping inside the pure controller.
- [ ] **Step 3:** Ensure every terminal state requests the safe command exactly once.
- [ ] **Step 4:** Run tests, commit `feat(mechanisms): schedule bounded recovery`, and push.

### Task 3: Adapt the Big-Robot Sorter First

**Files:**
- Modify: `include/aon/intake/intake.hpp`
- Modify: `src/aon/intake.cpp`
- Create: big target sorter adapter.
- Extend existing Shadow/mechanism tests.

**Interfaces:**
- Consumes: current `SortState`, release request/ack counters, proximity, optical, motor telemetry, and mutex-owned commands.
- Produces: observer snapshot and explicit safe/recovery commands through the existing single motor command function.

- [ ] **Step 1:** Characterize current transition timing and commands in fake-policy tests before changing behavior.
- [ ] **Step 2:** Publish intent/sample/observation without changing sorter transitions.
- [ ] **Step 3:** Add recovery only for a physically validated jam signature; preserve acknowledgement and `sortFaulted` behavior.
- [ ] **Step 4:** Run mechanism/Shadow/health tests and both builds; physically test sorter before enabling recovery.
- [ ] **Step 5:** Commit observation and recovery as separate checkpoints and push.

### Task 4: Convert Small Intake Blocking Pulses to a State Machine

**Files:**
- Modify small Intake implementation and target adapter.
- Create: `tests/small-intake-state-test.cpp`.

**Interfaces:**
- Produces: nonblocking scan/sort state with exact existing 500/100/125 ms semantics represented as deadlines initially.

- [ ] **Step 1:** Write characterization tests for current detection, motor commands, pulse durations, scan disable, alliance color decisions, and stop behavior.
- [ ] **Step 2:** Implement command-on-transition state machine with shared motor ownership; no delay inside its update.
- [ ] **Step 3:** Add immediate cancellation/manual takeover and stale sensor safe stop.
- [ ] **Step 4:** Run tests, dual builds, and side-by-side physical block handling before enabling jam recovery.
- [ ] **Step 5:** Commit `refactor(intake): make sorting transitions nonblocking` and push.

### Task 5: Extend to Scorer, Lever, Orbit, Cart, and Pneumatics

**Files:**
- Modify target mechanism adapters and add focused tests per mechanism.

**Interfaces:**
- Consumes: requested state, motor/sensor snapshot, and transition timing for each real mechanism.
- Produces: capability-specific health; absent mechanisms have no fake/no-op observer.

- [ ] **Step 1:** Migrate motorized scorer/lever and Orbit first because they have position/velocity evidence and existing unbounded waits.
- [ ] **Step 2:** Add timeout/cancellation before enabling any currently unbounded Orbit rotate path.
- [ ] **Step 3:** Observe pneumatic transitions only where a real confirming sensor exists; otherwise report commanded state, not fabricated success.
- [ ] **Step 4:** Run focused host/build/physical gates and commit each mechanism separately.

### Task 6: Health and Autonomous Failure Integration

**Files:**
- Modify health adapters and Phase 4 executive adapters.
- Create: `tests/mechanism-failure-propagation-test.cpp`.

**Interfaces:**
- Produces: stable mechanism fault codes and `ActionOutcome::MechanismFailure` after exhausted recovery.

- [ ] **Step 1:** Test optional/critical action response, retry exhaustion, cancellation, manual override, and stopped outputs.
- [ ] **Step 2:** Publish observer state to GUI/telemetry at low rate.
- [ ] **Step 3:** Integrate one migrated mechanism route at a time; native routes retain existing behavior until validated.
- [ ] **Step 4:** Commit and push every mechanism integration checkpoint.

### Task 7: Physical Jam and Recovery Matrix

**Files:**
- Create: `docs/testing/2026-08-10-mechanism-recovery-checklist.md`
- Modify handoff.

- [ ] **Step 1:** Define safe, repeatable jam fixtures and current/velocity baselines for every tested mechanism.
- [ ] **Step 2:** Verify normal cycles do not trigger recovery, then verify one bounded recovery and exhausted safe failure.
- [ ] **Step 3:** Test Controller-X/disable/manual takeover during every recovery stage.
- [ ] **Step 4:** Record attempts, elapsed time, peak current, final output/state, and fault code; commit measurements separately.

