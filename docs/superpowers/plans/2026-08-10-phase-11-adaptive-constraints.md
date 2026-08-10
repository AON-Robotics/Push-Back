# Phase 11 Adaptive Motion Constraints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce conservative linear/angular velocity and acceleration limits from path, battery, mechanism, localization, slip, current, and robot-stability state.

**Architecture:** Independent pure constraint providers return optional limits with freshness and reason; the manager intersects valid limits and falls back conservatively on invalid critical input. Path geometry remains separate, and LemLib integration uses narrow non-vendored adapters.

**Tech Stack:** C++17 fixed value types, Phase 2/5/6/8 snapshots, LemLib adapter, host scenario tests.

## Global Constraints

- Never increase a configured absolute robot limit.
- Missing/stale safety-critical input yields conservative limits, not aggressive defaults.
- Do not mix path generation with constraint policy.
- Do not fork vendored LemLib.
- Limit changes use bounded slew/hysteresis where abrupt reduction would destabilize control, while critical stop remains immediate.
- Keep per-robot limits and mechanism stability envelopes physically measured and authorization-gated.

---

## File Structure

- `include/aon/control/motion-constraints.hpp`: limits, reasons, provider result.
- `include/aon/control/constraint-manager.hpp` and `.cpp`: pure intersection and transition policy.
- `src/aon/control/constraint-providers.cpp`: curvature, battery, mechanism, pose, observer, current providers.
- `src/aon/lemlib/constraint-adapter.cpp`: supported LemLib seam.
- `tests/constraint-manager-test.cpp`: scenarios and numerical edges.

### Task 1: Constraint Contracts and Intersection

**Files:**
- Create value header, manager source, and tests.

**Interfaces:**
- Produces: `MotionConstraints`, `ConstraintReason`, `ConstraintCandidate`, `ConstraintInputs`, and `ConstraintManager::evaluate`.

- [ ] **Step 1:** Test no candidates, single/multiple minima, invalid/negative/non-finite limits, absolute cap, stale critical/noncritical input, reason retention, and deterministic ties.
- [ ] **Step 2:** Implement component-wise conservative intersection with configured fallback constraints.
- [ ] **Step 3:** Run tests, commit `feat(control): compose adaptive motion limits`, and push.

### Task 2: Curvature and Battery Providers

**Files:**
- Create provider source and extend tests.

**Interfaces:**
- Produces: curvature-derived linear velocity respecting wheel speed and battery/sag-derived command/acceleration limits.

- [ ] **Step 1:** Test straight, tight curve, zero/infinite/invalid curvature, weak battery, transient sag, recovery hysteresis, and robot-specific caps.
- [ ] **Step 2:** Implement pure formulas with documented units; battery provider consumes health snapshot, not direct hardware.
- [ ] **Step 3:** Run tests, commit `feat(control): constrain curvature and battery load`, and push.

### Task 3: Mechanism, Pose, Slip, Current, and Stability Providers

**Files:**
- Modify provider source/tests and robot target configuration.

**Interfaces:**
- Consumes: mechanism position/configuration, `PoseQuality`, drivetrain observation, current health, and stability envelope.

- [ ] **Step 1:** Test lowered/raised mechanism, good/degraded/poor/invalid pose, suspected/confirmed slip, current warning/fault, stale snapshots, and combined worst case.
- [ ] **Step 2:** Implement independent providers with reason codes; do not embed executive response.
- [ ] **Step 3:** Keep unmeasured mechanism/stability profiles at conservative baseline values and disabled aggressive recovery.
- [ ] **Step 4:** Run tests/builds, commit providers separately, and push.

### Task 4: Constraint Transition Policy

**Files:**
- Modify manager/tests.

**Interfaces:**
- Produces: `update(previous, requested, dt)` with immediate tightening for critical states and configured bounded relaxation.

- [ ] **Step 1:** Test abrupt tighten, gradual relax, jittering source, timestamp gap, critical stop, cancellation, and reset.
- [ ] **Step 2:** Implement allocation-free slew/hysteresis and expose active reasons.
- [ ] **Step 3:** Run tests, commit `feat(control): stabilize adaptive constraint transitions`, and push.

### Task 5: Controller and LemLib Adapters

**Files:**
- Create LemLib adapter and modify non-vendored action/controller seams.
- Create: `tests/constraint-adapter-test.cpp`.

**Interfaces:**
- Consumes: immutable constraints at action start and supported update points.
- Produces: capped action parameters/command references without altering path geometry.

- [ ] **Step 1:** Test conversion, unit scaling, clamping, unsupported dynamic update, and fallback behavior.
- [ ] **Step 2:** Apply static per-action limits first; dynamic updates only where LemLib/action seam is deterministic and host/physical tested.
- [ ] **Step 3:** Record requested versus applied constraints in telemetry.
- [ ] **Step 4:** Run motion/path/Shadow suites and dual builds, commit adapter locked behind authorization, and push.

### Task 6: Physical Constraint Calibration and Authorization

**Files:**
- Create: `docs/testing/2026-08-10-adaptive-constraints-checklist.md`
- Modify target config and handoff.

- [ ] **Step 1:** Measure normal straight/curve limits, low battery, raised mechanism, poor pose simulation, and induced slip at safe speeds.
- [ ] **Step 2:** Verify stability, endpoint error, time, current, sag, slip events, and cancellation against static baseline over repeated runs.
- [ ] **Step 3:** Enable one provider at a time only after it improves safety without unacceptable repeatability loss.
- [ ] **Step 4:** Commit each authorization and measurement separately; retain static constraints as fallback.

