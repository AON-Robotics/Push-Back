# Phase 7 Driver Assistance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add optional heading, alignment, precision, braking, and stability assistance without removing manual priority or allowing assistance to fight the driver.

**Architecture:** The dual-robot driver seam publishes one immutable controller snapshot and converts it to `DriverIntent`. Independent pure assist modules transform intent under health and motion constraints, then a target drivetrain adapter emits the final command. Every module has explicit engagement, override, stale-input, and disengagement rules.

**Tech Stack:** C++17 pure policies, PROS controller/drivetrain adapters, Phase 2/5/8/11 snapshots, host scenario tests, Brain/controller status.

## Global Constraints

- Driver input has clear priority; manual override disengages assistance immediately and predictably.
- Assistance never acquires drivetrain ownership during autonomous or another owner.
- A stale/poor pose or unhealthy sensor disengages dependent assistance without a command jump.
- Each feature has a separate robot-specific enable flag defaulting false until its physical gate passes.
- Do not combine several new assists in one physical checkpoint.
- No heap allocation, filesystem access, or high-rate formatting in the 10 ms driver path.

---

## File Structure

- `include/aon/driver/input.hpp`: timestamped axes/buttons and validity.
- `include/aon/driver/intent.hpp`: normalized translation/rotation/manual-priority intent.
- `include/aon/driver/assist.hpp`: module result, status, disengagement reason.
- `include/aon/driver/pipeline.hpp` and `.cpp`: ordered module evaluation and constraints.
- `src/aon/driver/heading-assist.cpp`, `alignment-assist.cpp`, `braking-assist.cpp`.
- `tests/driver-pipeline-test.cpp`, `heading-assist-test.cpp`, `alignment-assist-test.cpp`.

### Task 1: Controller Snapshot, Intent, and Manual Priority

**Files:**
- Create input/intent headers and `tests/driver-pipeline-test.cpp`.
- Modify the target driver adapters created by the dual-robot prerequisite.

**Interfaces:**
- Produces: `ControllerSnapshot`, `DriverIntent`, `DriverCommand`, and pure `makeDriverIntent`.

- [ ] **Step 1:** Test deadband, scaler output, axis saturation, button edge/held state, stale/invalid controller, small/big mappings, and manual rotation/translation priority.
- [ ] **Step 2:** Implement one controller read per 10 ms cycle and share the snapshot with Shadow capture rather than rereading axes.
- [ ] **Step 3:** Preserve exact current drivetrain/mechanism commands with all assists disabled.
- [ ] **Step 4:** Run driver/Shadow/route tests and dual builds; physically compare disabled-pipeline behavior before commit.
- [ ] **Step 5:** Commit `refactor(driver): publish one driver intent` and push.

### Task 2: Assist Pipeline and Arbitration

**Files:**
- Create assist/pipeline headers/source and extend pipeline tests.

**Interfaces:**
- Produces: fixed ordered `AssistId`, `AssistResult`, `AssistStatusSnapshot`, and `AssistPipeline::evaluate(intent, context)`.
- Consumes: pose/quality, drivetrain observation, health, constraints, mechanism configuration, and configured enable flags.

- [ ] **Step 1:** Test no assists, one/multiple transforms, manual override, autonomous owner, stale context, feature disabled, constraint application, deterministic order, and output saturation.
- [ ] **Step 2:** Implement fixed module table/function pointers; no runtime registration or dynamic allocation.
- [ ] **Step 3:** Make every disengagement return the unassisted current intent, not a stale assisted command.
- [ ] **Step 4:** Run tests, commit `feat(driver): arbitrate optional assistance`, and push.

### Task 3: Heading Hold and Snap-to-Heading

**Files:**
- Create heading assist source/test and target config values.

**Interfaces:**
- Produces: hold state, target heading, correction, engagement/disengagement reason, and snap targets per robot/field mode.

- [ ] **Step 1:** Test engage after rotation release, intentional turn override, heading wrap, snap shortest direction, poor/invalid pose, timeout, saturation, integral reset, and zero-translation policy.
- [ ] **Step 2:** Implement bounded P/PD correction using measured timestamps and existing output constraints; do not reuse autonomous PID state.
- [ ] **Step 3:** Keep flags false, run host/build gates, commit code and locked configuration separately.

### Task 4: Straight Correction, Precision, and Low-Speed Braking

**Files:**
- Create focused sources/tests and target configuration.

**Interfaces:**
- Produces: independent assist transforms; precision is explicit driver selection, straight correction requires forward intent, braking requires released input and healthy velocity estimate.

- [ ] **Step 1:** Test diagonal/manual turn intent, mode transitions, brake engage/release, velocity sign, stale estimate, and no command discontinuity.
- [ ] **Step 2:** Implement each feature independently and compose through the pipeline.
- [ ] **Step 3:** Run tests/builds; physically authorize one feature at a time and commit each flag separately.

### Task 5: Alignment and Anti-Tip/Stability Limits

**Files:**
- Create alignment assist source/test.
- Modify constraint provider configuration rather than duplicating limits in driver code.

**Interfaces:**
- Consumes: field observation/target, pose quality, mechanism state, and Phase 11 constraints.
- Produces: bounded alignment correction or predictable disengagement.

- [ ] **Step 1:** Test goal/loader target acquisition, target loss, ambiguous observation, manual override, timeout, field bounds, raised mechanism, and invalid pose.
- [ ] **Step 2:** Implement alignment as opt-in hold action; anti-tip behavior enters through motion constraints, not a second limiter.
- [ ] **Step 3:** Keep unavailable sensors/targets absent rather than synthetic no-ops.
- [ ] **Step 4:** Run tests/builds and physical alignment/stability gate before enabling.

### Task 6: Driver Status and Physical Acceptance

**Files:**
- Modify Brain/controller status adapter.
- Create: `docs/testing/2026-08-10-driver-assistance-checklist.md`
- Modify handoff.

- [ ] **Step 1:** Show only currently engaged assist and important disengagement/fault at low rate; no rumble/display flood.
- [ ] **Step 2:** For each feature record engagement, override latency, unexpected fight, endpoint/heading effect, current, stability, and driver judgment on both robots where supported.
- [ ] **Step 3:** Test disable/autonomous transition and Controller-X ownership boundaries.
- [ ] **Step 4:** Enable only individually accepted features; commit measured results and flags separately and push.

