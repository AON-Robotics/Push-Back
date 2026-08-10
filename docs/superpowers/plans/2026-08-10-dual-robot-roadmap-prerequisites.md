# Dual-Robot Roadmap Prerequisites Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish two named build targets, explicit robot composition, and narrow dependency seams needed by every roadmap subsystem without prematurely changing physical device ownership.

**Architecture:** Execute the behavior-preserving portion of the existing dual-robot production architecture plan through explicit checkpoints, then stop before its physical ownership gate. The roadmap consumes target configuration and injected services; it does not add new branches to `USING_BIG_ROBOT` or deepen global aliases.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, GNU Make, PowerShell, host GCC, Git.

## Global Constraints

- This plan refines sequencing for `docs/superpowers/plans/2026-08-03-dual-robot-production-architecture.md`; that plan's exact interface and test requirements remain authoritative.
- Do not start until the safety baseline plan has recorded passing physical results.
- Preserve all hardware values, route behavior, authorizations, initialization order, and task periods.
- Do not centralize or reconstruct physical devices until the original plan's Task 9 physical gate passes.
- Do not delete `USING_BIG_ROBOT`, `hardware()`, native compatibility code, or fallbacks during these prerequisites.
- One architectural seam per checkpoint; clean-build both named targets at every robot-code checkpoint.

---

## File Structure

- `apps/small-robot/`: small target composition and autonomous catalog.
- `apps/big-robot/`: big target composition and autonomous catalog.
- `include/aon/config/`: robot-neutral target configuration values.
- `include/aon/core/` and `src/aon/core/`: shared lifecycle with injected services.
- `include/aon/competition/`: driver input and target-specific adapters.
- `include/aon/auton/`: shared executor boundaries.
- `tests/architecture/`: target and dependency-structure tests.

### Task 1: Revalidate the Existing Hardware-Map Prerequisite

**Files:**
- Inspect: `include/aon/config/hardware-map.hpp`
- Inspect: `src/aon/config/hardware-map.cpp`
- Test: `tests/hardware-map-test.cpp`

**Interfaces:**
- Consumes: `smallRobotHardwareMap`, `bigRobotHardwareMap`, and `validateHardwareMap`.
- Produces: recorded confirmation that no port/reversal value changed and the big right-tracker mismatch remains visible.

- [ ] **Step 1:** Compile and run `hardware-map-test.cpp` with `-Wall -Wextra -Werror`.
- [ ] **Step 2:** Run `rg -n "smallRobotHardwareMap|bigRobotHardwareMap|RightTrackingReversalMismatch" include src tests` and compare values with the current handoff.
- [ ] **Step 3:** Clean-build small, big, then restored small without source changes.
- [ ] **Step 4:** Update `CURRENT_HANDOFF.md` with the verified prerequisite commit and existing mismatch.
- [ ] **Step 5:** Commit only the verification documentation with `git commit -m "docs: verify roadmap hardware-map prerequisite"` and push.

### Task 2: Add Named Build Targets Without Behavior Changes

**Files:**
- Modify: `Makefile`
- Modify: `include/aon/constants.hpp`
- Create: `tests/architecture/robot-targets-test.ps1`

**Interfaces:**
- Consumes: current monolithic PROS build and the two `USING_BIG_ROBOT` branches.
- Produces: `make small-robot`, `make big-robot`, and a temporary compatibility mapping to exactly one selected target macro.

- [ ] **Step 1:** Write a PowerShell test that fails unless both named targets exist, reject both target macros together, and select the expected compatibility value.
- [ ] **Step 2:** Run the test and confirm it fails on the current Makefile.
- [ ] **Step 3:** Add `small-robot` and `big-robot` targets with distinct build directories/outputs and exactly one target definition.
- [ ] **Step 4:** Make `constants.hpp` map the selected build definition to the existing macro temporarily and emit a preprocessor error for zero or two target definitions in production-target builds.
- [ ] **Step 5:** Run the architecture test and clean-build both named outputs.
- [ ] **Step 6:** Restore the expected small target, review generated artifacts, update the handoff, commit `build: add named robot targets`, and push.

### Task 3: Split Immutable Target Configuration

**Files:**
- Create: `include/aon/config/robot-identity.hpp`
- Create: `include/aon/config/target-config.hpp`
- Create: `apps/small-robot/target-config.cpp`
- Create: `apps/big-robot/target-config.cpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Test: `tests/architecture/target-config-test.cpp`

**Interfaces:**
- Produces: `enum class RobotIdentity { Small, Big };`, immutable `TargetConfig`, and one `const TargetConfig& targetConfig()` definition per executable.
- Consumes: existing hardware maps, LemLib/fallback values, authorization values, and mechanism capabilities.

- [ ] **Step 1:** Write host tests that construct both target values without preprocessing and assert identity, maps, dimensions, gains, and authorization values.
- [ ] **Step 2:** Run and confirm failure because `TargetConfig` does not exist.
- [ ] **Step 3:** Define value-only target configuration with no PROS includes or device objects.
- [ ] **Step 4:** Move initializer values without changing numbers or comments; leave `activeRobotConfig()` as a compatibility adapter returning the linked target.
- [ ] **Step 5:** Run target, hardware-map, authorization, and route-policy tests; clean-build both targets.
- [ ] **Step 6:** Commit `refactor(config): split immutable robot targets` and push.

### Task 4: Introduce Shared Lifecycle and Concrete Composition Roots

**Files:**
- Create: `include/aon/core/services.hpp`
- Modify: `include/aon/core/robot.hpp`
- Modify: `src/aon/core/robot.cpp`
- Create: `apps/small-robot/main.cpp`
- Create: `apps/big-robot/main.cpp`
- Test: `tests/architecture/composition-root-test.ps1`

**Interfaces:**
- Produces: a small `RobotServices` aggregate of borrowed references needed by lifecycle coordination and one concrete root per robot.
- Consumes: existing process-lifetime hardware, GUI, autonomous reader, actions, and Shadow service without changing construction order.

- [ ] **Step 1:** Write the structural test proving only app roots define PROS callbacks and shared lifecycle code contains no target-selection branch.
- [ ] **Step 2:** Run and confirm it fails.
- [ ] **Step 3:** Add a borrowed-reference service aggregate; do not create a service locator or polymorphic universal robot.
- [ ] **Step 4:** Move callback forwarding to both app roots while preserving initialization order and task creation.
- [ ] **Step 5:** Run function-registry, route-symbol, Shadow, and composition tests; clean-build both targets.
- [ ] **Step 6:** Commit `refactor(core): add concrete robot compositions` and push.

### Task 5: Inject Autonomous Catalogs and Driver Adapters

**Files:**
- Create: `include/aon/auton/catalog.hpp`
- Create: `apps/small-robot/autonomous-catalog.cpp`
- Create: `apps/big-robot/autonomous-catalog.cpp`
- Create: `include/aon/competition/driver-input.hpp`
- Create: `include/aon/competition/driver-intent.hpp`
- Create: target-specific driver adapters under each app.
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/competition/operator-control.hpp`
- Test: `tests/architecture/autonomous-catalog-test.cpp`
- Test: `tests/architecture/driver-adapter-test.cpp`

**Interfaces:**
- Produces: fixed catalogs containing only supported routines and `DriverInput -> DriverIntent -> target adapter` flow.
- Consumes: existing function registry, route wrappers, scaler, drivetrain, mechanism commands, and Shadow capture hooks.

- [ ] **Step 1:** Write catalog tests for exact red/blue/skills slot names and locked experimental behavior on each robot.
- [ ] **Step 2:** Write driver-intent tests for axis scaling, button edges, and robot-specific capability absence.
- [ ] **Step 3:** Move selection tables and conditional branches without behavioral edits.
- [ ] **Step 4:** Preserve semantic Shadow events at the target mechanism adapter boundary.
- [ ] **Step 5:** Run all existing route/header/symbol/function-registry tests plus new tests and dual clean builds.
- [ ] **Step 6:** Commit autonomous catalog and driver adapter as two separate checkpoints, pushing each.

### Task 6: Inject Actions, Shadow, GUI, and Safety Dependencies

**Files:**
- Modify: `include/aon/auton/actions.hpp`
- Modify: `include/aon/shadow/service.hpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Create: `include/aon/core/safety-coordinator.hpp`
- Modify: `include/aon/globals.hpp`
- Test: `tests/architecture/service-dependencies-test.ps1`
- Test: existing motion and Shadow host suites.

**Interfaces:**
- Produces: explicit borrowed dependencies and one safety coordinator that cancels actions, Shadow, drivetrain, and mechanisms.
- Consumes: existing services and adapters; physical hardware ownership remains unchanged.

- [ ] **Step 1:** Write the dependency-structure test rejecting new shared-module includes of app headers and new global hardware aliases.
- [ ] **Step 2:** Add constructors/factories at the composition roots while retaining compatibility accessors for unmigrated callers.
- [ ] **Step 3:** Route Controller-X and disabled callbacks through the coordinator, preserving cancellation latching and stopped outputs.
- [ ] **Step 4:** Run motion-fallback, Shadow, function registry, source-structure, and dual-build gates.
- [ ] **Step 5:** Commit one service at a time with handoff updates and pushes.

### Task 7: Stop at the Device-Ownership Physical Gate

**Files:**
- Modify: `docs/testing/2026-08-10-roadmap-baseline-checklist.md`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: behavior-preserving named-target and injection checkpoints.
- Produces: physical authorization for Task 10 of the existing dual-robot architecture plan, or a failure report.

- [ ] **Step 1:** Repeat the complete roadmap baseline checklist using both named build targets.
- [ ] **Step 2:** Record GUI selection, native route, LemLib route, Shadow recording/playback policy, cancellation, mechanisms, and outputs.
- [ ] **Step 3:** Stop if any behavior differs; do not centralize devices.
- [ ] **Step 4:** Commit measured results separately and push.

