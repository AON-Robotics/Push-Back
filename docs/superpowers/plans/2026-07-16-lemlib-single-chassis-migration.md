# LemLib Single-Chassis Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make LemLib the sole long-term drivetrain and odometry owner while preserving native Kevin routines until validated LemLib replacements exist.

**Architecture:** Isolate the legacy motion stack behind an explicit compatibility lifecycle, validate LemLib primitives, migrate routes one at a time, then delete duplicate motion ownership. GUI and mechanism behavior remain unchanged throughout the drivetrain migration.

**Tech Stack:** C++17, PROS 4, LemLib 0.5, GNU Make, Git.

## Global Constraints

- Keep Kevin Loader and Kevin Park selectable until their LemLib replacements pass physical testing.
- Keep every commit buildable for `USING_BIG_ROBOT false` and `true`.
- Restore `USING_BIG_ROBOT false` and rebuild before every checkpoint.
- Change only one calibration category per tuning commit.
- Do not change mechanism timing while migrating drivetrain motion.
- Do not run a full autonomous before its component movements pass.
- Do not edit vendored libraries.

---

### Task 1: Lazy Legacy Odometry Compatibility

**Files:**
- Create: `include/aon/drivetrain/legacy-motion.hpp`
- Create: `src/aon/drivetrain/legacy-motion.cpp`
- Modify: `src/aon/core/robot.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`

**Interfaces:**
- Produces: `aon::legacy_motion::prepare()`.
- Consumes: the existing global native drivetrain only inside the compatibility implementation.

- [x] Add an idempotent `prepare()` that owns one process-lifetime PROS task running `drivetrain.initialize()` and waits briefly for its first update.
- [x] Remove unconditional native odometry task construction from `Robot::initialize()`.
- [x] Call `prepare()` before every native routine and native drivetrain test, including native Skills.
- [x] Verify LemLib routes do not call `prepare()`.
- [x] Clean-build both hardware configurations.
- [x] Commit as `Start legacy odometry only for native motion`.
- [ ] Physically run AUT3 after reboot, then one native fallback after a separate reboot.

### Task 2: LemLib Primitive Validation Suite

**Files:**
- Create: `include/aon/auton/lemlib-validation.hpp`
- Create: `src/aon/auton/lemlib-validation.cpp`
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`

**Interfaces:**
- Produces: forward 12-inch, reverse 12-inch, clockwise 90-degree,
  counterclockwise 90-degree, and drive-turn-drive routines.
- Produces stable `LEMLIB_VALIDATION` result lines with expected and actual pose.

- [ ] Move the existing forward validation into the dedicated module without changing behavior.
- [ ] Add the reverse test at maximum speed 35 and a 3000 ms timeout.
- [ ] Add isolated clockwise and counterclockwise tests at maximum speed 40.
- [ ] Add the combined test only after individual primitives pass.
- [ ] Expose one unvalidated test at a time through AUT3.
- [ ] Clean-build both configurations and commit each newly exposed primitive separately.

### Task 3: Record and Apply Calibration

**Files:**
- Modify after measurements: `src/aon/config/robot-config.cpp`
- Modify after measurements: `src/aon/lemlib/chassis.cpp`
- Modify: this plan

- [ ] Record five runs per primitive: physical result, final pose, heading, battery state, and drift direction.
- [ ] Correct motor or sensor reversal first if direction is wrong.
- [ ] Calibrate tracking-wheel diameter from straight-distance measurements.
- [ ] Calibrate tracking offsets from in-place turns.
- [ ] Tune lateral PID, angular PID, then slew in separate commits.
- [ ] Reject any calibration change that does not improve repeated results.

### Task 4: Complete LemLib Actions

**Files:**
- Modify: `include/aon/auton/actions.hpp`
- Modify: `src/aon/auton/actions.cpp`

**Interfaces:**
- Produces: `turnToPoint`, async motion variants, `waitUntil`, and `waitUntilDone`.
- Preserves existing synchronous methods and logging.

- [ ] Add `turnToPoint` with the same synchronous logging contract as existing actions.
- [ ] Add explicitly named async variants rather than changing current defaults.
- [ ] Add distance/angle wait and completion wait operations.
- [ ] Add a validation routine proving one mid-motion callback boundary.
- [ ] Build both configurations and commit API additions independently from route changes.

### Task 5: Migrate Kevin Park

**Files:**
- Create: `src/aon/auton/lemlib-kevin-park.cpp`
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`

- [ ] Measure Park start pose and terminal contact point.
- [ ] Implement drivetrain motion only, leaving the park mechanism disabled.
- [ ] Validate drivetrain motion repeatedly.
- [ ] Add `deployParkMechanism()` at the validated boundary.
- [ ] Keep `Native Kevin Park` selectable until the full replacement passes.
- [ ] Commit each validated segment separately.

### Task 6: Migrate Kevin Loader in Segments

**Files:**
- Create: `src/aon/auton/lemlib-kevin-loader.cpp`
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`

- [ ] Measure poses for loader approach, loader contact, retreat, long-goal approach, and parking approach.
- [ ] Validate loader approach without mechanisms.
- [ ] Add collection actions using async distance boundaries.
- [ ] Validate retreat and scoring approach.
- [ ] Add scoring and parking actions only after drivetrain segments pass.
- [ ] Keep `Native Kevin Loader` selectable until the complete replacement passes.
- [ ] Commit each segment separately.

### Task 7: Remove Legacy Motion Stack

**Files:**
- Modify: `src/aon/core/hardware.cpp`
- Modify: `include/aon/core/hardware.hpp`
- Modify: `include/aon/globals.hpp`
- Delete after call-site removal: `include/aon/drivetrain/legacy-motion.hpp`
- Delete after call-site removal: `src/aon/drivetrain/legacy-motion.cpp`
- Modify or delete legacy drivetrain/odometry files only when `rg` shows no active consumers.

- [ ] Switch competition slots from native fallbacks to validated LemLib routes.
- [ ] Prove no active route calls native motion APIs.
- [ ] Remove lazy legacy odometry lifecycle.
- [ ] Remove duplicate drivetrain and sensor ownership from active hardware composition.
- [ ] Remove unused profiles, PIDs, globals, and Okapi compatibility dependencies one group per commit.
- [ ] Clean-build and physically verify driver control and both competition routes.

### Task 8: Final Organization and Documentation

**Files:**
- Modify: autonomous headers and sources based on remaining ownership.
- Modify: `docs/superpowers/plans/2026-07-11-push-back-refactor-phases-2-5.md`
- Modify: this plan

- [ ] Separate competition routes, validation tests, and compatibility fallbacks.
- [ ] Narrow public headers to slot entry points and stable action interfaces.
- [ ] Remove stale comments and completed compatibility notes.
- [ ] Record final hardware configuration and tuning values.
- [ ] Clean-build both configurations and commit documentation separately.

## Phase Gate

After every task, compare physical risk against expected value. Stop when the
next task requires unrecorded field measurements or when a fallback behaves
differently. Compilation alone never authorizes deleting a fallback.
