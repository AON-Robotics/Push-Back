# C++ Safety and Resource Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct demonstrated lifetime, timing, synchronization, task-start, authorization, memory, and documentation defects without enabling an unvalidated robot feature.

**Architecture:** Keep value-oriented localization and navigation modules. Add only three narrow seams: wrap-safe monotonic-time helpers, a bounded RAII mutex guard, and explicit RTOS task-start results. Configuration remains the single source of authorization truth, while planner scratch storage moves from task stacks into planner-owned bounded workspace.

**Tech Stack:** C++17, PROS 4 RTOS, LemLib, dependency-free host executables, PowerShell test orchestration, ARM GNU Make build.

## Global Constraints

- Exclude vendored PROS, LemLib, LVGL, fmt, and JSON sources.
- Keep `USING_BIG_ROBOT false` in every committed checkpoint.
- Keep automatic fallback, forced encoder testing, Shadow, Red Six Block, JerryIO, GPS, GPS heading, fused LemLib, and fused navigation disabled.
- Do not change controller gains, sensor geometry, route geometry, or field data.
- No allocation from periodic localization, navigation, protocol, or logging loops.
- All time intervals used with wrap-safe helpers must be less than `INT32_MAX` milliseconds.
- Use concise Doxygen briefs only for safety-significant public interfaces touched by this plan.
- Use `-std=c++17 -Wall -Wextra -Werror` for host compilation.
- Commit and push each completed task independently; never stage unrelated concurrent work.

---

## File Structure

- `include/aon/time/monotonic.hpp`: wrap-safe timestamp comparison and elapsed-time policy.
- `include/aon/tools/timed-mutex-lock.hpp`: bounded, non-copyable RAII mutex guard.
- `include/aon/config/localization-config.hpp`: plain localization configuration values, independent of `RobotConfig`.
- `include/aon/core/task-start.hpp`: task-start result and live-state predicate.
- `tests/resource-policy-test.cpp`: compile-time destructor/copy/move and state-size contracts.
- Existing focused tests: behavioral regressions beside navigation, pose history, confidence, localization math, integration, and configuration tests.
- `tools/run-host-tests.ps1`: authoritative dependency-free fused-platform host suite.

### Task 1: Make Polymorphic Ownership Destruction-Safe

**Files:**
- Create: `tests/resource-policy-test.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `include/aon/tools/gui/gui-debug.hpp`
- Modify: `include/aon/drivetrain/drivetrain.hpp`
- Modify: `tools/run-host-tests.ps1`

**Interfaces:**
- Consumes: existing `std::unique_ptr<Gui>` ownership and `Drivetrain` inheritance.
- Produces: non-throwing virtual base destruction and explicit non-copyable/non-movable hardware owners.

- [ ] **Step 1: Write the failing compile-time resource policy test**

Create a dependency-free test containing:

```cpp
#include "aon/drivetrain/drivetrain.hpp"
#include "aon/tools/gui/gui-debug.hpp"

#include <type_traits>

static_assert(std::has_virtual_destructor_v<aon::Gui>);
static_assert(std::has_virtual_destructor_v<aon::Drivetrain>);
static_assert(!std::is_copy_constructible_v<aon::Gui>);
static_assert(!std::is_move_constructible_v<aon::Gui>);
static_assert(!std::is_copy_constructible_v<aon::Drivetrain>);
static_assert(!std::is_move_constructible_v<aon::Drivetrain>);

int main() {}
```

- [ ] **Step 2: Run it and verify the expected failure**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\resource-policy-test.cpp -c `
  -o bin\host-tests\resource-policy-test.o
```

Expected: static assertions for virtual base destruction fail.

- [ ] **Step 3: Declare explicit base ownership policy**

Add to each base class public section:

```cpp
virtual ~Gui() noexcept = default;
Gui(const Gui&) = delete;
Gui& operator=(const Gui&) = delete;
Gui(Gui&&) = delete;
Gui& operator=(Gui&&) = delete;
```

```cpp
virtual ~Drivetrain() noexcept = default;
Drivetrain(const Drivetrain&) = delete;
Drivetrain& operator=(const Drivetrain&) = delete;
Drivetrain(Drivetrain&&) = delete;
Drivetrain& operator=(Drivetrain&&) = delete;
```

Change `GuiDebug` to `~GuiDebug() noexcept override = default;`.

- [ ] **Step 4: Add the test to the host runner and verify green**

Compile the resource test as an object from `tools/run-host-tests.ps1`, then run the complete script. Expected: all host tests pass.

- [ ] **Step 5: Commit and push**

```powershell
git add include/aon/tools/gui/gui.hpp include/aon/tools/gui/gui-debug.hpp `
  include/aon/drivetrain/drivetrain.hpp tests/resource-policy-test.cpp `
  tools/run-host-tests.ps1
git commit -m "fix(resources): make polymorphic ownership destruction-safe"
git push origin Testing
```

### Task 2: Centralize Wrap-Safe Monotonic Time

**Files:**
- Create: `include/aon/time/monotonic.hpp`
- Modify: `tests/pose-history-test.cpp`
- Modify: `tests/localization-confidence-test.cpp`
- Modify: `tests/navigation-test.cpp`
- Modify: `src/aon/localization/pose-history.cpp`
- Modify: `src/aon/localization/confidence.cpp`
- Modify: `src/aon/navigation/dynamic-obstacles.cpp`

**Interfaces:**
- Produces:
  - `time::strictlyAfter(candidate, reference) noexcept`
  - `time::elapsed(now, since) noexcept`
  - `time::elapsedAtLeast(now, since, duration) noexcept`
  - `time::beforeDeadline(now, deadline) noexcept`

- [ ] **Step 1: Add rollover regressions first**

Add cases using `std::numeric_limits<std::uint32_t>::max()`:

```cpp
const auto nearWrap = std::numeric_limits<std::uint32_t>::max() - 10U;
CHECK(history.push(timedPose(nearWrap)) == PoseHistoryPushResult::Accepted);
CHECK(history.push(timedPose(5U)) == PoseHistoryPushResult::Accepted);
CHECK(history.sampleAt(0U, sample) == PoseHistorySampleResult::Interpolated);
```

Add equivalent recovery observations at `nearWrap` then `5U`, and an obstacle inserted at `nearWrap` that remains present before its wrapped deadline and expires exactly at the lifetime.

- [ ] **Step 2: Run focused tests and verify failures**

Expected: pose history reports out-of-order, recovery rejects the sample, and obstacle expiry is wrong.

- [ ] **Step 3: Implement the pure helper**

```cpp
namespace aon::time {
[[nodiscard]] constexpr std::int32_t delta(std::uint32_t later,
                                            std::uint32_t earlier) noexcept {
  return static_cast<std::int32_t>(later - earlier);
}
[[nodiscard]] constexpr bool strictlyAfter(std::uint32_t candidate,
                                            std::uint32_t reference) noexcept {
  return delta(candidate, reference) > 0;
}
[[nodiscard]] constexpr std::uint32_t elapsed(std::uint32_t now,
                                               std::uint32_t since) noexcept {
  return now - since;
}
[[nodiscard]] constexpr bool elapsedAtLeast(std::uint32_t now,
                                             std::uint32_t since,
                                             std::uint32_t duration) noexcept {
  return elapsed(now, since) >= duration;
}
[[nodiscard]] constexpr bool beforeDeadline(std::uint32_t now,
                                             std::uint32_t deadline) noexcept {
  return delta(deadline, now) > 0;
}
}  // namespace aon::time
```

Replace raw ordering in the three implementations. Pose-history interpolation must compare offsets relative to the oldest retained timestamp rather than raw timestamp values.

- [ ] **Step 4: Run focused and full host suites**

Expected: rollover and ordinary-time cases pass.

- [ ] **Step 5: Commit and push**

```powershell
git add include/aon/time/monotonic.hpp tests/pose-history-test.cpp `
  tests/localization-confidence-test.cpp tests/navigation-test.cpp `
  src/aon/localization/pose-history.cpp src/aon/localization/confidence.cpp `
  src/aon/navigation/dynamic-obstacles.cpp
git commit -m "fix(time): handle millisecond rollover consistently"
git push origin Testing
```

### Task 3: Enforce Progress During Final Heading Alignment

**Files:**
- Modify: `tests/navigation-test.cpp`
- Modify: `src/aon/navigation/path-follower.cpp`
- Modify: `include/aon/navigation/path-follower.hpp`

**Interfaces:**
- Consumes: existing `minimumHeadingProgressRadians` and `blockedDwellMs`.
- Produces: `FollowerStatus::Blocked` during stalled final alignment.

- [ ] **Step 1: Write the failing stalled-alignment test**

Start a one-point path with final alignment enabled, hold heading error constant across the dwell period, and assert the last output is stopped with `Blocked`. Add a neighboring case where heading improves by more than `minimumHeadingProgressRadians` and remains `Following`.

- [ ] **Step 2: Verify red**

Run the navigation test. Expected: the constant-heading case remains `Following`.

- [ ] **Step 3: Apply one shared progress policy**

Extract a private helper:

```cpp
[[nodiscard]] bool recordProgress(double distanceErrorInches,
                                  double headingErrorRadians,
                                  std::uint32_t nowMs) noexcept;
```

It updates both best errors and `lastProgressAtMs_`; it returns false after `blockedDwellMs`. Call it in path travel and final alignment before commanding motors.

- [ ] **Step 4: Verify navigation and host suites**

- [ ] **Step 5: Commit and push**

```powershell
git add include/aon/navigation/path-follower.hpp `
  src/aon/navigation/path-follower.cpp tests/navigation-test.cpp
git commit -m "fix(navigation): stop stalled final alignment"
git push origin Testing
```

### Task 4: Commit GPS History Only After EKF Acceptance

**Files:**
- Modify: `include/aon/odometry/sensor-measurements.hpp`
- Modify: `tests/localization-math-test.cpp`
- Modify: `src/aon/odometry.cpp`

**Interfaces:**
- Changes `GpsGate::evaluate(GpsMeasurement)` to a const validation operation.
- Adds `GpsGate::commit(GpsMeasurement, bool positionAccepted, bool headingAccepted) noexcept`.

- [ ] **Step 1: Write the rejected-outlier recovery test**

Validate an in-bounds outlier, deliberately do not commit it to represent EKF rejection, then validate a nearby correct sample. Assert the correct sample is not rejected as a jump. Also verify a committed sample becomes the next jump baseline.

- [ ] **Step 2: Verify red**

Expected: the second sample is rejected because `evaluate()` mutated history.

- [ ] **Step 3: Split validation from commit**

Make `evaluate()` `const noexcept` and remove all state writes. Add:

```cpp
void commit(GpsMeasurement measurement, bool positionAccepted,
            bool headingAccepted) noexcept;
```

Commit timestamp and position only when `positionAccepted`; commit heading only when `headingAccepted`.

- [ ] **Step 4: Wire the EKF decision**

In `Odometry::update()`, run EKF position and heading updates first, then call:

```cpp
candidateGpsGate.commit(gpsMeasurement,
                        diagnostics.gpsPositionAccepted,
                        diagnostics.gpsHeadingAccepted);
```

- [ ] **Step 5: Verify localization and integration suites**

- [ ] **Step 6: Commit and push**

```powershell
git add include/aon/odometry/sensor-measurements.hpp `
  tests/localization-math-test.cpp src/aon/odometry.cpp
git commit -m "fix(localization): commit only fused GPS samples"
git push origin Testing
```

### Task 5: Reject Every Non-Finite Pose Component

**Files:**
- Modify: `include/aon/odometry/pose-estimator.hpp`
- Modify: `tests/localization-math-test.cpp`
- Modify: `src/aon/odometry.cpp`

**Interfaces:**
- Produces `localization::isFinite(EstimatorPose) noexcept`.

- [ ] **Step 1: Write table-driven finite-pose tests**

Assert the helper rejects NaN and infinity independently in X, Y, and heading, and accepts a fully finite pose.

- [ ] **Step 2: Verify red because the helper is absent**

- [ ] **Step 3: Implement and consume the helper**

```cpp
[[nodiscard]] inline bool isFinite(EstimatorPose pose) noexcept {
  return std::isfinite(pose.xInches) && std::isfinite(pose.yInches) &&
         std::isfinite(pose.headingRadians);
}
```

Replace the X-only check in `Odometry::update()` and duplicated whole-pose checks in touched localization/navigation code.

- [ ] **Step 4: Run localization, navigation, and full host suites**

- [ ] **Step 5: Commit and push**

```powershell
git add include/aon/odometry/pose-estimator.hpp `
  tests/localization-math-test.cpp src/aon/odometry.cpp
git commit -m "fix(localization): reject partially non-finite poses"
git push origin Testing
```

### Task 6: Cover Every Runtime Authorization and Break Config Coupling

**Files:**
- Create: `include/aon/config/localization-config.hpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `include/aon/odometry/odometry.hpp`
- Modify: `tests/authorization-policy-test.cpp`
- Modify: `tests/authorization-source-test.ps1`
- Modify: `tests/localization-config-test.cpp`

**Interfaces:**
- Extends `AuthorizationSnapshot` with `gpsHardware`, `gpsHeadingFusion`, `fusedLemLib`, and `fusedNavigation`.
- Moves `GpsHardwareConfig` and `LocalizationConfig` without changing field names or values.

- [ ] **Step 1: Extend failing authorization tests**

Use named field mutation to assert each new gate independently makes `safeForUnvalidatedBaseline()` false. Assert both active robot baselines keep all nine gates disabled. Extend the source test to require the new fields in the snapshot mapping.

- [ ] **Step 2: Verify red**

Expected: missing members or false-positive safe verdict.

- [ ] **Step 3: Move plain config values and complete the snapshot**

Create `localization-config.hpp` containing only required standard headers and localization value-type headers. Map:

```cpp
config.localization.gps.enabled,
config.localization.gps.headingUpdateEnabled,
config.localization.fusedLemLibAuthorized,
config.localization.fusedNavigationAuthorized,
```

into the snapshot and verdict.

- [ ] **Step 4: Verify include locality**

Compile `authorization-policy-test`, `localization-config-test`, and `localization-integration-test` with warnings as errors. Use `g++ -M` to confirm `odometry.hpp` no longer includes `robot-config.hpp` transitively.

- [ ] **Step 5: Run the full host suite**

- [ ] **Step 6: Commit and push**

```powershell
git add include/aon/config/localization-config.hpp `
  include/aon/config/robot-config.hpp src/aon/config/robot-config.cpp `
  include/aon/odometry/odometry.hpp tests/authorization-policy-test.cpp `
  tests/authorization-source-test.ps1 tests/localization-config-test.cpp
git commit -m "fix(config): cover every physical runtime gate"
git push origin Testing
```

### Task 7: Bound Mutex Waits and Report Task-Start Failure

**Files:**
- Create: `include/aon/tools/timed-mutex-lock.hpp`
- Create: `include/aon/core/task-start.hpp`
- Modify: `tests/resource-policy-test.cpp`
- Modify: `include/aon/odometry/diagnostics.hpp`
- Modify: `include/aon/odometry/odometry.hpp`
- Modify: `src/aon/odometry.cpp`
- Modify: `include/aon/lemlib/chassis.hpp`
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `include/aon/drivetrain/legacy-motion.hpp`
- Modify: `src/aon/drivetrain/legacy-motion.cpp`

**Interfaces:**
- Produces `TimedMutexLock<Mutex>` with `ownsLock()`.
- Produces `TaskStartResult { Disabled, Started, AlreadyRunning, Failed }`.
- Changes `startFusedLocalization()` and `legacy_motion::prepare()` to return `TaskStartResult`.
- Adds `snapshotLockTimeouts` to localization diagnostics.

- [ ] **Step 1: Write failing fake-mutex and task-state tests**

Use a fake mutex that records `take(timeout)` and `give()` calls. Assert success releases once, failure never releases, and the guard is neither copyable nor movable. Add pure state assertions that running, ready, blocked, and suspended are live; deleted and invalid are not.

- [ ] **Step 2: Verify red**

- [ ] **Step 3: Implement the bounded RAII guard**

```cpp
template <typename Mutex>
class TimedMutexLock {
 public:
  TimedMutexLock(Mutex& mutex, std::uint32_t timeoutMs) noexcept
      : mutex_(mutex), owns_(mutex_.take(timeoutMs)) {}
  ~TimedMutexLock() noexcept { if (owns_) (void)mutex_.give(); }
  TimedMutexLock(const TimedMutexLock&) = delete;
  TimedMutexLock& operator=(const TimedMutexLock&) = delete;
  TimedMutexLock(TimedMutexLock&&) = delete;
  TimedMutexLock& operator=(TimedMutexLock&&) = delete;
  [[nodiscard]] bool ownsLock() const noexcept { return owns_; }
 private:
  Mutex& mutex_;
  bool owns_;
};
```

- [ ] **Step 4: Replace odometry manual locking**

Use a constant two-millisecond timeout. Readers return a non-finite pose or default diagnostics marked with an incremented timeout count when acquisition fails. Update/reset paths skip their state commit on failure. Never access `published_`, estimator state, or baselines without owning the lock.

- [ ] **Step 5: Validate actual task state**

Construct a task into a local `unique_ptr`, call `get_state()`, and retain it only for a live state. Catch `std::bad_alloc`, return `Failed`, and leave storage empty so a later call retries. Existing callers must handle `Failed` by logging and leaving outputs stopped.

- [ ] **Step 6: Run host tests and clean ARM builds early**

This task touches PROS-only behavior. Clean-build big, restore `USING_BIG_ROBOT false`, and clean-build small before committing.

- [ ] **Step 7: Commit and push**

```powershell
git add include/aon/tools/timed-mutex-lock.hpp include/aon/core/task-start.hpp `
  tests/resource-policy-test.cpp include/aon/odometry/diagnostics.hpp `
  include/aon/odometry/odometry.hpp src/aon/odometry.cpp `
  include/aon/lemlib/chassis.hpp src/aon/lemlib/chassis.cpp `
  include/aon/drivetrain/legacy-motion.hpp src/aon/drivetrain/legacy-motion.cpp
git commit -m "fix(runtime): bound locks and validate task startup"
git push origin Testing
```

### Task 8: Move Planner Scratch State Off Task Stacks

**Files:**
- Modify: `include/aon/navigation/path-planner.hpp`
- Modify: `src/aon/navigation/path-planner.cpp`
- Modify: `tests/resource-policy-test.cpp`
- Modify: `tests/navigation-test.cpp`

**Interfaces:**
- Adds private `PathPlanner::Workspace`; keeps `plan(...)` and path capacity unchanged.
- Makes `plan(...)` non-const because it reuses owned scratch state.

- [ ] **Step 1: Add state-size and repeatability tests**

Assert fixed budgets:

```cpp
static_assert(sizeof(aon::navigation::PathPlanner) <= 16U * 1024U);
static_assert(sizeof(aon::navigation::PathPlanner) >
              sizeof(aon::navigation::PathPlannerConfig));
static_assert(sizeof(aon::communication::FrameParser) <= 512U);
static_assert(sizeof(aon::navigation::DynamicObstacleMap) <= 4U * 1024U);
```

Run the same planner instance repeatedly across success, unreachable, and success requests to prove workspace clearing.

- [ ] **Step 2: Verify the planner ownership assertion fails**

Expected: `sizeof(PathPlanner) > sizeof(PathPlannerConfig)` fails because the
current planner owns only its config and places scratch arrays on the caller's
stack. The behavioral repeat test may already pass.

- [ ] **Step 3: Move scratch arrays into one private workspace**

Define a value-initialized `Workspace workspace_{};` holding nodes, visibility, distances, predecessors, visited flags, and route reconstruction storage. At the beginning of `plan()`, reset only the used ranges. Do not allocate.

- [ ] **Step 4: Run navigation, resource, and full host suites**

- [ ] **Step 5: Commit and push**

```powershell
git add include/aon/navigation/path-planner.hpp `
  src/aon/navigation/path-planner.cpp tests/resource-policy-test.cpp `
  tests/navigation-test.cpp
git commit -m "refactor(navigation): reuse bounded planner workspace"
git push origin Testing
```

### Task 9: Standardize Safety Briefs and Complete Verification

**Files:**
- Modify only public headers changed in Tasks 1–8.
- Modify: `docs/CURRENT_HANDOFF.md`
- Modify: `docs/superpowers/plans/2026-08-10-cpp-safety-and-resource-hardening.md`

**Interfaces:**
- No behavior changes.

- [ ] **Step 1: Add concise contracts in touched headers**

For each safety-significant public method, document observable behavior, units, bounded timeout/rollover assumptions, ownership when non-obvious, and typed failure. Remove comments that merely repeat names. Wrap source to `.clang-format`'s 80-column limit without bulk-formatting unrelated code.

- [ ] **Step 2: Run all host verification fresh**

Run `tools/run-host-tests.ps1` plus the established authorization, hardware-map, vector, figure-eight, motion-fallback, red-six-block, JerryIO, Shadow SD, and Shadow host executables. Expected: every executable passes with warnings as errors.

- [ ] **Step 3: Clean-build both ARM configurations**

Build big from a temporary `USING_BIG_ROBOT true` edit, restore false immediately, then clean-build small. Expected: link succeeds; only previously documented vendored warnings may remain.

- [ ] **Step 4: Verify repository policy**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File tests\authorization-source-test.ps1
rg -n "#define USING_BIG_ROBOT|Authorized|gps.enabled|headingUpdateEnabled" `
  include/aon/constants.hpp include/aon/config src/aon/config
git diff --check
```

Expected: selected robot is small, every physical gate is false, and the diff is clean.

- [ ] **Step 5: Run two-axis review**

Review the hardening commit range against the approved design and repository standards. Fix every correctness or spec finding and rerun affected verification.

- [ ] **Step 6: Update the handoff without physical claims**

Record host/build evidence, memory sizes, remaining physical gate, and exact next action. Keep all physical checklist results `Not run`.

- [ ] **Step 7: Commit and push**

```powershell
git add docs/CURRENT_HANDOFF.md `
  docs/superpowers/plans/2026-08-10-cpp-safety-and-resource-hardening.md `
  include/aon/config/localization-config.hpp `
  include/aon/config/robot-config.hpp include/aon/core/task-start.hpp `
  include/aon/drivetrain/drivetrain.hpp include/aon/lemlib/chassis.hpp `
  include/aon/navigation/path-follower.hpp `
  include/aon/navigation/path-planner.hpp `
  include/aon/odometry/diagnostics.hpp include/aon/odometry/odometry.hpp `
  include/aon/odometry/pose-estimator.hpp `
  include/aon/odometry/sensor-measurements.hpp `
  include/aon/time/monotonic.hpp include/aon/tools/gui/gui.hpp `
  include/aon/tools/gui/gui-debug.hpp `
  include/aon/tools/timed-mutex-lock.hpp
git commit -m "docs: complete C++ safety hardening checkpoint"
git push origin Testing
```
