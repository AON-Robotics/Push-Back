# Push-Back Refactor Phases 2-5 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Improve autonomous observability, mechanism naming, LemLib validation, and repository organization while preserving the currently proven native robot behavior.

**Architecture:** Keep competition slot selection separate from route bodies. Route bodies call small autonomous status and mechanism-action interfaces, while drivetrain implementations remain native PROS or LemLib. Structural moves happen only after observable behavior has been characterized, and every task ends at a buildable checkpoint.

**Tech Stack:** C++17, PROS, LemLib, LVGL/PROS brain screen, GNU Make, Git.

## Global Constraints

- Preserve motor ports, motor reversals, piston ports, brake modes, gains, timeouts, route distances, route headings, and mechanism timing unless a physical test demonstrates a defect.
- Keep Okapi compatibility types only where current native drivetrain interfaces still consume them; new autonomous motion uses LemLib or existing native PROS wrappers.
- Keep native Kevin Loader in red and blue slot 1 and make red slot 1 the boot default.
- Keep each commit independently buildable for both `USING_BIG_ROBOT false` and `USING_BIG_ROBOT true`.
- Do not modify vendored files under `include/pros`, `include/lemlib`, `include/liblvgl`, or `include/fmt`.
- After every phase, compare remaining risk with expected value. Stop and request permission when physical behavior is unknown or the next phase changes hardware-facing behavior.
- Never run a full autonomous route as the first physical test of a new drivetrain configuration.

---

## Existing Baseline

- Phase 1 is complete at commit `6c3e04c`.
- Autonomous declarations live in `include/aon/auton/routines.hpp`.
- Slot mapping lives in `src/aon/auton/routine-selectors.cpp`.
- Native route and test bodies currently share `src/aon/auton/native-routines.cpp`.
- LemLib tests and experiments live in `src/aon/auton/lemlib-routines.cpp`.
- LemLib motion logging already exists in `aon::auton::Actions`.
- GUI selection persistence and debug autonomous status already exist, but competition execution does not consistently update the same state.
- `aon::core::Hardware` owns devices; `include/aon/globals.hpp` exposes compatibility references.
- There is no host-side automated test suite. Builds and narrowly scoped robot tests are the available regression checks.

## Verification Commands

Run the small-robot build with:

```powershell
$toolchain = "$env:USERPROFILE\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin"
$env:PATH = "$toolchain;$env:PATH"
& "$toolchain\make.exe" -j4
```

For the big-robot build, temporarily change only `#define USING_BIG_ROBOT false` to `true` in `include/aon/constants.hpp`, run the same command, then restore `false` and rebuild. Confirm `git diff -- include/aon/constants.hpp` is empty before every commit.

---

## Phase 2: Autonomous Reliability

### Task 2.1: Central Autonomous Runtime Status

**Files:**
- Create: `include/aon/auton/status.hpp`
- Create: `src/aon/auton/status.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `src/aon/core/robot.cpp`

**Interfaces:**
- Produces: `enum class RoutineState { Idle, Selected, Running, Completed, Failed };`
- Produces: `selectRoutine(const char*)`, `startRoutine(const char*)`, `finishRoutine(bool)`, `routineStatus()`.
- Consumes: PROS screen and standard output only; it must not own or command hardware.

- [x] **Step 1: Add the status interface**

```cpp
namespace aon::auton {

enum class RoutineState { Idle, Selected, Running, Completed, Failed };

struct RoutineStatus {
  const char* name;
  RoutineState state;
  std::uint32_t startedAt;
  std::uint32_t finishedAt;
};

void selectRoutine(const char* name);
void startRoutine(const char* name);
void finishRoutine(bool succeeded);
RoutineStatus routineStatus();

}  // namespace aon::auton
```

- [x] **Step 2: Implement status updates and serial logging**

Protect the shared status with a PROS mutex because GUI and competition callbacks run in different tasks. Each transition prints one structured line using the keys `AUTON_STATE`, `name`, `state`, and `time`.

- [x] **Step 3: Route all competition slot invocations through one named wrapper**

Replace the anonymous native-only wrapper with a wrapper accepting the displayed name and callable. The wrapper must call `startRoutine`, invoke the route, stop drivetrain and intake, call `finishRoutine(true)`, and return the route result. Do not catch failures that C++/PROS cannot safely recover from.

- [x] **Step 4: Remove the direct experimental fallback from robot initialization**

Keep GUI selection authoritative. The boot fallback must register `RedRoutine1`, matching the required native default, until GUI initialization applies a persisted selection.

- [x] **Step 5: Build both robot configurations**

Expected: both builds complete successfully; the existing vendored `json.hpp` deprecation warning is acceptable.

- [x] **Step 6: Commit**

```powershell
git add include/aon/auton/status.hpp src/aon/auton/status.cpp src/aon/auton/routine-selectors.cpp src/aon/core/robot.cpp
git commit -m "Add shared autonomous runtime status"
```

### Task 2.2: Brain-Screen Selected and Running Status

**Files:**
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`
- Modify: `src/aon/tools/gui/ui/gui-displays.cpp`
- Modify: `src/aon/tools/gui/debug-tools/autonrunner.cpp`

**Interfaces:**
- Consumes: `aon::auton::routineStatus()`.
- Produces: one consistent selected/running/completed label for normal and debug GUI modes.

- [x] **Step 1: Make `Gui` selection update shared status**

After `selectedAutonName` is assigned in `selectAutonByList`, call `aon::auton::selectRoutine(selectedAutonName.c_str())`.

- [x] **Step 2: Render status without clearing the active GUI**

Add a small status region to the existing main-menu drawing function. Render `Selected: <name>`, `Running: <name>`, `Completed: <name>`, or `Failed: <name>` from the shared status snapshot. Keep drawing inside the existing GUI task to avoid concurrent full-screen erases.

- [x] **Step 3: Make debug auton runner consume the same status**

Remove duplicate state transitions where `autonRunning` and `autonCompleted` can disagree with competition execution. Preserve the fields temporarily if other code still reads them, but derive their display from `routineStatus()`.

- [x] **Step 4: Build both robot configurations**

- [x] **Step 5: Commit**

```powershell
git add include/aon/tools/gui/gui.hpp src/aon/tools/gui/gui.cpp src/aon/tools/gui/ui/gui-displays.cpp src/aon/tools/gui/debug-tools/autonrunner.cpp
git commit -m "Show autonomous selection and execution status"
```

### Task 2.3: Native Kevin Route Step Logging

**Files:**
- Create: `include/aon/auton/step-logger.hpp`
- Create: `src/aon/auton/step-logger.cpp`
- Modify: `src/aon/auton/native-routines.cpp`

**Interfaces:**
- Produces: `logStep(const char* routine, const char* step)`.
- Consumes: time and console output only.

- [x] **Step 1: Add the logger**

```cpp
namespace aon::auton {

void logStep(const char* routine, const char* step);

}  // namespace aon::auton
```

Output format:

```text
AUTON_STEP routine=Kevin Loader step=align loader time=1234
```

- [x] **Step 2: Instrument `smallBotRoutine` and `bigBotStayThere`**

Place logs only at behavioral boundaries: start, align loader, collect blocks, approach long goal, score, park setup, finish. Do not insert delays or reorder commands.

- [x] **Step 3: Instrument `smallBotPark` and `bigBotPark`**

Log start, parking push, piston activation, and finish. Do not change velocities or durations.

- [x] **Step 4: Build both robot configurations**

- [x] **Step 5: Commit**

```powershell
git add include/aon/auton/step-logger.hpp src/aon/auton/step-logger.cpp src/aon/auton/native-routines.cpp
git commit -m "Add step logging to native Kevin autonomous routes"
```

### Task 2.4: Safe Drivetrain-Only Test Slots

**Files:**
- Modify: `include/aon/auton/routines.hpp`
- Create: `src/aon/auton/drivetrain-tests.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`

**Interfaces:**
- Produces: native low-speed forward/reverse test and native low-speed turn test.
- Constraint: tests must not command intake, cart, scorer, Brooks, Arrow, or SEM.

- [x] **Step 1: Add named drivetrain test routines**

The forward/reverse test moves 6 inches forward, pauses 500 ms, and moves 6 inches backward using the current native drivetrain. The turn test turns 45 degrees clockwise, pauses 500 ms, and returns 45 degrees counterclockwise. Both stop the drivetrain on completion.

- [x] **Step 2: Expose tests only through skills/debug slots**

Do not replace red or blue competition slots. Give each test a precise brain-screen label containing `TEST`.

- [x] **Step 3: Build both robot configurations**

- [x] **Step 4: Commit**

```powershell
git add include/aon/auton/routines.hpp src/aon/auton/drivetrain-tests.cpp src/aon/auton/routine-selectors.cpp include/aon/tools/gui/gui.hpp
git commit -m "Add isolated native drivetrain test routines"
```

### Task 2.5: Restore Proven Native Default

**Files:**
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `src/aon/core/robot.cpp`

- [x] **Step 1: Set red slot 1 as the preselection**

Set `selectedRedAut = 1`, `selectedBlueAut = 0`, and `selectedSkill = 0`. Register `RedRoutine1` as the initialization fallback. A valid saved SD-card selection takes precedence over this first-run fallback.

- [x] **Step 2: Build both robot configurations**

- [x] **Step 3: Commit**

```powershell
git add include/aon/tools/gui/gui.hpp src/aon/core/robot.cpp
git commit -m "Restore Kevin Loader as the default autonomous"
```

### Phase 2 Risk Gate

- Confirm the GUI loads and displays Kevin Loader before selection input.
- Confirm a changed selection survives reboot.
- Run only the drivetrain tests with the robot on a clear field or raised safely.
- Confirm native Kevin Loader and Kevin Park logs reach expected steps without changing observed motion.
- Stop before Phase 3 if GUI task contention, selection loss, or altered native behavior appears.

---

## Phase 3: Mechanism Interfaces

### Task 3.1: Introduce Named Mechanism Actions

**Files:**
- Create: `include/aon/auton/mechanism-actions.hpp`
- Create: `src/aon/auton/mechanism-actions.cpp`

**Interfaces:**
- Produces: a small stateless facade over the existing global hardware compatibility references.

- [x] **Step 1: Add only operations already repeated by active routes**

```cpp
namespace aon::auton::mechanisms {

void beginLoaderCollection();
void finishLoaderCollection();
void prepareLoaderCart();
void resetLoaderCart();
void prepareTopScorer();
void scoreTopBlocks(std::uint32_t durationMs);
void deployParkMechanism();
void stopAll();

}  // namespace aon::auton::mechanisms
```

- [x] **Step 2: Implement exact delegation**

Each function must contain only the current intake or piston calls. Do not add delays, drivetrain motion, retries, sensor decisions, or new safety policy.

- [x] **Step 3: Build both robot configurations**

- [x] **Step 4: Commit**

```powershell
git add include/aon/auton/mechanism-actions.hpp src/aon/auton/mechanism-actions.cpp
git commit -m "Add named autonomous mechanism actions"
```

### Task 3.2: Migrate Kevin Loader Mechanism Calls

**Files:**
- Modify: `src/aon/auton/native-routines.cpp`

- [x] **Step 1: Replace only matching mechanism call sequences in `smallBotRoutine` and `bigBotStayThere`**

Keep every drivetrain command and delay at its original position. Use action names only where their implementation is exactly equivalent to the replaced calls.

- [x] **Step 2: Compare the diff command by command**

Run:

```powershell
git diff --word-diff=plain -- src/aon/auton/native-routines.cpp
```

Expected: drivetrain calls, numeric parameters, and delays are unchanged.

- [x] **Step 3: Build both robot configurations**

- [x] **Step 4: Commit**

```powershell
git add src/aon/auton/native-routines.cpp
git commit -m "Route Kevin Loader mechanisms through named actions"
```

### Task 3.3: Migrate Kevin Park and LemLib Experiment Mechanisms

**Files:**
- Modify: `src/aon/auton/native-routines.cpp`
- Modify: `src/aon/auton/lemlib-routines.cpp`

- [x] **Step 1: Replace equivalent park and intake operations**

Preserve each route's timing and order. The LemLib route remains experimental and must not become the default.

- [x] **Step 2: Build both robot configurations**

- [x] **Step 3: Commit**

```powershell
git add src/aon/auton/native-routines.cpp src/aon/auton/lemlib-routines.cpp
git commit -m "Use named mechanism actions in park and LemLib routes"
```

### Phase 3 Risk Gate

- Compare serial step sequences before and after the helper migration.
- Physically run native slot 1 and native slot 2 once each.
- Stop before Phase 4 if any mechanism activates at a different time, remains running, or changes direction.

---

## Phase 4: LemLib Validation

### Task 4.1: Add LemLib Validation Configuration and Results Format

**Files:**
- Create: `include/aon/auton/lemlib-validation.hpp`
- Create: `src/aon/auton/lemlib-validation.cpp`
- Modify: `include/aon/auton/routines.hpp`

**Interfaces:**
- Produces: isolated forward, reverse, clockwise-turn, counterclockwise-turn, and combined tests.
- Consumes: `aon::auton::Actions` and `activeRobotConfig()`.

- [ ] **Step 1: Define conservative validation routines**

Use 12 inches for straight tests, 90 degrees for turn tests, maximum speed 40, and explicit timeouts. Every test sets pose to `(0, 0, 0)`, logs expected motion, runs one motion, stops, and prints final pose.

- [ ] **Step 2: Add a stable result line**

```text
LEMLIB_VALIDATION test=forward expected_x=0 expected_y=12 expected_h=0 actual_x=... actual_y=... actual_h=...
```

- [ ] **Step 3: Build both robot configurations**

- [ ] **Step 4: Commit**

```powershell
git add include/aon/auton/lemlib-validation.hpp src/aon/auton/lemlib-validation.cpp include/aon/auton/routines.hpp
git commit -m "Add isolated LemLib validation routines"
```

### Task 4.2: Expose One LemLib Test at a Time

**Files:**
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`

- [ ] **Step 1: Map the forward test to one clearly labeled skills slot**

Do not expose all tests simultaneously. Keep native red and blue slots unchanged.

- [ ] **Step 2: Build both robot configurations**

- [ ] **Step 3: Commit**

```powershell
git add src/aon/auton/routine-selectors.cpp include/aon/tools/gui/gui.hpp
git commit -m "Expose the first LemLib validation slot"
```

### Task 4.3: Physical Validation Sequence

**Files:**
- Modify after measurements only: `src/aon/config/robot-config.cpp`
- Modify after measurements only: `src/aon/lemlib/chassis.cpp`
- Modify: `docs/superpowers/plans/2026-07-11-push-back-refactor-phases-2-5.md`

- [ ] **Step 1: Run forward test five times**

Record commanded distance, measured physical distance, final X/Y/heading, battery state, and whether either side moved backward.

- [ ] **Step 2: Run reverse test five times after forward passes**

- [ ] **Step 3: Run clockwise 90-degree test five times after straight tests pass**

- [ ] **Step 4: Run counterclockwise 90-degree test five times**

- [ ] **Step 5: Change one calibration category per commit**

Motor reversal, tracking-wheel diameter, tracking offsets, track width, and PID gains are separate calibration categories. Never combine them in one tuning commit.

- [ ] **Step 6: Commit recorded calibration only after repeatable improvement**

Use a message naming the measurement changed, for example:

```powershell
git commit -m "Calibrate small robot LemLib lateral tracking offset"
```

### Task 4.4: Validate LoaderScore in Segments

**Files:**
- Modify: `src/aon/auton/lemlib-routines.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`

- [ ] **Step 1: Split the experimental route into independently invocable segments**

Use named internal functions for loader approach, collection and retreat, scoring approach, and parking setup. Preserve the current full-route sequence.

- [ ] **Step 2: Expose only the first unvalidated segment in a skills/debug slot**

- [ ] **Step 3: Advance segment exposure only after five acceptable repetitions**

- [ ] **Step 4: Build and commit each newly validated segment separately**

Commit messages must identify the validated segment, for example `Validate LemLib loader approach segment`.

### Task 4.5: Retry JerryIO Last

**Files:**
- Modify: `src/aon/auton/lemlib-routines.cpp`
- Modify only if the asset changes: `static/path.jerryio.txt`

- [ ] **Step 1: Confirm all previous LemLib tests pass**

Do not proceed based only on successful compilation.

- [ ] **Step 2: Verify path start pose, heading convention, units, and forward direction against the exported asset**

- [ ] **Step 3: Run at reduced speed in a clear field**

- [ ] **Step 4: Commit only an asset/configuration combination that produces repeatable geometry**

### Phase 4 Risk Gate

- Require five repeatable runs for each primitive before combining primitives.
- Stop immediately for wrong motor direction, large lateral drift, heading wrap errors, or unsafe acceleration.
- Keep Kevin Loader as the competition default throughout LemLib validation.

---

## Phase 5: Repository Organization

### Task 5.1: Move Native Test Functions Out of Route Code

**Files:**
- Create: `include/aon/auton/native-tests.hpp`
- Create: `src/aon/auton/native-tests.cpp`
- Modify: `src/aon/auton/native-routines.cpp`

**Interfaces:**
- Moves existing test bodies without renaming or changing them.
- Keeps competition route declarations in `include/aon/auton/routines.hpp`.

- [x] **Step 1: Move functions from `alignRobotTo` through `xDriveRoutine` into the test file**

Use `git diff --color-moved` to confirm the bodies are moves rather than rewrites.

- [x] **Step 2: Include only headers required by each resulting translation unit**

- [x] **Step 3: Build both robot configurations**

- [x] **Step 4: Commit**

```powershell
git add include/aon/auton/native-tests.hpp src/aon/auton/native-tests.cpp src/aon/auton/native-routines.cpp
git commit -m "Separate native drivetrain tests from autonomous routes"
```

### Task 5.2: Split Native Routes by Robot and Purpose

**Files:**
- Create: `src/aon/auton/native-big-robot.cpp`
- Create: `src/aon/auton/native-small-robot.cpp`
- Create: `src/aon/auton/native-skills.cpp`
- Delete after successful moves: `src/aon/auton/native-routines.cpp`

- [x] **Step 1: Move big-robot route bodies unchanged**

- [x] **Step 2: Build both configurations and commit**

```powershell
git commit -m "Separate big robot native autonomous routes"
```

- [x] **Step 3: Move small-robot route bodies unchanged**

- [x] **Step 4: Build both configurations and commit**

```powershell
git commit -m "Separate small robot native autonomous routes"
```

- [x] **Step 5: Move skills route bodies and remove the empty source file**

- [x] **Step 6: Build both configurations and commit**

```powershell
git commit -m "Separate native skills autonomous routes"
```

### Task 5.3: Narrow the Public Autonomous Header

**Files:**
- Modify: `include/aon/auton/routines.hpp`
- Create: `include/aon/auton/native-routes.hpp`
- Create: `include/aon/auton/lemlib-routes.hpp`

- [x] **Step 1: Keep only competition slot entry points in `routines.hpp`**

- [x] **Step 2: Put native route bodies and LemLib experimental entry points in their respective internal headers**

- [x] **Step 3: Build both robot configurations**

- [x] **Step 4: Commit**

```powershell
git add include/aon/auton/routines.hpp include/aon/auton/native-routes.hpp include/aon/auton/lemlib-routes.hpp src/aon/auton
git commit -m "Narrow autonomous route interfaces"
```

### Task 5.4: Reduce Autonomous Global Access One Dependency at a Time

**Files:**
- Modify: `include/aon/core/hardware.hpp`
- Modify: autonomous source files one at a time

- [x] **Step 1: Pass `aon::core::Hardware&` into mechanism actions**

Keep `globals.hpp` compatibility references for untouched code. Do not migrate drivetrain, GUI, intake tasks, and operator control simultaneously.

- [ ] **Step 2: Convert one autonomous translation unit per commit**

Build both configurations after each conversion. A commit must not mix dependency migration with route tuning or file movement.

- [ ] **Step 3: Stop when remaining globals are shared with operator control or background tasks**

Those require a separate concurrency and lifecycle plan rather than mechanical replacement.

### Phase 5 Risk Gate

- Confirm `git diff --color-moved` shows structural moves rather than behavioral rewrites.
- Run both native competition slots after the final file split.
- Leave globals used by operator control, intake scanning/sorting tasks, or GUI lifecycle in place until a dedicated concurrency plan exists.

---

## Final Review

- Run the small and big builds from a clean worktree.
- Confirm red slot 1 is Kevin Loader and remains the default.
- Confirm red/blue competition slots still map to their intended native routines.
- Confirm LemLib validation routines remain clearly labeled and non-default.
- Review every commit for one responsibility and an explicit rollback point.
- Push the `Testing` branch only after the phase risk gate passes.

## Out of Scope

- Rewriting Kevin routes in LemLib before primitive validation passes.
- Mirroring red routes into blue field coordinates without field testing.
- Replacing the native drivetrain implementation used by proven routes.
- Removing Okapi compatibility types from unrelated drivetrain APIs.
- Reworking intake scanning/sorting concurrency.
- Changing hardware ports, motor reversals, gearsets, PID gains, or physical calibration without measurements.
- Editing vendored PROS, LemLib, LVGL, fmt, or JSON sources.
