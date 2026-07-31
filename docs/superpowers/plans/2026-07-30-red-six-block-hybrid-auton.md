# Red Six-Block Hybrid Autonomous Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a red-side six-block autonomous that showcases smooth LemLib pursuit in open travel while retaining precise loader contact, smooth reverse exit, and reliable long-goal scoring.

**Architecture:** A deterministic PowerShell generator owns two open JerryIO path assets and emits a shared route contract. A platform-independent phase sequencer guarantees that any motion/mechanism failure suppresses later phases. The V5 routine adapts those phases to existing monitored LemLib actions and explicit mechanism actions, initially registering the full routine only in Red AUT3.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, JerryIO/LemLib text assets, PowerShell, MinGW g++ host tests, GNU Make/ARM toolchain, Git.

## Global Constraints

- Target only the small differential robot and red side.
- Use pure pursuit for broad travel, `moveToPose` for loader/goal contact, and chained reverse point/pose actions for loader exit.
- Use no automatic motor-encoder fallback.
- Initial pursuit lookahead is 7 inches for the loader path and 3 inches for the short goal-transfer path.
- Initial maxima: open path 90, staging tails 45, loader contact 30, reverse clearance 60, reverse alignment 40, goal contact 30.
- Start `(0,0,0)`, loader stage `(0,24)`, loader contact `(4,31,86)`, reverse clearance `(-5,31)`, reverse alignment `(-9,31,171)`, goal contact `(-8,25,171)`.
- Every action has an explicit timeout; configured worst-case duration must leave at least 1000 ms inside the competition autonomous period.
- Red AUT1/2 Kevin routines remain unchanged; the hybrid begins in Red AUT3.
- Skills AUT3 is reserved for Shadow playback; do not use it for hybrid phase tests.
- Keep `USING_BIG_ROBOT false` in the committed upload configuration.
- Do not mirror to blue or promote to Red AUT1 until all physical gates pass.

## File Map

- Create `include/aon/auton/red-six-block.hpp`: dependency-free poses, speeds, timeouts, and route policy.
- Create `include/aon/auton/red-six-block-generated.hpp`: generator-owned asset endpoints and point counts.
- Create `tools/generate-red-six-block-paths.ps1`: deterministic Bézier sampling, curvature speed limiting, JerryIO output, and generated staging header update.
- Create `static/red-six-loader-approach.jerryio.txt`: smooth forward loader approach.
- Create `static/red-six-goal-transfer.jerryio.txt`: smooth forward goal transfer.
- Create `include/aon/auton/hybrid-sequence.hpp`: platform-independent phase callback/result contract.
- Create `src/aon/auton/hybrid-sequence.cpp`: strict ordered execution and failure suppression.
- Create `tests/red-six-block-path-test.cpp`: asset geometry, speed, clearance, timeout, deterministic output, and sequencer tests.
- Modify `include/aon/auton/routines.hpp` and `src/aon/auton/lemlib-routines.cpp`: hybrid routine declaration and V5 phase adapters.
- Modify `src/aon/auton/routine-selectors.cpp` and `include/aon/tools/gui/gui.hpp`: Red AUT3 testing registration.
- Create `docs/testing/2026-07-30-red-six-block-checklist.md`: progressive physical gates and tuning log.

---

### Task 1: Deterministic Route Contract and Assets

**Files:**
- Create: `include/aon/auton/red-six-block.hpp`
- Create: `include/aon/auton/red-six-block-generated.hpp`
- Create: `tools/generate-red-six-block-paths.ps1`
- Create: `static/red-six-loader-approach.jerryio.txt`
- Create: `static/red-six-goal-transfer.jerryio.txt`
- Create: `tests/red-six-block-path-test.cpp`

**Interfaces:**
- Produces:

```cpp
namespace aon::auton {

struct RedSixPose {
  double x;
  double y;
  double heading;
};

struct RedSixBlockGenerated {
  static constexpr double loaderStartX = 0.0;
  static constexpr double loaderStartY = 0.0;
  static constexpr double loaderEndX = 0.0;
  static constexpr double loaderEndY = 24.0;
  static constexpr double goalStartX = -9.0;
  static constexpr double goalStartY = 31.0;
  static constexpr double goalStageX = -8.3;
  static constexpr double goalStageY = 27.0;
  static constexpr double goalStageHeading = 171.0;
  static constexpr std::size_t loaderPointCount = 25;
  static constexpr std::size_t goalPointCount = 13;
};

struct RedSixBlock {
  static constexpr const char* name = "RED 6-BLOCK HYBRID";
  static constexpr RedSixPose start{0.0, 0.0, 0.0};
  static constexpr RedSixPose loaderStage{0.0, 24.0, 30.0};
  static constexpr RedSixPose loaderContact{4.0, 31.0, 86.0};
  static constexpr RedSixPose reverseClearance{-5.0, 31.0, 90.0};
  static constexpr RedSixPose reverseAlignment{-9.0, 31.0, 171.0};
  static constexpr RedSixPose goalStage{
      RedSixBlockGenerated::goalStageX,
      RedSixBlockGenerated::goalStageY,
      RedSixBlockGenerated::goalStageHeading};
  static constexpr RedSixPose goalContact{-8.0, 25.0, 171.0};

  static constexpr float loaderLookahead = 7.0F;
  static constexpr float goalLookahead = 3.0F;
  static constexpr int loaderPathTimeoutMs = 3500;
  static constexpr int loaderContactTimeoutMs = 1800;
  static constexpr int collectTimeoutMs = 4000;
  static constexpr int reverseClearanceTimeoutMs = 1600;
  static constexpr int reverseAlignmentTimeoutMs = 1800;
  static constexpr int goalPathTimeoutMs = 2200;
  static constexpr int goalContactTimeoutMs = 1600;
  static constexpr int scoreTimeoutMs = 2700;
  static constexpr int autonomousLimitMs = 15000;
  static constexpr int requiredMarginMs = 1000;
};

}  // namespace aon::auton
```

- The generator and test own any later coordinate tuning; generated assets must never be edited manually.

- [ ] **Step 1: Write the failing asset parser/geometry test**

Create a dependency-free test that parses lines until `endData` into `{x,y,speed}` and validates both named assets. Assert:

- at least 12 points per asset;
- finite coordinates and speeds;
- maximum consecutive point spacing `<= 2.0` inches;
- speeds in `[0,90]`;
- only the final point has speed zero;
- maximum adjacent speed delta `<= 15`;
- endpoints match `start/loaderStage` and `reverseAlignment/goalStage` within `0.05` inches;
- start/end tangents match the shared headings within `8` degrees;
- no non-adjacent line segments intersect;
- curvature radius never forces inner-wheel reversal using half-track width `6.25` inches;
- swept centerline plus 8-inch footprint radius stays inside the configured local test region `x=[-18,18]`, `y=[-8,40]`.

Also assert:

```cpp
constexpr int configuredBudget =
    RedSixBlock::loaderPathTimeoutMs +
    RedSixBlock::loaderContactTimeoutMs +
    RedSixBlock::collectTimeoutMs +
    RedSixBlock::reverseClearanceTimeoutMs +
    RedSixBlock::reverseAlignmentTimeoutMs +
    RedSixBlock::goalPathTimeoutMs +
    RedSixBlock::goalContactTimeoutMs +
    RedSixBlock::scoreTimeoutMs;
static_assert(configuredBudget <=
              RedSixBlock::autonomousLimitMs - RedSixBlock::requiredMarginMs);
```

Use actual configured action deadlines, not estimated motion duration.

- [ ] **Step 2: Compile to prove the route contract is missing**

Run:

```powershell
& 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe' `
  -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\red-six-block-path-test.cpp `
  -o bin\host-tests\red-six-block-path-test.exe
```

Expected: FAIL because `red-six-block.hpp` and both assets do not exist.

- [ ] **Step 3: Add the exact route constants**

Create `red-six-block-generated.hpp` with `#pragma once`, `<cstddef>`, namespace `aon::auton`, and the exact `RedSixBlockGenerated` content shown above. Create `red-six-block.hpp` with `#pragma once`, include the generated header, then define `RedSixPose` and the `RedSixBlock` policy shown above in the same namespace. The initial deadline sum is intentionally too large and must keep the test red until Step 5 replaces the loose values with a feasible budget.

- [ ] **Step 4: Implement deterministic loader geometry**

In the PowerShell generator, sample a cubic Bézier at 25 equally spaced parameters:

```text
P0 = (0,0)
P1 = (0,8)
P2 = (-4,17)
P3 = (0,24)
```

This starts along heading 0 and leaves a smooth final tangent toward loader contact. Compute point spacing, discrete curvature, and a curvature-aware speed cap:

```text
curveCap = 90 / (1 + abs(curvature) * 6.25)
tailCap  = max(45, 90 * remainingArcLength / 10)
speed    = round(clamp(min(curveCap, tailCap), 25, 90))
```

Force only the last speed to zero. Output invariant-culture `x, y, speed` lines plus `endData` and deterministic JerryIO JSON metadata naming the script.

- [ ] **Step 5: Implement deterministic goal geometry and feasible deadlines**

Sample a cubic Bézier at 13 equally spaced parameters:

```text
P0 = (-9,31)
P1 = (-8.77,29.67)
P2 = (-8.53,28.33)
P3 = (-8.3,27)
```

Use maximum speed 45, minimum moving speed 20, and final speed zero. Generate `red-six-block-generated.hpp` from the same control-point arrays used to write the assets. The generator emits the exact endpoint poses and point counts; `red-six-block.hpp` consumes `RedSixBlockGenerated::goalStage` instead of duplicating it.

Update the shared policy constants to this feasible initial deadline budget:

```cpp
static constexpr int loaderPathTimeoutMs = 2000;
static constexpr int loaderContactTimeoutMs = 900;
static constexpr int collectTimeoutMs = 4000;
static constexpr int reverseClearanceTimeoutMs = 800;
static constexpr int reverseAlignmentTimeoutMs = 1000;
static constexpr int goalPathTimeoutMs = 800;
static constexpr int goalContactTimeoutMs = 800;
static constexpr int scoreTimeoutMs = 2700;
```

Total configured budget is `13000 ms`, leaving `2000 ms`. If physical collection or scoring exceeds its deadline, do not increase total above 14000 ms; instead keep the routine in Red AUT3 and revise the mechanism/route plan.

- [ ] **Step 6: Generate and verify deterministic output**

Run:

```powershell
& '.\tools\generate-red-six-block-paths.ps1'
$first = Get-FileHash static\red-six-loader-approach.jerryio.txt,static\red-six-goal-transfer.jerryio.txt,include\aon\auton\red-six-block-generated.hpp
& '.\tools\generate-red-six-block-paths.ps1'
$second = Get-FileHash static\red-six-loader-approach.jerryio.txt,static\red-six-goal-transfer.jerryio.txt,include\aon\auton\red-six-block-generated.hpp
Compare-Object $first.Hash $second.Hash
```

Expected: two generated assets and no `Compare-Object` output.

- [ ] **Step 7: Run the host geometry test**

Compile using Step 2 and run:

```powershell
& '.\bin\host-tests\red-six-block-path-test.exe'
```

Expected: `red six-block path tests passed`.

- [ ] **Step 8: Commit the deterministic route**

```powershell
git add -- include/aon/auton/red-six-block.hpp `
  include/aon/auton/red-six-block-generated.hpp `
  tools/generate-red-six-block-paths.ps1 `
  static/red-six-loader-approach.jerryio.txt `
  static/red-six-goal-transfer.jerryio.txt `
  tests/red-six-block-path-test.cpp
git commit -m "Add validated red six-block pursuit assets"
```

---

### Task 2: Fail-Closed Hybrid Phase Sequencer

**Files:**
- Create: `include/aon/auton/hybrid-sequence.hpp`
- Create: `src/aon/auton/hybrid-sequence.cpp`
- Modify: `tests/red-six-block-path-test.cpp`

**Interfaces:**
- Produces:

```cpp
namespace aon::auton {

enum class RedSixPhase : std::uint8_t {
  LoaderPursuit,
  LoaderContact,
  CollectSix,
  ReverseClearance,
  ReverseAlignment,
  GoalPursuit,
  GoalContact,
  ScoreSix
};

struct RedSixPhaseResult {
  bool succeeded;
  RedSixPhase failedPhase;
};

struct RedSixCallbacks {
  std::function<bool(RedSixPhase)> run;
  std::function<void()> stopAll;
};

RedSixPhaseResult runRedSixSequence(RedSixCallbacks& callbacks,
                                   RedSixPhase stopAfter =
                                       RedSixPhase::ScoreSix);

}  // namespace aon::auton
```

- [ ] **Step 1: Add failing phase-order tests**

Test all eight phases in exact enum order, with `stopAfter` inclusive. For every failure index from 0 through 7, make `run` return false at that phase and assert:

- no later phase runs;
- `failedPhase` is exact;
- `succeeded == false`;
- `stopAll` is called exactly once.

On success, assert all phases run and `stopAll` still runs once. Test `stopAfter=LoaderPursuit`, `LoaderContact`, `CollectSix`, `ReverseAlignment`, and `GoalContact`; these are the physical-gate entry points.

- [ ] **Step 2: Compile and observe the missing sequencer**

Compile the path test with `src\aon\auton\hybrid-sequence.cpp`.

Expected: FAIL because the header/source do not exist.

- [ ] **Step 3: Implement strict ordered execution**

Use one constexpr array:

```cpp
constexpr std::array<RedSixPhase, 8> kRedSixOrder{{
    RedSixPhase::LoaderPursuit,
    RedSixPhase::LoaderContact,
    RedSixPhase::CollectSix,
    RedSixPhase::ReverseClearance,
    RedSixPhase::ReverseAlignment,
    RedSixPhase::GoalPursuit,
    RedSixPhase::GoalContact,
    RedSixPhase::ScoreSix,
}};
```

Call `stopAll` through one finalizer on success, requested partial completion, or first failure. Reject an invalid `stopAfter` value without running a phase.

- [ ] **Step 4: Run and commit**

Run the host test. Expected: `red six-block path tests passed`.

```powershell
git add -- include/aon/auton/hybrid-sequence.hpp `
  src/aon/auton/hybrid-sequence.cpp tests/red-six-block-path-test.cpp
git commit -m "Add fail-closed red six-block phase sequencing"
```

---

### Task 3: Adapt the Hybrid Phases to LemLib and Mechanisms

**Files:**
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/lemlib-routines.cpp`
- Modify: `tests/red-six-block-path-test.cpp`

**Interfaces:**
- Consumes: Task 1 assets/constants, Task 2 `runRedSixSequence`, existing `Actions`, and existing loader/scorer mechanism actions.
- Produces:

```cpp
int RunRedSixBlockHybridAuton(
    aon::auton::RedSixPhase stopAfter =
        aon::auton::RedSixPhase::ScoreSix);
```

- [ ] **Step 1: Add the routine signature check**

Add:

```cpp
static_assert(std::is_same_v<
    decltype(&aon::routines::RunRedSixBlockHybridAuton),
    int (*)(aon::auton::RedSixPhase)>);
```

Compile. Expected: FAIL because the routine is undeclared.

- [ ] **Step 2: Embed both generated assets**

In `lemlib-routines.cpp`:

```cpp
ASSET(red_six_loader_approach_jerryio_txt);
ASSET(red_six_goal_transfer_jerryio_txt);
```

Set the pose once at routine start from `RedSixBlock::start`. Reject `USING_BIG_ROBOT` immediately with a logged unsupported result and zero mechanism motion.

- [ ] **Step 3: Implement exact motion phases**

Map phases as follows:

```cpp
LoaderPursuit:
  actions().followPath("red-six loader pursuit",
      red_six_loader_approach_jerryio_txt,
      RedSixBlock::loaderLookahead,
      RedSixBlock::loaderPathTimeoutMs, true);

LoaderContact:
  actions().moveToPose(4.0, 31.0, 86.0,
      RedSixBlock::loaderContactTimeoutMs,
      {.forwards = true, .maxSpeed = 30});

ReverseClearance:
  actions().moveToPoint(-5.0, 31.0,
      RedSixBlock::reverseClearanceTimeoutMs,
      {.forwards = false, .maxSpeed = 60,
       .minSpeed = 25, .earlyExitRange = 2.5});

ReverseAlignment:
  actions().moveToPose(-9.0, 31.0, 171.0,
      RedSixBlock::reverseAlignmentTimeoutMs,
      {.forwards = false, .maxSpeed = 40,
       .minSpeed = 15, .earlyExitRange = 1.0});

GoalPursuit:
  actions().followPath("red-six goal pursuit",
      red_six_goal_transfer_jerryio_txt,
      RedSixBlock::goalLookahead,
      RedSixBlock::goalPathTimeoutMs, true);

GoalContact:
  actions().moveToPose(-8.0, 25.0, 171.0,
      RedSixBlock::goalContactTimeoutMs,
      {.forwards = true, .maxSpeed = 30});
```

Convert every `MotionResult` to `bool` using `result.succeeded`; log the failing phase and `MotionFailureReason`. Do not continue after false.

- [ ] **Step 4: Implement exact mechanism phases**

Use the exact existing mechanism API:

- Before pursuit: `mechanisms::finishLoaderCollection()` and `mechanisms::prepareLoaderCart()`.
- `CollectSix`: call `mechanisms::beginLoaderCollection()`, then poll for exactly `4000 ms` in 20 ms increments. At each poll, fail if controller-X cancellation is latched or `pros::competition::is_disabled()` becomes true. On completion or failure call `mechanisms::finishLoaderCollection()`.
- Before reverse: call `mechanisms::resetLoaderCart()`.
- At the beginning of `GoalPursuit`: call `mechanisms::prepareTopScorer()` before following the asset.
- `ScoreSix`: call `mechanisms::scoreTopBlocks(2700)`, which blocks for the configured scoring interval and stops the intake through `Intake::score`.

The current small intake exposes cart/scorer piston state feedback but no block-count sensor. Therefore the initial collection and scoring success criterion is bounded completion without cancellation/disable; actual six-block result is a required physical gate. Do not invent a sensor predicate.

- [ ] **Step 5: Implement final stopping**

Supply `runRedSixSequence` a `stopAll` callback that cancels LemLib motion and explicitly stops drivetrain, intake, loader, and scoring outputs. Return `1` only when the sequencer reports success. Partial `stopAfter` gates return `1` after their requested phase and still stop all outputs.

- [ ] **Step 6: Run host and robot compilation**

Run the red-six host suite. Then:

```powershell
make clean
make
```

Temporarily set `USING_BIG_ROBOT true`, clean-build, restore false, and clean-build again. Expected: both configurations compile and the big routine exits unsupported.

- [ ] **Step 7: Commit the V5 routine**

```powershell
git add -- include/aon/auton/routines.hpp `
  src/aon/auton/lemlib-routines.cpp tests/red-six-block-path-test.cpp
git commit -m "Implement red six-block hybrid routine"
```

---

### Task 4: Red AUT3 Registration and Automated Regression Gate

**Files:**
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `tests/red-six-block-path-test.cpp`

**Interfaces:**
- Consumes: Task 3 `RunRedSixBlockHybridAuton`.
- Produces: Red AUT3 entry named `RED 6-BLOCK HYBRID`; Blue AUT3 keeps figure-eight; Skills AUT3 remains reserved for Shadow.

- [ ] **Step 1: Register Red AUT3**

Change only `RedRoutine3`:

```cpp
int RedRoutine3() {
  return runRoutine(aon::auton::RedSixBlock::name,
                    []() {
                      return RunRedSixBlockHybridAuton(
                          aon::auton::RedSixPhase::ScoreSix);
                    });
}
```

Because `runRoutine` currently accepts a function pointer, add a zero-argument wrapper instead of passing a capturing lambda:

```cpp
int RunRedSixBlockHybridFull() {
  return RunRedSixBlockHybridAuton(RedSixPhase::ScoreSix);
}
```

Declare the wrapper in `routines.hpp` and pass `RunRedSixBlockHybridFull`. Keep Blue AUT3 figure-eight.

- [ ] **Step 2: Update the Brain label**

Include `red-six-block.hpp` in `gui.hpp` and replace only the third red option with:

```cpp
{aon::auton::RedSixBlock::name, aon::routines::RedRoutine3},
```

Do not touch Red AUT1, Red AUT2, blue options, or Skills AUT3.

- [ ] **Step 3: Add selector source assertions**

The host test should read `src/aon/auton/routine-selectors.cpp` and `include/aon/tools/gui/gui.hpp` as text and assert:

- `RedRoutine3` references `RunRedSixBlockHybridFull`;
- the red GUI option references `RedSixBlock::name`;
- `BlueRoutine3` still references `RunLemLibFigureEightValidation`;
- neither `SkillsRoutine3` nor the third skills GUI option references the red-six-block routine (if the Shadow plan has already executed, its `RunShadowPlayback` registration remains unchanged; otherwise the native turn test remains unchanged);
- `RedRoutine1` and `RedRoutine2` remain present.

- [ ] **Step 4: Run all automated gates**

Run:

1. `red-six-block-path-test`;
2. `figure-eight-path-test`;
3. `shadow-auton-test`;
4. `motion-fallback-test`;
5. SD integration tests;
6. `git diff --check`;
7. clean small build;
8. clean big build;
9. restore `USING_BIG_ROBOT false` and clean small build.

Expected: every host test and build passes, no new warnings, and the configured deadline budget is `<= 14000 ms`.

- [ ] **Step 5: Commit selector registration**

```powershell
git add -- src/aon/auton/routine-selectors.cpp `
  include/aon/tools/gui/gui.hpp tests/red-six-block-path-test.cpp
git commit -m "Register red six-block hybrid in AUT3"
```

---

### Task 5: Progressive Physical Gates and Promotion Decision

**Files:**
- Create: `docs/testing/2026-07-30-red-six-block-checklist.md`
- Modify only after measured tuning: `include/aon/auton/red-six-block.hpp`
- Regenerate after coordinate/speed tuning: `static/red-six-loader-approach.jerryio.txt`
- Regenerate after coordinate/speed tuning: `static/red-six-goal-transfer.jerryio.txt`

**Interfaces:**
- Consumes: Red AUT3 full routine and the `stopAfter` phase entry points.
- Produces: measured physical evidence; no Red AUT1 promotion in this task.

- [ ] **Step 1: Create the physical test sheet**

Make a table with columns:

```text
Gate | Build commit | Start placement | Path max speed | Elapsed ms |
Final x | Final y | Final heading | Mechanism result | X-cancel |
Pass/fail | Adjustment
```

List these exact gates:

1. `LoaderPursuit`;
2. `LoaderContact` from measured staging pose;
3. `CollectSix` after contact;
4. chained `ReverseClearance` + `ReverseAlignment`;
5. `GoalPursuit` from measured reverse pose;
6. `GoalContact` + `ScoreSix`;
7. full route at generated speed capped to 60;
8. full route at approved speed up to 90;
9. three consecutive cold-boot full runs.

- [ ] **Step 2: Expose one physical gate at a time without consuming another selector**

Before upload, temporarily make `RunRedSixBlockHybridFull()` call the required `stopAfter` value for gates 1-6. Never commit those temporary gate limits. After each upload, restore `ScoreSix` and verify `git diff -- src/aon/auton/routine-selectors.cpp src/aon/auton/lemlib-routines.cpp` is empty except intended measured tuning.

- [ ] **Step 3: Tune through the generator contract**

For any geometry or speed change:

1. edit only `red-six-block.hpp` and `generate-red-six-block-paths.ps1`;
2. regenerate both assets;
3. run the deterministic hash check;
4. run the red-six host test;
5. clean-build small;
6. record the change and measured effect in the checklist.

Never hand-edit either `.jerryio.txt` file.

- [ ] **Step 4: Apply pass/fail rules**

Block advancement on collision, missed contact, pursuit oscillation, an unexpected full stop between reverse actions, odometry fault, missed score, timeout, mechanism failure, or failed X cancellation. Keep Red AUT3 as the test slot until every gate passes.

- [ ] **Step 5: Record the promotion decision**

After three cold-boot successes, record whether the measured total leaves at least 1000 ms. If yes, create a separate promotion change replacing Red AUT1 only after explicit approval. If no, leave Kevin Loader in Red AUT1 and the hybrid in Red AUT3.

- [ ] **Step 6: Commit measured results, not temporary gates**

```powershell
git add -- docs/testing/2026-07-30-red-six-block-checklist.md `
  include/aon/auton/red-six-block.hpp `
  include/aon/auton/red-six-block-generated.hpp `
  tools/generate-red-six-block-paths.ps1 `
  static/red-six-loader-approach.jerryio.txt `
  static/red-six-goal-transfer.jerryio.txt
git commit -m "Record red six-block physical tuning"
```

Before committing, confirm `RunRedSixBlockHybridFull()` again uses `ScoreSix`, `USING_BIG_ROBOT false`, and `git diff --check` is clean.
