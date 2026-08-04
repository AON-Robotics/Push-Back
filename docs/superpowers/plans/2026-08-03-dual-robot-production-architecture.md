# Dual-Robot Production Architecture Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the source-edited small/big robot switch with one shared competition framework linked into two independently buildable production executables, each assembled by a concrete robot composition with one hardware owner, a robot-specific autonomous catalog, and no unsupported capabilities exposed.

**Architecture:** Keep robot-neutral implementation under `include/aon` and `src/aon`, and add `apps/small-robot` and `apps/big-robot` as link-time composition roots. Introduce seams only where two real adapters exist; do not add a polymorphic `RobotComposition`, universal `Mechanisms`, or service-locator `RobotContext`. Migrate behavior in characterized checkpoints, preserve the current compatibility path until physical gates pass, and finish by archiving the shared sources as `libaon.a` and linking both named robot images against it.

**Tech Stack:** C++17 host tests, PROS 4.2.2, LemLib 0.5.6, Arm GNU Toolchain 14.3.1, GCC 13.1 host compiler, GNU Make, PowerShell, Git.

## Global Constraints

- Preserve every current motor port, sensor port, reversal, calibration value, autonomous behavior, task period, authorization flag, and initialization order unless a separately approved behavioral checkpoint explicitly changes it.
- Preserve and report the known big-robot right-tracking reversal mismatch; do not repair it as part of architectural migration.
- Do not authorize automatic encoder fallback, forced encoder testing, new Shadow playback, or any currently gated physical behavior.
- Treat `docs/CURRENT_HANDOFF.md` physical gates as hard stops. Compilation and host tests do not satisfy a physical gate.
- Build both production targets from a clean state at every robot-code checkpoint.
- Never edit a source header to switch robots after Task 2.
- Keep the shared library free of `AON_TARGET_SMALL`, `AON_TARGET_BIG`, `USING_BIG_ROBOT`, and includes from either app directory by the end of Task 11.
- Construct each physical PROS motor, sensor, controller, and ADI device exactly once in its robot's hardware owner after Task 9.
- A robot-specific capability must be absent from the other robot's type and autonomous catalog; do not represent absence with a no-op or capability query.
- Keep `FileStore` as a real seam because production SD and host in-memory adapters exist. Add other abstract interfaces only where at least two adapters are named in this plan.
- Keep embedded production code allocation-bounded where the existing implementation is fixed-capacity.
- Do not edit vendored PROS, LemLib, LVGL, fmt, firmware, or `include/aon/tools/json.hpp` files.
- Preserve unexpected user changes. The hardware-map checkpoint landed as `56bfca4`; Task 1 verifies that prerequisite before Task 2 begins.

---

## Target File Structure

Shared files remain in their established include/source roots to avoid a low-value mass move:

```text
include/aon/
  app/                         shared competition lifecycle
  auton/                       actions, scheduling, status, pure sequences
  config/                      shared identities and configuration vocabulary
  drive/                       driver-output seam and robot-neutral drive values
  motion/                      motion ownership, health, cancellation, fallback
  shadow/                      recorder, processor, codec, storage, scheduler
  safety/                      emergency-stop coordination
  diagnostics/                 status and logging interfaces
  tools/gui/                   GUI driven by injected catalogs and state
  compat/                      temporary legacy adapters only

src/aon/                       shared implementations compiled into libaon.a

apps/small-robot/
  include/aon/robots/small/
    config.hpp
    hardware.hpp
    mechanisms.hpp
    operator-control.hpp
    autonomous-program.hpp
    composition.hpp
  src/
    main.cpp
    config.cpp
    hardware.cpp
    mechanisms.cpp
    shadow-mechanisms.cpp
    operator-control.cpp
    autonomous-program.cpp
    autonomous/
      native-routines.cpp
      lemlib-routines.cpp

apps/big-robot/                corresponding big-robot files

tests/
  architecture/
  shared/
  small-robot/
  big-robot/

mk/
  aon-shared-library.mk
  robot-executable.mk
```

## Stable Interfaces Used Across Tasks

The following names are the intended end-state contracts. Later tasks must not silently rename them.

```cpp
namespace aon {

enum class RobotIdentity : std::uint8_t { Small = 1, Big = 2 };

enum class RobotMode : std::uint8_t {
  Disabled,
  Autonomous,
  OperatorControl,
};

namespace config {
struct SafetyPolicy {
  bool automaticFallbackAuthorized = false;
  bool forcedEncoderTestingAuthorized = false;
  bool shadowPlaybackAuthorized = false;
};
}  // namespace config

enum class BrakeMode : std::uint8_t { Coast, Brake, Hold };

struct DriveIntent {
  int forward = 0;
  int sideways = 0;
  int turn = 0;
};

struct RobotManifest {
  RobotIdentity identity;
  const char* displayName;
  config::SafetyPolicy safety;
};

namespace auton {
enum class RoutineOutcome : std::uint8_t {
  Succeeded,
  Failed,
  Cancelled,
  Unsupported,
};

struct RoutineResult {
  RoutineOutcome outcome = RoutineOutcome::Failed;
  MotionFailureReason motionFailure = MotionFailureReason::None;
};

struct AutonomousRoutine {
  const char* id;
  const char* label;
  Alliance alliance;
  RoutineResult (*run)(void* context);
  void* context;
};

struct AutonomousCatalog {
  const AutonomousRoutine* routines;
  std::size_t count;
};
}  // namespace auton

class Startup {
 public:
  virtual ~Startup() = default;
  virtual void initialize() = 0;
};

class CompetitionState {
 public:
  virtual ~CompetitionState() = default;
  virtual RobotMode mode() const = 0;
};

class DriveOutput {
 public:
  virtual ~DriveOutput() = default;
  virtual void command(const DriveIntent& intent) = 0;
  virtual void stop(BrakeMode mode) = 0;
};

class PoseSource {
 public:
  virtual ~PoseSource() = default;
  virtual Pose pose() const = 0;
  virtual void setPose(Pose pose) = 0;
  virtual auton::MotionFailureReason health() const = 0;
};

class OperatorControl {
 public:
  virtual ~OperatorControl() = default;
  virtual void runStep() = 0;
};

class AutonomousProgram {
 public:
  virtual ~AutonomousProgram() = default;
  virtual const auton::AutonomousCatalog& catalog() const = 0;
  virtual auton::RoutineResult runSelected() = 0;
  virtual void cancel() = 0;
};

class EmergencyStop {
 public:
  virtual ~EmergencyStop() = default;
  virtual void stopAll() = 0;
};

class CompetitionApp {
 public:
  CompetitionApp(Startup& startup, AutonomousProgram& autonomous,
                 OperatorControl& operatorControl,
                 EmergencyStop& emergencyStop,
                 CompetitionState& competitionState);
  void initialize();
  void disabled();
  void competitionInitialize();
  void autonomous();
  void opcontrol();
};

}  // namespace aon
```

`CompetitionApp` is the deep lifecycle module. The app composition functions construct concrete dependencies and return the fully assembled module:

```cpp
namespace aon::robots::small {
CompetitionApp& competitionApp();
}

namespace aon::robots::big {
CompetitionApp& competitionApp();
}
```

There is deliberately no `RobotComposition` base class and no runtime `createActiveRobot()`.

---

### Task 1: Verify the completed hardware-map characterization prerequisite

**Files:**
- Existing plan: `docs/superpowers/plans/2026-08-03-hardware-map-consistency.md`
- Verify: `include/aon/config/hardware-map.hpp`
- Verify: `src/aon/config/hardware-map.cpp`
- Test: `tests/hardware-map-test.cpp`
- Verify checkpoint: commit `56bfca4`

**Interfaces:**
- Consumes: existing signed PROS port conventions.
- Produces: `smallRobotHardwareMap`, `bigRobotHardwareMap`, and `validateHardwareMap(const RobotHardwareMap&) noexcept`.

- [ ] **Step 1: Confirm the prerequisite checkpoint and clean worktree**

Run:

```powershell
git show --stat --oneline 56bfca4
git status --short --branch
```

Expected: `56bfca4 Centralize robot hardware port maps` is reachable from `Testing`, and no unexpected worktree changes overlap its production/test files.

- [ ] **Step 2: Verify the known configurations**

Run:

```powershell
& .\bin\host-tests\hardware-map-test.exe
```

Expected: the small map reports `HardwareMapIssue::None`; the big map reports exactly `HardwareMapIssue::RightTrackingReversalMismatch`.

- [ ] **Step 3: Confirm the checkpoint's recorded embedded verification**

Read the completed checklist in `docs/superpowers/plans/2026-08-03-hardware-map-consistency.md` and the commit diff. Expected: small and big compiled, the committed target remained small, and no port or reversal value changed. If that evidence is incomplete, rerun the plan's complete host and embedded verification matrix before Task 2; do not create a duplicate commit.

---

### Task 2: Add two named production build targets without changing behavior

**Files:**
- Create: `mk/robot-targets.mk`
- Create: `tools/build-all.ps1`
- Create: `tests/architecture/build-targets-test.ps1`
- Modify: `Makefile`
- Modify: `common.mk`
- Modify: `include/aon/constants.hpp`
- Modify: `README.md`

**Interfaces:**
- Consumes: the existing monolithic source tree and `USING_BIG_ROBOT` branches.
- Produces: `make small`, `make big`, `make robots`, `bin/small/monolith.bin`, and `bin/big/monolith.bin`.

- [ ] **Step 1: Write the failing target-definition test**

Create `tests/architecture/build-targets-test.ps1`:

```powershell
$makefile = Get-Content -Raw -LiteralPath 'Makefile'
$targets = Get-Content -Raw -LiteralPath 'mk/robot-targets.mk'

foreach ($required in @('small:', 'big:', 'robots:')) {
  if (-not $targets.Contains($required)) {
    throw "missing build target: $required"
  }
}
if (-not $makefile.Contains('-include ./mk/robot-targets.mk')) {
  throw 'Makefile does not load robot targets'
}
if ((Get-Content -Raw 'include/aon/constants.hpp').Contains(
      '#define USING_BIG_ROBOT false')) {
  throw 'robot selection is still source-edited'
}
Write-Output 'dual build target tests passed'
```

- [ ] **Step 2: Run the test and verify RED**

```powershell
& tests\architecture\build-targets-test.ps1
```

Expected: failure because `mk/robot-targets.mk` does not exist.

- [ ] **Step 3: Move the temporary selection to build definitions**

Replace the editable definition in `include/aon/constants.hpp` with the transitional compatibility definition:

```cpp
#if defined(AON_TARGET_BIG)
#define USING_BIG_ROBOT true
#elif defined(AON_TARGET_SMALL)
#define USING_BIG_ROBOT false
#else
#error "Build with exactly one of AON_TARGET_SMALL or AON_TARGET_BIG"
#endif
```

Add a mutual-exclusion check above it:

```cpp
#if defined(AON_TARGET_SMALL) && defined(AON_TARGET_BIG)
#error "A robot build cannot select both production targets"
#endif
```

This macro is a migration bridge only and is deleted in Task 11.

- [ ] **Step 4: Add isolated recursive build targets**

Create `mk/robot-targets.mk`:

```make
.PHONY: small big robots clean-small clean-big

small:
	$(MAKE) quick ROBOT_TARGET=small BINDIR=bin/small DEPDIR=.d/small \
	  EXTRA_CXXFLAGS="$(EXTRA_CXXFLAGS) -DAON_TARGET_SMALL"

big:
	$(MAKE) quick ROBOT_TARGET=big BINDIR=bin/big DEPDIR=.d/big \
	  EXTRA_CXXFLAGS="$(EXTRA_CXXFLAGS) -DAON_TARGET_BIG"

robots: small big

clean-small:
	$(MAKE) clean BINDIR=bin/small DEPDIR=.d/small

clean-big:
	$(MAKE) clean BINDIR=bin/big DEPDIR=.d/big
```

At the end of `Makefile`, add:

```make
-include ./mk/robot-targets.mk
```

Update the `BINDIR` assignment in `Makefile` and the `DEPDIR` assignment in `common.mk` to use `?=` rather than overwriting command-line values. Do not alter compiler, linker, or optimization flags.

- [ ] **Step 5: Add the build-all entry point**

Create `tools/build-all.ps1` to run the host suite, `make clean-small small`, and `make clean-big big`, aborting on the first nonzero exit code. Use the existing compiler and PROS toolchain paths documented in the latest plans; do not download dependencies.

- [ ] **Step 6: Verify GREEN and distinct artifacts**

```powershell
& tests\architecture\build-targets-test.ps1
make clean-small small
make clean-big big
Get-Item bin\small\monolith.bin, bin\big\monolith.bin
```

Expected: both files exist and neither build modifies a source file.

- [ ] **Step 7: Commit the build checkpoint**

```powershell
git add -- Makefile common.mk mk/robot-targets.mk tools/build-all.ps1 `
  tests/architecture/build-targets-test.ps1 include/aon/constants.hpp README.md
git diff --cached --check
git commit -m "Add separate small and big robot builds"
```

---

### Task 3: Unify robot identity and split immutable target configuration

**Files:**
- Create: `include/aon/config/robot-identity.hpp`
- Create: `include/aon/config/safety-policy.hpp`
- Create: `apps/small-robot/include/aon/robots/small/config.hpp`
- Create: `apps/small-robot/src/config.cpp`
- Create: `apps/big-robot/include/aon/robots/big/config.hpp`
- Create: `apps/big-robot/src/config.cpp`
- Create: `tests/small-robot/config-test.cpp`
- Create: `tests/big-robot/config-test.cpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `include/aon/shadow/types.hpp`
- Modify: `src/aon/config/robot-config.cpp`

**Interfaces:**
- Produces: `aon::RobotIdentity`, `aon::config::SafetyPolicy`, `small::config()`, and `big::config()`.
- Preserves temporarily: `activeRobotConfig()` as a compatibility adapter selected by the executable target.

- [ ] **Step 1: Write identity and policy tests**

Each target test must assert:

```cpp
static_assert(static_cast<std::uint8_t>(aon::RobotIdentity::Small) == 1);
static_assert(static_cast<std::uint8_t>(aon::RobotIdentity::Big) == 2);
CHECK(aon::robots::small::config().manifest.identity ==
      aon::RobotIdentity::Small);
CHECK(aon::robots::small::config().manifest.safety.shadowPlaybackAuthorized);
CHECK(!aon::robots::big::config().manifest.safety.shadowPlaybackAuthorized);
```

Also characterize both fallback authorization flags as `false`.

- [ ] **Step 2: Verify RED**

Compile each dependency-free test with its target `config.cpp`. Expected: missing headers and functions.

- [ ] **Step 3: Introduce the stable serialized identity**

Create:

```cpp
namespace aon {
enum class RobotIdentity : std::uint8_t { Small = 1, Big = 2 };
}
```

Replace `config::RobotIdentity` with `aon::RobotIdentity`. In Shadow, use the same type internally and keep explicit codec validation of serialized values `1` and `2`.

- [ ] **Step 4: Define target-local configuration types**

`small::RobotConfig` contains only the small differential drive, tracking, intake, pneumatics, tuning, and safety values. `big::RobotConfig` contains only the H-drive, big intake/sorter, pneumatics, tuning, and safety values. Both reference the Task 1 hardware maps rather than repeating ports.

- [ ] **Step 5: Keep a narrow compatibility adapter**

Make `activeRobotConfig()` delegate to the link-selected target configuration while legacy callers remain. Mark the declaration with a comment naming Task 11 as its deletion point. Do not let new code call it.

- [ ] **Step 6: Verify both tests, all Shadow codec tests, and both embedded builds**

Expected: serialized robot IDs remain byte-compatible and no authorization changes.

- [ ] **Step 7: Commit**

```powershell
git add -- include/aon/config/robot-identity.hpp include/aon/config/safety-policy.hpp `
  apps/small-robot apps/big-robot tests/small-robot tests/big-robot `
  include/aon/config/robot-config.hpp include/aon/shadow/types.hpp `
  src/aon/config/robot-config.cpp
git diff --cached --check
git commit -m "Split immutable robot target configuration"
```

---

### Task 4: Introduce the shared lifecycle module and concrete composition roots

**Files:**
- Create: `include/aon/app/competition-app.hpp`
- Create: `include/aon/app/startup.hpp`
- Create: `include/aon/app/operator-control.hpp`
- Create: `include/aon/app/autonomous-program.hpp`
- Create: `include/aon/safety/emergency-stop.hpp`
- Create: `src/aon/app/competition-app.cpp`
- Create: `tests/competition-app-test.cpp`
- Create: `apps/small-robot/include/aon/robots/small/composition.hpp`
- Create: `apps/small-robot/src/composition.cpp`
- Create: `apps/small-robot/src/main.cpp`
- Create: corresponding big-robot files
- Modify: `mk/robot-targets.mk`
- Delete after both targets link: `src/main.cpp`
- Retain temporarily: `include/aon/core/robot.hpp`, `src/aon/core/robot.cpp` as compatibility implementation details

**Interfaces:**
- Produces: the stable interfaces in this plan's “Stable Interfaces” section.
- Consumes temporarily: existing `core::Robot`, global hardware, GUI, Actions, and Shadow implementations through target-local adapters.

- [ ] **Step 1: Write a lifecycle delegation test**

Create fakes recording calls and assert:

```cpp
app.initialize();
CHECK(startup.initializeCalls == 1);

app.autonomous();
CHECK(autonomous.runCalls == 1);

app.disabled();
CHECK(autonomous.cancelCalls == 1);
CHECK(emergencyStop.stopCalls == 1);

operatorControl.stopAfterOneStep = true;
app.opcontrol();
CHECK(operatorControl.stepCalls == 1);
```

Make the operator loop testable by injecting a `CompetitionState` whose fake reports operator control for one iteration and then disabled.

- [ ] **Step 2: Verify RED, then implement the minimal deep lifecycle module**

`CompetitionApp` owns ordering and mode transitions. It must not expose its dependencies or contain robot identity branches.

- [ ] **Step 3: Add concrete target roots**

Each `composition.cpp` constructs process-lifetime target objects and returns `CompetitionApp&`. Initially, adapters may delegate into existing globals, but all such uses stay inside the target composition or `compat/`.

- [ ] **Step 4: Add separate PROS callback wrappers**

Each app `main.cpp` contains only the five callbacks forwarding to its namespace-specific `competitionApp()`.

- [ ] **Step 5: Make target source selection exclusive**

Update `mk/robot-targets.mk` so the small link includes only `apps/small-robot/src` and the big link includes only `apps/big-robot/src`. Add a link test that fails if both callback wrappers are selected.

- [ ] **Step 6: Build and inspect both images**

Expected: both link, each boot string names its robot, and neither includes the other target's display name when inspected with the toolchain `strings` utility.

- [ ] **Step 7: Commit**

```powershell
git add -- include/aon/app include/aon/safety src/aon/app `
  tests/competition-app-test.cpp apps mk/robot-targets.mk src/main.cpp
git diff --cached --check
git commit -m "Add shared lifecycle and robot composition roots"
```

---

### Task 5: Replace conditional selectors with injected autonomous catalogs

**Files:**
- Create: `include/aon/auton/catalog.hpp`
- Create: `include/aon/auton/routine-result.hpp`
- Create: `src/aon/auton/catalog.cpp`
- Create: `tests/autonomous-catalog-test.cpp`
- Create/modify: `apps/small-robot/src/autonomous-program.cpp`
- Create/modify: `apps/big-robot/src/autonomous-program.cpp`
- Move robot-specific bodies from: `src/aon/auton/native-routines.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`

**Interfaces:**
- Produces: `AutonomousRoutine`, `AutonomousCatalog`, and `RoutineResult`.
- Consumes: existing routine bodies and current GUI selection persistence.

- [ ] **Step 1: Write catalog behavior tests**

Assert unique stable IDs, valid labels, alliance grouping, three-slot GUI projection, selection by stable ID, and rejection of an unknown ID. Assert small contains Red Six Block, JerryIO, and authorized Shadow; assert big contains its loader, park, and skills routines but no Shadow playback or small-only routes.

- [ ] **Step 2: Verify RED and implement the dependency-free catalog**

Use the `AutonomousRoutine` and `AutonomousCatalog` definitions from the stable interfaces section. They form a non-owning array view compatible with C++17:

```cpp
const AutonomousCatalog catalog{routines.data(), routines.size()};
```

The catalog owns no GUI or hardware logic.

- [ ] **Step 3: Build target-specific catalogs**

Move small-only and big-only routine bodies to their app directories. Keep pure route constants, shared sequence helpers, motion health, and Actions in `src/aon`.

- [ ] **Step 4: Inject catalogs into GUI**

Remove routine arrays from `Gui`. Its constructor receives `const AutonomousCatalog&`; saved selection stores the stable routine ID plus alliance. GUI invokes selection through `AutonomousProgram`, not `FunctionReader`.

- [ ] **Step 5: Delete selector preprocessing**

Remove every `USING_BIG_ROBOT` branch from `routine-selectors.cpp`. Delete the file when its wrapping/status behavior has moved into the shared autonomous executor and target catalogs.

- [ ] **Step 6: Run catalog, path, GUI compile, and both embedded target tests**

Expected: current displayed slots remain unchanged for the corresponding robot, and unsupported routines are absent rather than returning `UnsupportedRobot` after selection.

- [ ] **Step 7: Commit**

```powershell
git add -- include/aon/auton src/aon/auton apps tests/autonomous-catalog-test.cpp `
  include/aon/tools/gui/gui.hpp src/aon/tools/gui/gui.cpp
git diff --cached --check
git commit -m "Inject robot-specific autonomous catalogs"
```

---

### Task 6: Split operator control into input, intent, and target adapters

**Files:**
- Create: `include/aon/opcontrol/input-snapshot.hpp`
- Create: `include/aon/opcontrol/drive-intent.hpp`
- Create: `include/aon/opcontrol/operator-loop.hpp`
- Create: `src/aon/opcontrol/operator-loop.cpp`
- Create: `tests/operator-intent-test.cpp`
- Create/modify: small and big `operator-control.cpp`
- Modify then delete: `include/aon/competition/operator-control.hpp`

**Interfaces:**
- Produces: `InputSnapshot`, `DriveIntent`, and the existing `OperatorControl::runStep()` seam.
- Consumes: controller input, robot-specific mechanisms, drive output, and Shadow capture.

- [ ] **Step 1: Characterize Kevin and Fabian decisions with pure tests**

Cover joystick scaling, turbo, double-tap timing, intake intents, sorting mode, and every button edge currently used. Tests return intents and never construct PROS devices.

- [ ] **Step 2: Implement shared value types**

```cpp
struct InputSnapshot {
  int leftX = 0;
  int leftY = 0;
  int rightX = 0;
  int rightY = 0;
  std::uint32_t nowMs = 0;
  ButtonSnapshot buttons{};
};
```

- [ ] **Step 3: Implement target operator adapters**

`SmallOperatorControl::runStep()` reads one snapshot, applies Kevin mapping, executes small drive/mechanism intents, and records effective semantic Shadow events. `BigOperatorControl::runStep()` does the same for Fabian mapping and big sorting.

- [ ] **Step 4: Preserve effective-drive capture**

Assert Shadow receives the clamped command actually sent to motors, not raw joystick values. Preserve the existing atomic effective-drive command behavior.

- [ ] **Step 5: Remove operator preprocessing and global state**

Move `sortActive`, double-tap timestamps, and merge/sort state into the corresponding operator adapter instance. Delete the old header after no source includes it.

- [ ] **Step 6: Run host tests and both target builds**

Expected: no robot conditional remains in operator control and current button behavior is characterized unchanged.

- [ ] **Step 7: Commit**

```powershell
git add -- include/aon/opcontrol src/aon/opcontrol apps `
  tests/operator-intent-test.cpp include/aon/competition/operator-control.hpp
git diff --cached --check
git commit -m "Split robot operator-control adapters"
```

---

### Task 7: Split robot mechanisms and capability-specific route dependencies

**Files:**
- Create: `include/aon/mechanisms/block-handler.hpp`
- Create: `include/aon/mechanisms/parking-mechanism.hpp`
- Create: target-specific mechanism headers and implementations
- Create: `tests/small-robot/mechanisms-test.cpp`
- Create: `tests/big-robot/mechanisms-test.cpp`
- Split then delete: `include/aon/intake/intake.hpp`
- Split then delete: `src/aon/intake.cpp`
- Modify: `include/aon/auton/mechanism-actions.hpp`
- Modify: `src/aon/auton/mechanism-actions.cpp`
- Modify: target routine bodies

**Interfaces:**
- Produces common capabilities only where both targets implement the same semantics: `BlockHandler` and `ParkingMechanism`.
- Produces target-only concrete types: `SmallIntake`, `BigIntake`, `SmallScoringMechanisms`, and `BigSortingMechanisms`.

- [ ] **Step 1: Write semantic adapter tests**

For both robots, test `store`, `scoreBottom`, `stop`, and parking. For small only, test reject, corridor, scorer height, trapdoor, lever, and arrow. For big only, test middle/top scoring, sort-normal, sort-inverted, bounded sort-stop acknowledgement, and Sem.

- [ ] **Step 2: Split the preprocessor-selected Intake type**

Move the existing branches without behavioral edits into concrete target files. Preserve task synchronization, motor ownership, delays, and current state-machine transitions.

- [ ] **Step 3: Replace the universal mechanism idea with required capabilities**

Shared routes accept only common semantic interfaces. Small-only routes receive `SmallScoringMechanisms&`; big-only routes receive `BigSortingMechanisms&`. Do not add `hasTrapdoor()`, `supportsSort()`, or no-op methods.

- [ ] **Step 4: Split Shadow mechanism execution adapters**

Keep pure event validation and planning in shared Shadow code. Move hardware execution to `small/shadow-mechanisms.cpp` and `big/shadow-mechanisms.cpp`.

- [ ] **Step 5: Remove all mechanism robot conditionals**

Require:

```powershell
$hits = rg -n 'USING_BIG_ROBOT' include\aon\intake src\aon\intake.cpp `
  src\aon\auton\mechanism-actions.cpp src\aon\shadow\mechanisms.cpp
if ($LASTEXITCODE -eq 0) { throw $hits }
```

- [ ] **Step 6: Run all mechanism/Shadow host tests and both embedded builds**

Expected: serialized events remain compatible and wrong-robot files still fail before hardware calls.

- [ ] **Step 7: Commit**

```powershell
git add -- include/aon/mechanisms apps tests/small-robot tests/big-robot `
  include/aon/intake src/aon/intake.cpp include/aon/auton/mechanism-actions.hpp `
  src/aon/auton/mechanism-actions.cpp src/aon/shadow/mechanisms.cpp
git diff --cached --check
git commit -m "Split small and big robot mechanisms"
```

---

### Task 8: Make Actions, Shadow, GUI, and safety explicit dependencies

**Files:**
- Modify: `include/aon/auton/actions.hpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: `include/aon/shadow/service.hpp`
- Modify: `src/aon/shadow/service.cpp`
- Create: `include/aon/safety/safety-coordinator.hpp`
- Create: `src/aon/safety/safety-coordinator.cpp`
- Create: `tests/safety-coordinator-test.cpp`
- Modify: GUI constructors and target compositions
- Delete after replacement: global `actions()`, `shadow::service()`, `gui`, and `autonomousReader` accessors

**Interfaces:**
- Consumes: concrete drive, pose, storage, mechanism, controller-cancellation, and competition-state adapters.
- Produces: explicitly constructed `Actions`, Shadow module, GUI, and `SafetyCoordinator`.

- [ ] **Step 1: Write fail-closed coordination tests**

Assert controller-X, disabled, motion failure, and Shadow cancellation each result in one latched motion cancellation, drivetrain stop, and motorized-mechanism stop. Assert later commands remain suppressed until a new autonomous dispatch explicitly rearms motion.

- [ ] **Step 2: Add constructors and remove internal dependency creation**

`Actions` receives its motion backend, drive I/O, pose source, clock, cancellation source, and immutable safety policy. Shadow receives identity, policy, storage, pose source, playback adapter, and mechanism adapter. GUI receives catalogs and state interfaces only.

- [ ] **Step 3: Implement safety as coordination, not duplicate ownership**

`SafetyCoordinator::stopAll()` invokes `Actions::cancelMotion()`, the drive stop adapter, and the target mechanism stop adapter. MotionControl remains the owner of the cancellation latch; mechanisms remain owners of their state.

- [ ] **Step 4: Correct the GUI dependency direction**

Shadow exposes status and commands. GUI reads status and requests safe state transitions. Shadow must not include or call GUI code.

- [ ] **Step 5: Remove singleton accessors one at a time**

After each removal, compile both targets before removing the next. Do not replace them with a `RobotContext` object passed everywhere.

- [ ] **Step 6: Run lifecycle, safety, motion, Shadow, GUI compile, and both target builds**

Expected: observable cancellation and playback behavior is unchanged.

- [ ] **Step 7: Commit**

```powershell
git add -- include/aon/auton src/aon/auton include/aon/shadow src/aon/shadow `
  include/aon/safety src/aon/safety tests/safety-coordinator-test.cpp `
  include/aon/tools/gui src/aon/tools/gui apps
git diff --cached --check
git commit -m "Inject lifecycle safety and Shadow dependencies"
```

---

### Task 9: Pass the physical baseline gate before changing device ownership

**Files:**
- Modify with measured results: `docs/CURRENT_HANDOFF.md`
- Modify with measured results: applicable files under `docs/testing/`

**Interfaces:**
- Consumes: the exact physical-gate instructions already recorded in the handoff.
- Produces: recorded authorization to proceed with hardware ownership consolidation; no code interface.

- [ ] **Step 1: Upload and validate the small production artifact**

Use `bin/small/monolith.bin`. Run the existing LemLib figure-eight/native baseline and Shadow SD recording-only gates with controller X ready, exactly as documented.

- [ ] **Step 2: Record measured results**

Record completion time, final pose, heading error, mechanism behavior, SD persistence, cancellation behavior, and every observed deviation. Do not summarize a gate as merely “passed.”

- [ ] **Step 3: Stop on any failure**

If a gate fails, diagnose it under a separate bug task. Do not begin Task 10, enable fallback, or reinterpret the big tracking mismatch.

- [ ] **Step 4: Commit only gate documentation**

```powershell
git add -- docs/CURRENT_HANDOFF.md docs/testing
git diff --cached --check
git commit -m "Record dual-architecture baseline validation"
```

---

### Task 10: Centralize drive and localization device ownership

**Files:**
- Create: target `hardware.hpp` and `hardware.cpp` implementations owning all PROS devices
- Create: `include/aon/drive/drive-output.hpp`
- Create: `include/aon/localization/pose-source.hpp`
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `src/aon/auton/encoder-motion.cpp`
- Modify: `src/aon/core/hardware.cpp`
- Modify: target compositions
- Create: `tests/architecture/device-ownership-test.ps1`

**Interfaces:**
- Produces: `DriveOutput` with small differential and big H-drive adapters; `PoseSource` backed by the one LemLib localization owner.
- Preserves: relative encoder fallback as a motion adapter, not a `Localizer`.

- [ ] **Step 1: Write the ownership architecture test**

Scan production sources and assert drive motor groups, tracking rotations, and IMU constructors occur only in the two target hardware implementations. Assert `src/aon/lemlib/chassis.cpp` contains no static PROS device constructor.

- [ ] **Step 2: Verify RED**

Expected: current duplicate device construction is reported.

- [ ] **Step 3: Move devices into target hardware owners**

Construct each target's motors, tracking sensors, IMU, controller, intake sensors, and pneumatics once. Preserve member ordering where initialization dependencies exist.

- [ ] **Step 4: Make LemLib consume references**

Replace the lazy device functions in `chassis.cpp` with a `LemLibMotion` constructor receiving references to the target hardware and immutable drive configuration.

- [ ] **Step 5: Keep fallback semantics honest**

Encoder fallback receives the same drive motor references but remains a relative controller. Do not add `EncoderFallbackLocalizer` until continuous dead-reckoned field pose exists and has its own approved design.

- [ ] **Step 6: Quarantine remaining legacy ownership**

Move any still-required native adapters under `include/aon/compat` and `src/aon/compat`. They may borrow devices but may not construct duplicates.

- [ ] **Step 7: Run ownership test, host suite, and both clean embedded builds**

Expected: one constructor per physical device, unchanged map values, and no ownership test findings.

- [ ] **Step 8: Perform a second physical smoke gate and commit**

Run one low-speed drive and one mechanism action on each available robot. Record results before committing.

```powershell
git add -- apps include/aon/drive include/aon/localization include/aon/compat `
  src/aon/lemlib src/aon/auton/encoder-motion.cpp src/aon/core/hardware.cpp `
  src/aon/compat tests/architecture/device-ownership-test.ps1 docs/testing
git diff --cached --check
git commit -m "Centralize robot device ownership"
```

---

### Task 11: Remove the robot macro, global compatibility surface, and false interfaces

**Files:**
- Delete: `include/aon/globals.hpp`
- Delete or reduce to robot-neutral values: `include/aon/constants.hpp`
- Delete: `include/aon/config/robot-config.hpp`
- Delete: `src/aon/config/robot-config.cpp`
- Delete completed adapters under: `include/aon/compat`, `src/aon/compat`
- Modify all remaining shared callers
- Create: `tests/architecture/dependency-rules-test.ps1`

**Interfaces:**
- Removes: `USING_BIG_ROBOT`, `activeRobotConfig()`, `hardware()`, global hardware aliases, and global robot selection.
- Preserves: explicit target `config()`, `competitionApp()`, shared lifecycle, catalogs, and capability interfaces.

- [ ] **Step 1: Write the final dependency-rule test**

Create a PowerShell test that fails if:

```text
include/aon or src/aon contains USING_BIG_ROBOT
include/aon or src/aon includes apps/small-robot or apps/big-robot
shared production code includes aon/globals.hpp
both app main.cpp files enter one link
big catalog contains small-only IDs
small catalog contains big-only IDs
```

- [ ] **Step 2: Verify RED, then migrate each remaining caller explicitly**

Replace global references with constructor-held references owned by the nearest deep module. Do not introduce `RobotContext`, `Services`, or `RobotComposition` as a replacement grab bag.

- [ ] **Step 3: Delete transitional selection code**

Remove `AON_TARGET_SMALL`, `AON_TARGET_BIG`, and the `USING_BIG_ROBOT` compatibility mapping from shared compilation. Target selection now happens solely through target source lists.

- [ ] **Step 4: Delete obsolete compatibility modules**

Run the deletion test mentally and through callers: if deleting an adapter merely removes a pass-through, delete it. Retain legacy behavior only when a physical fallback is still actively required and documented.

- [ ] **Step 5: Run final dependency tests and both builds**

```powershell
& tests\architecture\dependency-rules-test.ps1
rg -n 'USING_BIG_ROBOT|aon/globals.hpp|activeRobotConfig\(' include\aon src\aon apps
make clean-small small
make clean-big big
```

Expected: the search has no production matches; both images build.

- [ ] **Step 6: Commit**

```powershell
git add -A -- include/aon src/aon apps tests/architecture Makefile mk
git diff --cached --check
git commit -m "Remove global robot selection and compatibility access"
```

---

### Task 12: Build the robot-neutral sources as one shared static library

**Files:**
- Create: `mk/aon-shared-library.mk`
- Create: `mk/robot-executable.mk`
- Modify: `Makefile`
- Modify: `common.mk`
- Modify: `tools/build-all.ps1`
- Create: `tests/architecture/link-contents-test.ps1`

**Interfaces:**
- Produces: `bin/shared/libaon.a`, `bin/small/aon-small.bin`, and `bin/big/aon-big.bin`.
- Consumes: only robot-neutral `src/aon` objects in the archive and one app source tree per executable.

- [ ] **Step 1: Write the failing archive-content test**

Use `arm-none-eabi-ar t bin/shared/libaon.a` and fail if any member path contains `small-robot`, `big-robot`, or a callback `main` object. Use `arm-none-eabi-nm` on each executable and assert exactly one definition of every PROS callback.

- [ ] **Step 2: Verify RED**

Expected: `bin/shared/libaon.a` does not exist.

- [ ] **Step 3: Compile `src/aon` into the archive**

`mk/aon-shared-library.mk` builds shared objects under `bin/shared/obj` with no robot target definition and archives them as:

```make
$(AR) rcs bin/shared/libaon.a $(AON_SHARED_OBJECTS)
```

Do not put app configurations, hardware owners, compositions, or callback wrappers in this archive.

- [ ] **Step 4: Link each app against the same archive**

`mk/robot-executable.mk` compiles exactly one app tree, then links those objects with `bin/shared/libaon.a` and the existing PROS/LemLib libraries. Emit named ELF and binary artifacts under the target directory.

- [ ] **Step 5: Build from a completely clean state**

```powershell
make clean-robots
make robots
& tests\architecture\link-contents-test.ps1
```

Expected: the shared archive is built once, both app images link, and archive/link content checks pass.

- [ ] **Step 6: Commit**

```powershell
git add -- Makefile common.mk mk tools/build-all.ps1 `
  tests/architecture/link-contents-test.ps1
git diff --cached --check
git commit -m "Link both robot apps against shared library"
```

---

### Task 13: Consolidate tests around public module interfaces

**Files:**
- Move focused tests into: `tests/shared`, `tests/small-robot`, `tests/big-robot`
- Create: `tools/test-all.ps1`
- Modify or delete implementation-coupled tests after equivalent interface coverage exists
- Modify: `tools/build-all.ps1`

**Interfaces:**
- Consumes: final lifecycle, catalog, motion, mechanism, Shadow, safety, and configuration interfaces.
- Produces: one deterministic host-test entry point.

- [ ] **Step 1: Classify existing tests by public interface**

Preserve all current behavioral assertions for vector math, paths, motion health/fallback geometry, Shadow recorder/processor/codec/storage/player/state, SD behavior, and hardware maps.

- [ ] **Step 2: Add composition tests**

Small tests assert small identity, device map, catalog, policies, and capabilities. Big tests assert the corresponding big values and absence of small-only capabilities.

- [ ] **Step 3: Replace source-text assertions where a real interface now exists**

For catalog registration and configuration, call the actual target `catalog()` and `config()` functions. Retain architecture text scans only for dependency direction, forbidden macros, and link membership.

- [ ] **Step 4: Implement `tools/test-all.ps1`**

Compile every host test with `-std=c++17 -Wall -Wextra -Werror`, run each executable, run architecture PowerShell tests, and stop on first failure.

- [ ] **Step 5: Run the complete verification matrix**

```powershell
& tools\test-all.ps1
make clean-robots
make robots
git diff --check
```

Expected: all host and architecture tests pass and both production artifacts build.

- [ ] **Step 6: Commit**

```powershell
git add -A -- tests tools/test-all.ps1 tools/build-all.ps1
git diff --cached --check
git commit -m "Consolidate dual-robot architecture tests"
```

---

### Task 14: Document operation, ownership, and future deletion gates

**Files:**
- Modify: `README.md`
- Modify: `docs/CURRENT_HANDOFF.md`
- Create: `docs/architecture/dual-robot-architecture.md`
- Create: `docs/architecture/dependency-rules.md`
- Modify: `docs/TWO_COMPUTER_WORKFLOW.md`

**Interfaces:**
- Documents the final build and module interfaces; produces no code interface.

- [ ] **Step 1: Document the two production artifacts**

State which command builds each image, where it is emitted, how the boot screen identifies it, and that source editing is never used to choose a robot.

- [ ] **Step 2: Document device ownership and dependency direction**

Include the final graph:

```text
small app ─┐
           ├──> libaon.a ──> PROS / LemLib
big app ───┘

libaon.a never depends on either app.
```

List every module owner, stable interface, production adapter, and test adapter.

- [ ] **Step 3: Document remaining compatibility code**

For every retained file under `compat`, name the physical gate that permits deletion. If no gate or active caller exists, remove the file before completing this task.

- [ ] **Step 4: Run documentation and final repository checks**

```powershell
rg -n 'pros build|USING_BIG_ROBOT|bin/monolith.bin' README.md docs `
  -g '!docs/superpowers/plans/**' -g '!docs/superpowers/specs/**'
git diff --check
git status --short --branch
```

Expected: current operational docs describe named targets, and historical plans/specs remain unchanged as records.

- [ ] **Step 5: Commit**

```powershell
git add -- README.md docs/architecture docs/CURRENT_HANDOFF.md `
  docs/TWO_COMPUTER_WORKFLOW.md
git diff --cached --check
git commit -m "Document dual-robot production architecture"
```

---

## Final Acceptance Checklist

- [ ] `tools/test-all.ps1` passes with warnings treated as errors.
- [ ] `make clean-robots robots` produces `libaon.a`, `aon-small.bin`, and `aon-big.bin`.
- [ ] Shared sources contain no robot-selection macro or app include.
- [ ] Each executable defines exactly one PROS callback set.
- [ ] Each physical device has exactly one owning construction site.
- [ ] The big tracking mismatch remains explicitly reported until a separate calibrated fix is approved.
- [ ] Small and big autonomous catalogs expose only valid target routines.
- [ ] GUI depends on catalog/status interfaces; Shadow does not depend on GUI.
- [ ] Encoder fallback is still represented as relative motion unless continuous pose has been separately implemented.
- [ ] Global hardware aliases, `activeRobotConfig()`, and runtime robot factory patterns are absent.
- [ ] Every retained compatibility adapter names a deletion gate.
- [ ] Physical gate results are recorded rather than inferred from compilation.

## Program Checkpoints

Do not execute all fourteen tasks as one branch or review. Recommended review boundaries are:

1. **Foundation:** Tasks 1–4 — hardware characterization, dual targets, identity/configuration, lifecycle compositions.
2. **Behavior separation:** Tasks 5–8 — catalogs, operator control, mechanisms, explicit dependencies and safety.
3. **Physical gate:** Task 9 — required measured validation.
4. **Ownership end state:** Tasks 10–12 — single hardware ownership, removal of global selection, shared archive.
5. **Hardening:** Tasks 13–14 — interface-level tests and operational documentation.

Each checkpoint must be independently buildable, reviewable, and recoverable. A failure in a later checkpoint must not require reverting earlier characterized architecture work.
