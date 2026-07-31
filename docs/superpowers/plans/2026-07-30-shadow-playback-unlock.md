# Shadow Playback Unlock Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Safely unlock one-shot, small-robot Shadow playback using closed-loop LemLib paths and every validated recorded mechanism event.

**Architecture:** A host-testable scheduler owns validation, segment order, event timing, and fail-closed semantics. A V5-only adapter turns one processed segment at a time into a bounded in-memory LemLib asset, reports monotonic path progress, and delegates semantic mechanisms to the existing adapter. The Shadow service owns the one-shot arm and decoded snapshot; the GUI confirms the start pose and selects a dedicated Skills AUT3 playback routine.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, V5 Brain screen API, fixed-capacity Shadow codec/storage, MinGW g++ host tests, GNU Make/ARM toolchain, Git.

## Global Constraints

- Playback is authorized only on the small differential robot; big/H-drive always returns `UnsupportedRobot`.
- Playback authorization starts false and is changed to true for the small robot only after all automated checks pass.
- Use a 6-inch LemLib lookahead and timeout `max(recordedDuration * 2 + 1000 ms, 2000 ms)`.
- Do not authorize motor-encoder fallback.
- Controller X, competition disable, invalid odometry, timeout, storage failure, motion failure, mechanism failure, or cancellation stops drivetrain and mechanisms and suppresses all later work.
- The arm is one-shot and slot-specific; recording, deletion, cancellation, slot change, or five-second confirmation expiry clears it.
- Keep the decoded recording and runtime path buffer alive for the entire synchronous playback.
- Keep `USING_BIG_ROBOT false` in the committed upload configuration.
- Do not promote playback beyond the progressive physical rollout in this plan.

## File Map

- Create `include/aon/shadow/player.hpp`: platform-independent playback policy, callbacks, scheduler, timeout, and monotonic projection interfaces.
- Create `src/aon/shadow/player.cpp`: pure validation, segment scheduling, event dispatch, timeout calculation, and progress projection.
- Create `include/aon/shadow/player-pros.hpp`: V5 adapter entry point and cancellation/stop boundary.
- Create `src/aon/shadow/player-pros.cpp`: bounded JerryIO serialization, LemLib execution, pose projection, dwell polling, and mechanism calls.
- Modify `include/aon/shadow/mechanisms.hpp` and `src/aon/shadow/mechanisms.cpp`: explicit motor-output stop used by every playback exit.
- Modify `include/aon/auton/actions.hpp` and `src/aon/auton/actions.cpp`: optional monitored-loop poll callback.
- Modify `include/aon/shadow/service-state.hpp` and `src/aon/shadow/service-state.cpp`: explicit `Armed` state and playback completion.
- Modify `include/aon/shadow/service.hpp` and `src/aon/shadow/service.cpp`: policy checks, snapshot loading, one-shot consume, and playback result publication.
- Modify `include/aon/config/robot-config.hpp` and `src/aon/config/robot-config.cpp`: robot-specific playback authorization.
- Modify `include/aon/tools/gui/gui.hpp` and `src/aon/tools/gui/gui.cpp`: two-press confirmation, start-pose display, arm status, and Skills AUT3 selection.
- Modify `include/aon/auton/routines.hpp`, `src/aon/auton/routine-selectors.cpp`, and `src/aon/core/robot.cpp`: registered routine and disable cancellation.
- Modify `tests/shadow-auton-test.cpp`: scheduler, state, runtime-path, and policy coverage.

---

### Task 1: Pure Playback Scheduler

**Files:**
- Create: `include/aon/shadow/player.hpp`
- Create: `src/aon/shadow/player.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: `DecodedRecording`, `ProcessedRoute`, `RouteSegment`, `AnchoredEvent`, `MechanismEvent`, `RobotIdentity`, and `ResultCode` from the existing Shadow modules.
- Produces:

```cpp
namespace aon::shadow {

struct PlaybackPolicy {
  bool authorized = false;
  bool armed = false;
  RobotIdentity activeRobot = RobotIdentity::Small;
};

using MotionProgress = std::function<ResultCode(float progress)>;
using DwellProgress = std::function<ResultCode(std::uint32_t elapsedMs)>;

struct PlaybackCallbacks {
  std::function<ResultCode(const ProcessedRoute&, const RouteSegment&,
                           const MotionProgress&)> follow;
  std::function<ResultCode(std::uint32_t durationMs,
                           const DwellProgress&)> dwell;
  std::function<ResultCode(const MechanismEvent&)> mechanism;
  std::function<bool()> cancelled;
  std::function<void()> stopAll;
};

ResultCode validatePlayback(const DecodedRecording& recording,
                            const PlaybackPolicy& policy);
ResultCode playRecording(const DecodedRecording& recording,
                         const PlaybackPolicy& policy,
                         PlaybackCallbacks& callbacks);
int playbackTimeoutMs(std::uint32_t durationMs);
float monotonicPolylineProgress(const PathPoint* points, std::size_t count,
                                float x, float y, float previous);

}  // namespace aon::shadow
```

- `playRecording` must call `stopAll` exactly once on every return path, including successful completion.

- [ ] **Step 1: Add failing scheduler-policy tests**

Add table-driven cases to `tests/shadow-auton-test.cpp` that construct a valid two-segment `DecodedRecording` and count callback invocations:

```cpp
struct PlaybackSpy {
  int follows = 0;
  int dwells = 0;
  int mechanisms = 0;
  int stops = 0;
  std::vector<int> eventValues;
};

for (const auto policy : {
         PlaybackPolicy{false, true, RobotIdentity::Small},
         PlaybackPolicy{true, false, RobotIdentity::Small},
         PlaybackPolicy{true, true, RobotIdentity::Big},
     }) {
  PlaybackSpy spy;
  auto callbacks = makePlaybackSpyCallbacks(spy);
  const auto result = playRecording(validSmallRecording(), policy, callbacks);
  expect(result != ResultCode::Ok, "unsafe policy must be rejected");
  expect(spy.follows == 0 && spy.dwells == 0 && spy.mechanisms == 0,
         "policy rejection must touch no hardware");
  expect(spy.stops == 1, "policy rejection must stop outputs once");
}
```

Also corrupt one route point, one segment range, one direction, one anchored-event index, and one mechanism value. Each case must return `CorruptFile` or the existing precise validation error with zero motion/mechanism calls.

- [ ] **Step 2: Compile to prove the player boundary is missing**

Run:

```powershell
& 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe' `
  -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp `
  src\aon\shadow\processor.cpp src\aon\shadow\codec.cpp `
  src\aon\shadow\storage.cpp src\aon\shadow\service-state.cpp `
  src\aon\shadow\player.cpp -o bin\host-tests\shadow-auton-test.exe
```

Expected: FAIL because `aon/shadow/player.hpp` and its functions do not exist.

- [ ] **Step 3: Implement validation and timeout calculation**

Create `player.hpp` with the exact interfaces above. In `player.cpp`, make `playbackTimeoutMs` overflow-safe:

```cpp
int playbackTimeoutMs(std::uint32_t durationMs) {
  constexpr std::uint64_t kMinimum = 2000;
  constexpr std::uint64_t kMaximum = static_cast<std::uint64_t>(INT_MAX);
  const auto scaled = static_cast<std::uint64_t>(durationMs) * 2U + 1000U;
  return static_cast<int>(std::min(kMaximum, std::max(kMinimum, scaled)));
}
```

`validatePlayback` must check, in this order:

1. `authorized`, then `armed`;
2. active robot is `Small`;
3. recording robot equals active robot;
4. route counts are within fixed capacities and contain at least one segment;
5. each segment has valid type/direction, positive duration, and an in-range point or dwell range;
6. motion segments contain at least two finite points and no zero-length consecutive pair;
7. anchored events reference valid segments, are ordered, and have progress in `[0,1]` or dwell offsets within duration;
8. every mechanism kind/value passes `validateMechanism`.

Return `PlayLocked` for false authorization or false arming, `UnsupportedRobot` for an active big robot, `WrongRobot` for a recording/active-robot mismatch, and `CorruptFile` for every structural or value validation failure.

- [ ] **Step 4: Add failing ordering, timing, cancellation, and failure tests**

Use callback lambdas that synchronously report progress `{0.0F, 0.2F, 0.6F, 1.0F}` and dwell elapsed times `{0, 20, 40, duration}`. Assert:

- forward, dwell, and reverse callbacks occur in route order;
- multiple events at the same anchor retain recorded order;
- events fire exactly once when progress/time crosses the anchor;
- a motion failure suppresses later segments/events;
- a mechanism failure suppresses later events and segments;
- cancellation during motion and during dwell returns `Cancelled`;
- `stopAll` runs exactly once for success and every failure.

Use an event log such as:

```cpp
expect(log == std::vector<std::string>({
  "follow-forward", "event-1", "event-2", "dwell", "event-3",
  "follow-reverse", "stop"
}), "scheduler order");
```

- [ ] **Step 5: Implement the scheduler**

Implement `playRecording` with one local `finish` lambda:

```cpp
const auto finish = [&](ResultCode result) {
  callbacks.stopAll();
  return result;
};
```

Validate all input before invoking `follow`, `dwell`, or `mechanism`. For each segment, hold an index to its next anchored event. The motion observer clamps reported progress to `[0,1]`, rejects non-finite input, and dispatches all newly reached events. The dwell observer dispatches all events whose offset is `<= elapsedMs`. Check `cancelled()` before each segment, inside both observers, and after each segment. Propagate the first non-`Ok` result without executing later work.

- [ ] **Step 6: Implement and test monotonic projection**

For each polyline edge, project `(x,y)` onto the segment, clamp the local parameter to `[0,1]`, and convert projected arc length to total-path progress. Choose the nearest projected point and return `max(previous, candidate)`. Return `previous` for null pointers, fewer than two points, non-finite poses, or zero total length.

Add tests for:

- start, midpoint, and endpoint;
- an off-path pose;
- a pose that moves backward after reaching 0.7;
- a zero-length/invalid path;
- self-crossing geometry using the previous value to prevent regression.

- [ ] **Step 7: Run the complete host suite**

Run the compile command from Step 2 and:

```powershell
& '.\bin\host-tests\shadow-auton-test.exe'
```

Expected: `shadow auton tests passed`.

- [ ] **Step 8: Commit the scheduler**

```powershell
git add -- include/aon/shadow/player.hpp src/aon/shadow/player.cpp tests/shadow-auton-test.cpp
git commit -m "Add fail-closed Shadow playback scheduler"
```

---

### Task 2: Bounded LemLib Runtime Adapter

**Files:**
- Create: `include/aon/shadow/player-pros.hpp`
- Create: `src/aon/shadow/player-pros.cpp`
- Modify: `include/aon/auton/actions.hpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: `include/aon/shadow/mechanisms.hpp`
- Modify: `src/aon/shadow/mechanisms.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: Task 1 `PlaybackCallbacks`, `playRecording`, `playbackTimeoutMs`, and `monotonicPolylineProgress`; existing `actions()`, `applyMechanism`, and `stopAllMechanisms`.
- Produces:

```cpp
namespace aon::shadow {

constexpr std::size_t kRuntimePathCapacity = 64U * 1024U;

struct RuntimePath {
  std::array<std::uint8_t, kRuntimePathCapacity> bytes{};
  std::size_t used = 0;
};

ResultCode serializeRuntimePath(const ProcessedRoute& route,
                                const RouteSegment& segment,
                                RuntimePath& output);
ResultCode playOnRobot(const DecodedRecording& recording,
                       const PlaybackPolicy& policy);
void cancelRobotPlayback();
void stopAllMechanisms();

}  // namespace aon::shadow
```

- Extends the existing method without breaking callers:

```cpp
MotionResult Actions::followPath(
    const char* name, const asset& path, float lookahead, int timeout,
    bool forwards = true, const std::function<void()>& onPoll = {});
```

- [ ] **Step 1: Add failing bounded-serialization tests**

Add tests that pass a valid motion segment to `serializeRuntimePath`, reinterpret `bytes.data()` as text, and assert:

- each line is `x, y, speed`;
- all points except the final point have speed in `[1,100]`;
- the final point is forced to speed `0`;
- JerryIO `endData` metadata is present;
- `used <= kRuntimePathCapacity`;
- invalid count/range/NaN/Inf/zero-length geometry returns `CorruptFile`;
- deliberately oversized formatted input returns `CapacityReached` and leaves `used == 0`.

- [ ] **Step 2: Compile to verify the PROS adapter is absent**

Run the Task 1 host command with `src\aon\shadow\player-pros.cpp` added.

Expected: FAIL because `player-pros.hpp` and `serializeRuntimePath` do not exist. The host test must compile only the pure serialization portion; guard PROS includes and `playOnRobot` implementation with the same platform convention already used by storage.

- [ ] **Step 3: Implement bounded asset serialization**

Use `std::snprintf` into the remaining buffer and reject negative or truncated results. Copy only the segment's `[firstPoint, firstPoint + pointCount)` range. Preserve validated inferred speeds except force the last speed to zero. Append:

```text
endData
#PATH.JERRYIO-DATA {"appVersion":"0.11.0","format":"LemLib v0.5","name":"Shadow Runtime"}
```

Do not allocate on the V5 task stack: the `RuntimePath` instance must have static/process lifetime in `player-pros.cpp`.

- [ ] **Step 4: Add the monitored poll callback**

Change the internal `runMonitored` helper in `actions.cpp` to accept `const std::function<void()>& onPoll`. Invoke it once inside every monitoring-loop iteration after cancellation/health checks and before `pros::delay`. Pass `{}` from point/pose/turn calls and forward the optional callback from `Actions::followPath`.

No callback may bypass `MotionResult` failure handling or release the drivetrain lease early.

- [ ] **Step 5: Implement the V5 callbacks**

In `playOnRobot`:

1. Reject `USING_BIG_ROBOT` with `UnsupportedRobot` before serialization or hardware calls.
2. Store the active segment pointer and previous progress in process-lifetime state.
3. Serialize the segment into the static `RuntimePath`.
4. Construct `asset runtimeAsset{runtimePath.bytes.data(), runtimePath.used}`.
5. Call `actions().followPath("Shadow segment", runtimeAsset, 6.0F, playbackTimeoutMs(segment.durationMs), segment.direction == Direction::Forward, poll)`.
6. In `poll`, read `chassis.getPose()`, call `monotonicPolylineProgress`, and send the value to the scheduler observer. Latch the observer's first failure, call `actions().cancelMotion()`, and return that result after `followPath`.
7. Poll dwell segments every 20 ms, checking `pros::competition::is_disabled()`, the controller-X cancellation latch, and the scheduler observer.
8. Route mechanisms through `applyMechanism`.
9. Make `stopAll` call `actions().cancelMotion()`, `actions().stop()`, and `stopAllMechanisms()`.

`cancelRobotPlayback` sets a process-lifetime atomic cancellation flag, calls `actions().cancelMotion()`, and stops every mechanism.

Add `stopAllMechanisms()` beside `applyMechanism` in the existing Shadow mechanism adapter. On the small robot it must call `intake.stopScan()` and `intake.stop()` so corridor/elevator/intake motor commands are zero. On the big robot it must call `intake.stopReleasingAndWait()`, `intake.stopScan()`, and `intake.stop()`. Piston target states remain unchanged because they are passive state, not running motor output.

- [ ] **Step 6: Run host regressions and clean robot builds**

Run:

```powershell
& 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe' `
  -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp `
  src\aon\shadow\processor.cpp src\aon\shadow\codec.cpp `
  src\aon\shadow\storage.cpp src\aon\shadow\service-state.cpp `
  src\aon\shadow\player.cpp src\aon\shadow\player-pros.cpp `
  -o bin\host-tests\shadow-auton-test.exe
& '.\bin\host-tests\shadow-auton-test.exe'
make clean
make
```

Expected: host suite passes and the small-robot build completes. Temporarily set `USING_BIG_ROBOT true`, run `make clean; make`, restore `false`, then clean-build again. Accept only the existing vendored `json.hpp` warning.

- [ ] **Step 7: Commit the runtime adapter**

```powershell
git add -- include/aon/shadow/player-pros.hpp src/aon/shadow/player-pros.cpp `
  include/aon/auton/actions.hpp src/aon/auton/actions.cpp `
  include/aon/shadow/mechanisms.hpp src/aon/shadow/mechanisms.cpp `
  tests/shadow-auton-test.cpp
git commit -m "Run Shadow segments through monitored LemLib paths"
```

---

### Task 3: One-Shot Service State and Robot Policy

**Files:**
- Modify: `include/aon/shadow/service-state.hpp`
- Modify: `src/aon/shadow/service-state.cpp`
- Modify: `include/aon/shadow/service.hpp`
- Modify: `src/aon/shadow/service.cpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: Task 2 `playOnRobot` and `cancelRobotPlayback`; existing `Storage::load`, slot summaries, and `ServiceStateMachine`.
- Produces:

```cpp
enum class ServiceMode : std::uint8_t {
  Idle, Recording, Processing, Saved, Invalid, Armed, Playing, Finished,
  Cancelled
};

ResultCode ServiceStateMachine::armPlay(std::uint8_t slot,
                                        std::uint32_t now = 0);
bool ServiceStateMachine::consumeArm(std::uint8_t slot,
                                     std::uint32_t now = 0);
ResultCode ServiceStateMachine::finishPlayback(ResultCode result,
                                               std::uint32_t now = 0);

ResultCode Service::armPlayback(std::uint8_t slot, bool startConfirmed,
                                bool robotDisabled);
ResultCode Service::runArmedPlayback();
void Service::clearPlaybackArm();

using PlaybackRunner = std::function<ResultCode(
    const DecodedRecording&, const PlaybackPolicy&)>;

ResultCode dispatchArmedPlayback(ServiceStateMachine& state, Storage& storage,
                                std::uint8_t slot, RobotIdentity robot,
                                DecodedRecording& snapshot,
                                const PlaybackRunner& runner,
                                std::uint32_t now = 0);

ResultCode authorizePlaybackArm(bool authorized, RobotIdentity activeRobot,
                                bool robotDisabled,
                                const SlotSummary& summary);

struct RobotConfig {
  RobotIdentity identity;
  LemLibDriveConfig lemlib;
  bool shadowPlaybackAuthorized;
};
```

- [ ] **Step 1: Add failing one-shot state tests**

Extend the state-machine tests to assert:

```cpp
expect(state.armPlay(2, 100) == ResultCode::Ok, "arm slot 2");
expect(state.status().mode == ServiceMode::Armed, "state is visibly armed");
expect(!state.consumeArm(1, 101), "different slot cannot consume");
expect(state.consumeArm(2, 102), "selected slot consumes once");
expect(state.status().mode == ServiceMode::Playing, "consume starts playback");
expect(!state.consumeArm(2, 103), "arm is one-shot");
expect(state.finishPlayback(ResultCode::Ok, 200) == ResultCode::Ok,
       "playback finishes");
expect(state.status().mode == ServiceMode::Finished, "finished is published");
```

Test that record start, delete start, cancel, and a newly selected slot clear an existing arm. Test that failure completion publishes the exact result and `Cancelled` uses the cancelled mode.

- [ ] **Step 2: Run the host suite and observe the expected failure**

Run the Task 1 host compile/run command.

Expected: compilation fails because `Armed`, `Finished`, timestamped `consumeArm`, and `finishPlayback` do not exist.

- [ ] **Step 3: Implement explicit arm transitions**

Make `armPlay` enter `Armed`, record the slot, and increment revision. `consumeArm` succeeds only in `Armed` for the exact slot, clears `armedSlot_`, changes the mode to `Playing`, and updates `changedAt`. `finishPlayback` is valid only from `Playing`; publish `Finished/Ok`, `Cancelled/Cancelled`, or `Invalid/<exact error>`.

Ensure `beginRecord`, erase preparation, and `cancel` clear `armedSlot_`.

- [ ] **Step 4: Add failing service-policy and dispatch tests**

Use the exact `authorizePlaybackArm` and `dispatchArmedPlayback` interfaces above. Back `Storage` with the existing in-memory `MemoryFileStore` used by storage tests and pass a runner lambda that records calls. Assert:

- authorization false returns `PlayLocked`;
- big robot returns `UnsupportedRobot`;
- enabled robot state returns `UnsafeState`;
- invalid or wrong-robot slot does not arm;
- confirmed valid small slot arms;
- `runArmedPlayback` consumes before load, so a failed load cannot be retried without re-arming;
- second dispatch returns `PlayLocked`;
- exact player result appears in `Status`.

- [ ] **Step 5: Add the robot configuration flag**

Add `shadowPlaybackAuthorized` to `RobotConfig`. Initialize it to `false` in both small and big configurations during this task. Do not use a global macro or infer authorization from `USING_BIG_ROBOT`.

- [ ] **Step 6: Implement service arming and dispatch**

`authorizePlaybackArm` performs the first five checks below and returns the precise result without changing state. `armPlayback` must:

1. read `activeRobotConfig()`;
2. reject big before reading the SD;
3. reject false authorization;
4. fetch and validate the selected `SlotSummary`;
5. call `authorizePlay(startConfirmed, robotDisabled, slotValid)`;
6. arm exactly that slot without loading or commanding hardware.

`dispatchArmedPlayback` must consume the supplied slot first. Only after successful consumption, load through `storage.load(slot, robot, snapshot)`, validate the decoded robot/route, invoke `runner`, and publish `finishPlayback`, including load failure. `Service::runArmedPlayback` captures the armed slot from status and delegates to this helper with its process-lifetime `playbackSnapshot` and a runner calling `playOnRobot`. `clearPlaybackArm` cancels state and calls `cancelRobotPlayback`.

- [ ] **Step 7: Run host tests and both robot builds**

Run the Shadow host suite, small clean build, big clean build, restore `USING_BIG_ROBOT false`, and clean-build small again.

Expected: all builds pass; playback remains policy-locked because both configuration values are still false.

- [ ] **Step 8: Commit the service boundary**

```powershell
git add -- include/aon/shadow/service-state.hpp src/aon/shadow/service-state.cpp `
  include/aon/shadow/service.hpp src/aon/shadow/service.cpp `
  include/aon/config/robot-config.hpp src/aon/config/robot-config.cpp `
  tests/shadow-auton-test.cpp
git commit -m "Add one-shot Shadow playback service"
```

---

### Task 4: Brain Confirmation Flow and Registered Autonomous

**Files:**
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `src/aon/core/robot.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: Task 3 `Service::armPlayback`, `Service::runArmedPlayback`, and `Service::clearPlaybackArm`.
- Produces:

```cpp
int aon::routines::RunShadowPlayback();
```

- Skills AUT3 is renamed to `SHADOW PLAYBACK`; Red AUT3 remains available for the hybrid autonomous plan.

- [ ] **Step 1: Add the confirmation state**

Add `Play` to `Gui::ShadowConfirmation`. Add:

```cpp
static constexpr std::uint32_t kShadowPlayConfirmationMs = 5000;
```

When the selected slot changes, the menu exits, recording/deletion begins, or the expiry time passes, set confirmation to `None` and call `service().clearPlaybackArm()` only when an arm exists.

- [ ] **Step 2: Render the exact play states and start pose**

For a valid selected slot, render duration and the stored start `x`, `y`, and heading. The button/state text must be:

- `PLAY LOCKED` when policy, robot, slot, or disabled-state checks fail;
- `PLAY` when eligible;
- `CONFIRM PLAY` during the five-second window;
- `ARMED` after successful arming;
- `PLAYING`, `FINISHED`, `CANCELLED`, or the exact `ResultCode` afterward.

Do not use file validity as a synonym for arming.

- [ ] **Step 3: Implement two-press arming**

On first eligible PLAY press, set `ShadowConfirmation::Play` and expiry `pros::millis() + 5000`. On the second press before expiry:

```cpp
const auto result = shadow::service().armPlayback(
    selectedShadowSlot, true, pros::competition::is_disabled());
```

On `Ok`, call `selectAutonByList(Alliance::Skills, 3)` and show `ARMED`. On error, display the exact result and clear confirmation. Never call the player directly from the GUI task.

- [ ] **Step 4: Register the synchronous autonomous**

Declare `RunShadowPlayback` in `routines.hpp`. Implement:

```cpp
int RunShadowPlayback() {
  return aon::shadow::service().runArmedPlayback() ==
                 aon::shadow::ResultCode::Ok
             ? 1
             : 0;
}
```

Change `SkillsRoutine3` to call the normal fail-closed `runRoutine("SHADOW PLAYBACK", RunShadowPlayback)` boundary. Change the third skills GUI label to `SHADOW PLAYBACK`. Do not change Red AUT3 or the Kevin routines in this task.

- [ ] **Step 5: Wire disable/cancel cleanup**

In `Robot::disabled()`, call `shadow::service().clearPlaybackArm()`, then explicitly stop the drivetrain and mechanisms through the existing safety boundary. Confirm this does not delete the saved slot.

Controller X remains latched through `Actions`; `cancelRobotPlayback` converts it into scheduler cancellation and suppresses later events.

- [ ] **Step 6: Add selector and state regression assertions**

In the dependency-free host test, include `routines.hpp` and add:

```cpp
static_assert(std::is_same_v<decltype(&aon::routines::RunShadowPlayback),
                             int (*)()>);
```

Add pure state assertions for confirmation expiry, slot change, and cancellation clearing an arm. Robot GUI rendering itself is verified by the clean ARM build and the physical checklist.

- [ ] **Step 7: Run all host tests and both robot builds**

Run Shadow, motion-fallback, SD integration, and figure-eight host suites using their existing plan commands. Then clean-build small, clean-build big, restore `USING_BIG_ROBOT false`, and clean-build small.

Expected: all suites/builds pass; Skills AUT3 compiles as Shadow playback; configuration still keeps it locked.

- [ ] **Step 8: Commit GUI and routine registration**

```powershell
git add -- include/aon/tools/gui/gui.hpp src/aon/tools/gui/gui.cpp `
  include/aon/auton/routines.hpp src/aon/auton/routine-selectors.cpp `
  src/aon/core/robot.cpp tests/shadow-auton-test.cpp
git commit -m "Add confirmed Shadow playback arming flow"
```

---

### Task 5: Verify, Authorize Small Robot, and Perform the Physical Gate

**Files:**
- Modify: `src/aon/config/robot-config.cpp`
- Create: `docs/testing/2026-07-30-shadow-playback-checklist.md`

**Interfaces:**
- Consumes: Tasks 1-4 complete and green.
- Produces: small configuration with `shadowPlaybackAuthorized = true`, big configuration false, and a repeatable physical test record.

- [ ] **Step 1: Run the complete automated gate while still locked**

Run every executable under `bin\host-tests` that belongs to motion, Shadow, SD, and path validation. Run `git diff --check`.

Expected: every suite reports passed; no whitespace errors.

- [ ] **Step 2: Prove big playback remains rejected**

Set only `USING_BIG_ROBOT true`, run `make clean; make`, and confirm the big `RobotConfig` still has `shadowPlaybackAuthorized = false`. Restore `USING_BIG_ROBOT false`.

- [ ] **Step 3: Authorize only the small configuration**

Append the authorization field to the positional small `RobotConfig` initializer:

```cpp
      true,  // shadowPlaybackAuthorized
```

Keep the appended big initializer field:

```cpp
      false,  // shadowPlaybackAuthorized
```

Clean-build small again.

- [ ] **Step 4: Create the physical checklist**

Record these exact gates in `docs/testing/2026-07-30-shadow-playback-checklist.md`:

1. clear field, robot on small configuration, controller X ready;
2. record/save a short low-speed route without mechanisms;
3. verify displayed start pose, confirm PLAY twice, and verify Skills AUT3 is selected;
4. place robot at start and run autonomous;
5. verify direction, endpoint, cancellation, and stopped outputs;
6. repeat with one intake state and one piston transition;
7. verify event order and final explicit states;
8. record pass/fail, slot, route duration, endpoint error, and observed fault text.

State that unexpected movement, event ordering, odometry, endpoint, or cancellation behavior requires immediately setting small authorization back to false before any longer test.

- [ ] **Step 5: Final static verification**

Run:

```powershell
rg -n "shadowPlaybackAuthorized" include src
rg -n "SHADOW PLAYBACK|RunShadowPlayback" include src
git diff --check
git status --short
```

Expected: exactly one enabled authorization in the small config, the big config false, Skills AUT3 registered, and only intended changes.

- [ ] **Step 6: Commit the controlled unlock**

```powershell
git add -- src/aon/config/robot-config.cpp `
  docs/testing/2026-07-30-shadow-playback-checklist.md
git commit -m "Authorize verified small-robot Shadow playback"
```

- [ ] **Step 7: Upload and execute only the progressive physical gate**

Upload the small build. Do the drivetrain-only test first, then the one-intake/one-piston test. Do not replay the long recording until both pass. Enter results in the checklist and commit those measured results separately.
