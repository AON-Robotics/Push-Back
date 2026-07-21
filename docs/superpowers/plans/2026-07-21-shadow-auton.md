# Shadow Auton Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three SD-backed driver-route recording slots and, after explicit physical validation, closed-loop autonomous playback with synchronized mechanism events and latched emergency cancellation.

**Architecture:** Keep capture, processing, serialization, SD access, mechanism adaptation, playback, and GUI as separate modules. Capture fixed-capacity 20 ms pose/input samples plus semantic mechanism events; process them into forward/reverse LemLib path assets and dwell events; use dual-generation files per slot so an interrupted save cannot destroy the last valid recording. Playback uses the existing autonomous action/cancellation boundary and remains unauthorized until recording-only and odometry physical gates pass.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, V5 Brain screen API, C stdio on `/usd`, GNU Make/ARM toolchain, MinGW g++ host tests, Git.

## Global Constraints

- Implement the approved design in `docs/superpowers/specs/2026-07-21-shadow-auton-design.md`.
- Keep `USING_BIG_ROBOT false` in the committed upload configuration.
- Sample at exactly 20 ms, with at most 3,000 samples and 512 mechanism events per recording.
- Support exactly three SD slots, each with `a` and `b` generation files.
- Record for at most 60 seconds and never wrap or overwrite an in-memory capture buffer.
- Store semantic mechanism intent, not raw motor or pneumatic ports.
- Never write C++ structs directly to disk; serialize integers explicitly little-endian and floats as IEEE-754 binary32.
- Keep `Capture`, `ProcessedRoute`, `EncodedRecording`, and `DecodedRecording` in process-lifetime service storage on V5; do not place these large buffers on a PROS task stack.
- Playback must default to unauthorized in both robot configurations.
- Playback must require healthy odometry, a valid matching-robot file, an armed start confirmation, and exclusive drivetrain ownership.
- Controller X, robot disable, invalid odometry, timeout, or any failed required motion must stop the entire playback and suppress later events.
- Shadow playback must not authorize or silently enter motor-encoder fallback.
- Preserve Kevin Loader, Kevin Park, current LemLib tests, and existing autonomous selection persistence.
- Do not enable the existing `TESTING_AUTONOMOUS` mode to expose Shadow controls.
- Do not edit vendored PROS, LemLib, LVGL, fmt, or JSON sources.
- Clean-build both robot configurations at robot-code checkpoints, then restore and rebuild `USING_BIG_ROBOT false`.
- The first playback implementation targets the selected small differential robot. Big-robot capture and file validation must compile, but H-drive playback returns `UnsupportedRobot` until a separately reviewed holonomic playback design exists.

## File Structure

- `include/aon/shadow/types.hpp`: dependency-free recording, event, route, metadata, and error types.
- `include/aon/shadow/recorder.hpp`: fixed-capacity capture interface.
- `src/aon/shadow/recorder.cpp`: sample cadence, pose validation, event deduplication, and limits.
- `include/aon/shadow/processor.hpp`: pure raw-capture to processed-route interface.
- `src/aon/shadow/processor.cpp`: direction splitting, dwell extraction, point simplification, and event anchoring.
- `include/aon/shadow/codec.hpp`: binary encode/decode and generation-selection interface.
- `src/aon/shadow/codec.cpp`: explicit serialization, CRC-32, and structural validation.
- `include/aon/shadow/storage.hpp`: three-slot SD storage interface and slot summaries.
- `src/aon/shadow/storage.cpp`: platform-independent dual-generation selection, save, load, and deletion policy over an injected file store.
- `src/aon/shadow/storage-pros.cpp`: checked `/usd` stdio implementation of the file-store interface.
- `include/aon/shadow/service-state.hpp`: dependency-free recording and one-shot playback-arm state machine.
- `src/aon/shadow/service-state.cpp`: pure service transitions used by host tests and the PROS service.
- `include/aon/shadow/service.hpp`: process-lifetime recording/playback facade used by GUI and operator control.
- `src/aon/shadow/service.cpp`: mutex-protected state, recorder polling, save processing, slot cache, and play authorization.
- `include/aon/shadow/mechanisms.hpp`: semantic mechanism capture and playback adapter.
- `src/aon/shadow/mechanisms.cpp`: robot-variant mapping to existing intake and piston operations.
- `include/aon/shadow/player.hpp`: closed-loop playback and event-scheduler interface.
- `src/aon/shadow/player.cpp`: dynamic LemLib assets, segment execution, dwell scheduling, health checks, and cancellation.
- `include/aon/auton/actions.hpp`: optional monitored-path poll callback used to synchronize Shadow events.
- `src/aon/auton/actions.cpp`: invoke the Shadow poll callback inside the existing health/cancellation loop.
- `tests/shadow-auton-test.cpp`: dependency-free test runner for recorder, processor, codec, scheduler, and state policy.
- `include/aon/competition/operator-control.hpp`: report effective drive commands and semantic mechanism transitions.
- `include/aon/config/robot-config.hpp`: Shadow playback authorization field.
- `src/aon/config/robot-config.cpp`: false authorization for both robot variants.
- `include/aon/tools/gui/gui.hpp`: Shadow screen state and handlers.
- `include/aon/tools/gui/ui/gui-layout.hpp`: Shadow button geometry.
- `src/aon/tools/gui/gui.cpp`: navigation, touch dispatch, and service polling.
- `src/aon/tools/gui/ui/gui-displays.cpp`: slot/status rendering and confirmations.
- `include/aon/auton/routines.hpp`: Shadow slot routine declarations.
- `src/aon/auton/routine-selectors.cpp`: selected Shadow slot dispatch and status propagation.
- `src/aon/core/robot.cpp`: start the recorder service task and stop playback on disable.
- `docs/CURRENT_HANDOFF.md`: implementation checkpoints and physical-gate results.

---

### Task 1: Pure Capture Model and Fixed-Capacity Recorder

**Files:**
- Create: `include/aon/shadow/types.hpp`
- Create: `include/aon/shadow/recorder.hpp`
- Create: `src/aon/shadow/recorder.cpp`
- Create: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: C++ standard-library headers only.
- Produces: `ResultCode`, `RawSample`, `MechanismEvent`, `Capture`, `Recorder::start`, `Recorder::sample`, `Recorder::event`, and `Recorder::stop`.

- [ ] **Step 1: Write failing tests for cadence, event deduplication, invalid poses, and capacity**

Create a dependency-free test runner with `CHECK` and these cases:

```cpp
#include "aon/shadow/recorder.hpp"

#include <cstdlib>
#include <iostream>

#define CHECK(x) do { if (!(x)) { std::cerr << #x << '\n'; std::exit(1); } } while (false)

using namespace aon::shadow;

RawSample frame(std::uint32_t ms, float x) {
  RawSample value{};
  value.timeMs = ms;
  value.x = x;
  value.poseValid = true;
  return value;
}

void recorderTests() {
  Recorder recorder;
  CHECK(recorder.start(RobotIdentity::Small) == ResultCode::Ok);
  CHECK(recorder.sample(frame(0, 0)) == ResultCode::Ok);
  CHECK(recorder.sample(frame(10, 1)) == ResultCode::SampleTooSoon);
  CHECK(recorder.sample(frame(20, 1)) == ResultCode::Ok);
  CHECK(recorder.event({20, MechanismKind::Cart, 1}) == ResultCode::Ok);
  CHECK(recorder.event({30, MechanismKind::Cart, 1}) == ResultCode::DuplicateEvent);
  CHECK(recorder.event({40, MechanismKind::Cart, 0}) == ResultCode::Ok);
  auto bad = frame(40, 2);
  bad.poseValid = false;
  CHECK(recorder.sample(bad) == ResultCode::Ok);
  bad.timeMs = 60;
  CHECK(recorder.sample(bad) == ResultCode::Ok);
  bad.timeMs = 80;
  CHECK(recorder.sample(bad) == ResultCode::InvalidPose);
  CHECK(!recorder.isRecording());

  Recorder jumping;
  CHECK(jumping.start(RobotIdentity::Small) == ResultCode::Ok);
  CHECK(jumping.sample(frame(0, 0)) == ResultCode::Ok);
  CHECK(jumping.sample(frame(20, 20)) == ResultCode::Ok);
  CHECK(jumping.sample(frame(40, 40)) == ResultCode::PoseJump);
}

int main() {
  recorderTests();
  std::cout << "shadow auton tests passed\n";
}
```

- [ ] **Step 2: Compile to verify the new interfaces are missing**

Run:

```powershell
New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp -o bin\host-tests\shadow-auton-test.exe
```

Expected: FAIL because the Shadow headers and source do not exist.

- [ ] **Step 3: Add the exact dependency-free data model**

Define these public constants and types in `types.hpp`:

```cpp
namespace aon::shadow {
constexpr std::uint32_t kSamplePeriodMs = 20;
constexpr std::uint32_t kMaximumDurationMs = 60000;
constexpr std::size_t kMaximumSamples = 3000;
constexpr std::size_t kMaximumEvents = 512;
constexpr std::size_t kSlotCount = 3;

enum class RobotIdentity : std::uint8_t { Small = 1, Big = 2 };
enum class Direction : std::int8_t { Reverse = -1, Stopped = 0, Forward = 1 };
enum class MechanismKind : std::uint8_t {
  IntakeMode, ScorerHeight, Cart, Trapdoor, Lever, Brooks, Sem, Arrow
};
enum class ResultCode : std::uint8_t {
  Ok, NotRecording, AlreadyRecording, SampleTooSoon, DuplicateEvent,
  CapacityReached, InvalidPose, PoseJump, EmptyRecording, NoSd, ReadOnly,
  OpenFailed, ReadFailed, WriteFailed, FlushFailed, CloseFailed, DeleteFailed,
  CorruptFile, UnsupportedVersion, WrongRobot, InvalidSlot, PlayLocked,
  UnsafeState, Cancelled, OdometryFailure, MotionFailure, UnsupportedRobot
};

struct RawSample {
  std::uint32_t timeMs = 0;
  float x = 0, y = 0, heading = 0;
  std::int8_t leftX = 0, leftY = 0, rightX = 0, rightY = 0;
  std::int8_t leftCommand = 0, rightCommand = 0;
  Direction direction = Direction::Stopped;
  bool poseValid = false;
};
struct MechanismEvent {
  std::uint32_t timeMs = 0;
  MechanismKind kind = MechanismKind::IntakeMode;
  std::int16_t value = 0;
};
struct Capture {
  RobotIdentity robot = RobotIdentity::Small;
  std::array<RawSample, kMaximumSamples> samples{};
  std::array<MechanismEvent, kMaximumEvents> events{};
  std::size_t sampleCount = 0;
  std::size_t eventCount = 0;
  std::uint32_t durationMs = 0;
};
}
```

- [ ] **Step 4: Implement the recorder state machine**

Expose this exact API and implement it without heap allocation:

```cpp
class Recorder {
 public:
  explicit Recorder(RecorderLimits limits = {});
  ResultCode start(RobotIdentity robot);
  ResultCode sample(const RawSample& sample);
  ResultCode event(const MechanismEvent& event);
  ResultCode stop();
  bool isRecording() const;
  ResultCode result() const;
  const Capture& capture() const;
 private:
  Capture capture_{};
  ResultCode result_ = ResultCode::NotRecording;
  bool recording_ = false;
  std::uint32_t invalidPoseCount_ = 0;
  std::uint32_t poseJumpCount_ = 0;
  RecorderLimits limits_{};
};
```

`sample` accepts only elapsed times separated by at least 20 ms, rejects non-finite pose values, aborts on the third consecutive invalid pose, aborts after 60,000 ms or 3,000 accepted frames, and treats a healthy sample as resetting the invalid counter. `event` suppresses consecutive identical kind/value states and aborts rather than overwriting event 513. `stop` rejects an entirely empty capture.

Define `RecorderLimits` with `invalidPoseSamples = 3`,
`poseJumpSamples = 2`, `maximumPoseJumpInches = 8.0F`, and
`maximumHeadingJumpDegrees = 45.0F`. A jump must be consecutive to abort;
ordinary healthy samples reset the jump counter.

- [ ] **Step 5: Run the host test and commit**

Run the compile command from Step 2, then:

```powershell
& .\bin\host-tests\shadow-auton-test.exe
```

Expected: `shadow auton tests passed`.

Commit:

```powershell
git add -- include/aon/shadow/types.hpp include/aon/shadow/recorder.hpp src/aon/shadow/recorder.cpp tests/shadow-auton-test.cpp
git commit -m "Add fixed-capacity shadow route recorder"
```

---

### Task 2: Pure Route Processing and Event Anchoring

**Files:**
- Create: `include/aon/shadow/processor.hpp`
- Create: `src/aon/shadow/processor.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: `Capture`, `Direction`, and `MechanismEvent` from Task 1.
- Produces: `ProcessedRoute process(const Capture&)`, path/dwell segments, and progress-anchored events.

- [ ] **Step 1: Add failing straight, corner, reverse, dwell, and event-order tests**

Add tests that construct captures directly and assert:

```cpp
const ProcessedRoute straight = process(captureAt({{0, 0}, {0, 6}, {0, 12}}));
CHECK(straight.result == ResultCode::Ok);
CHECK(straight.segmentCount == 1);
CHECK(straight.segments[0].direction == Direction::Forward);
CHECK(straight.segments[0].points.front().y == 0.0F);
CHECK(straight.segments[0].points.back().y == 12.0F);

const ProcessedRoute paused = process(captureWithPauseAndCartEvent());
CHECK(paused.segmentCount == 3);
CHECK(paused.segments[1].kind == SegmentKind::Dwell);
CHECK(paused.segments[1].durationMs == 500);
CHECK(paused.events[0].segmentIndex == 1);
CHECK(paused.events[0].offsetMs == 200);
```

Include a heading sequence `179, -179, -175` and assert it does not create a false discontinuity. Include forward-to-reverse input and assert it creates two motion segments. Include an event at a geometric corner and assert simplification retains the anchor point.

- [ ] **Step 2: Compile to verify processing symbols are missing**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp src\aon\shadow\processor.cpp -o bin\host-tests\shadow-auton-test.exe
```

Expected: FAIL for missing `processor.hpp` or `process`.

- [ ] **Step 3: Define bounded processed-route types**

Add the exact interfaces below; use fixed capacities so corrupt files cannot force unbounded allocation:

```cpp
constexpr std::size_t kMaximumSegments = 64;
constexpr std::size_t kMaximumPathPoints = 1000;
enum class SegmentKind : std::uint8_t { Motion, Dwell };
struct PathPoint { float x = 0, y = 0, speed = 0; };
struct RouteSegment {
  SegmentKind kind = SegmentKind::Motion;
  Direction direction = Direction::Forward;
  std::uint16_t firstPoint = 0, pointCount = 0;
  std::uint32_t durationMs = 0;
};
struct AnchoredEvent {
  MechanismEvent event{};
  std::uint16_t segmentIndex = 0;
  float progress = 0;
  std::uint32_t offsetMs = 0;
};
struct ProcessedRoute {
  ResultCode result = ResultCode::EmptyRecording;
  RawSample start{};
  std::array<RouteSegment, kMaximumSegments> segments{};
  std::array<PathPoint, kMaximumPathPoints> points{};
  std::array<AnchoredEvent, kMaximumEvents> events{};
  std::size_t segmentCount = 0, pointCount = 0, eventCount = 0;
};
ProcessedRoute process(const Capture& capture);
```

- [ ] **Step 4: Implement deterministic processing**

Normalize headings with `std::remainder(delta, 360.0F)`. Treat at least five consecutive stopped samples as a dwell. Split when direction changes. Preserve the first/last point, points adjacent to a split, event anchors, any point whose perpendicular error exceeds 0.5 inches, and any point whose normalized heading differs by at least 5 degrees from the last retained point. Infer path speed as `clamp(round(hypot(delta) / 0.02 * 2), 20, 100)`. Reject output exceeding any fixed capacity instead of truncating it.

- [ ] **Step 5: Run tests and commit**

Run the Step 2 compile command and executable. Expected: `shadow auton tests passed`.

```powershell
git add -- include/aon/shadow/processor.hpp src/aon/shadow/processor.cpp tests/shadow-auton-test.cpp
git commit -m "Process shadow captures into anchored paths"
```

---

### Task 3: Versioned Codec, CRC, and Dual-Generation Selection

**Files:**
- Create: `include/aon/shadow/codec.hpp`
- Create: `src/aon/shadow/codec.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: `Capture` and `ProcessedRoute`.
- Produces: `encode`, `decode`, `inspect`, CRC-32 validation, and `chooseGeneration`.

- [ ] **Step 1: Add failing round-trip and corruption tests**

Test deterministic encoding, full round trip, one-byte corruption, truncation at every section boundary, wrong magic, version 2, wrong robot, non-finite float bits, impossible counts, and generation choice:

```cpp
EncodedRecording bytes{};
CHECK(encode(capture, route, 7, bytes) == ResultCode::Ok);
DecodedRecording decoded{};
CHECK(decode(bytes.data.data(), bytes.size, RobotIdentity::Small, decoded) == ResultCode::Ok);
CHECK(decoded.generation == 7);
bytes.data[bytes.size - 1] ^= 0x01;
CHECK(decode(bytes.data.data(), bytes.size, RobotIdentity::Small, decoded) == ResultCode::CorruptFile);
CHECK(chooseGeneration({true, 4}, {true, 9}) == Generation::B);
CHECK(chooseGeneration({true, 4}, {false, 0}) == Generation::A);
```

- [ ] **Step 2: Compile to verify codec symbols are missing**

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp src\aon\shadow\processor.cpp src\aon\shadow\codec.cpp -o bin\host-tests\shadow-auton-test.exe
```

Expected: FAIL for missing codec declarations.

- [ ] **Step 3: Define the on-disk contract**

Use magic `AONSHDW1`, format version `1`, and this encoded header order: magic[8], version u16, header-size u16, generation u32, robot u8, sample-period u16, duration u32, start X/Y/heading f32, sample/event/segment/point/anchored-event counts u16, five section sizes u32, payload CRC-32 u32. Expose:

```cpp
constexpr std::size_t kMaximumEncodedBytes = 256 * 1024;
struct EncodedRecording { std::array<std::uint8_t, kMaximumEncodedBytes> data{}; std::size_t size = 0; };
struct DecodedRecording { std::uint32_t generation = 0; Capture capture{}; ProcessedRoute route{}; };
struct GenerationInfo { bool valid = false; std::uint32_t generation = 0; };
enum class Generation : std::uint8_t { None, A, B };
ResultCode encode(const Capture&, const ProcessedRoute&, std::uint32_t, EncodedRecording&);
ResultCode decode(const std::uint8_t*, std::size_t, RobotIdentity, DecodedRecording&);
Generation chooseGeneration(GenerationInfo a, GenerationInfo b);
```

- [ ] **Step 4: Implement explicit serialization and strict validation**

Use byte-wise `putU16`, `putU32`, `getU16`, and `getU32`; convert floats through `std::memcpy` to/from `std::uint32_t`; calculate CRC-32 polynomial `0xEDB88320`. Validate all sizes and counts before reading a section, require exactly the declared file length, then validate finite poses, chronological timestamps, segment point ranges, event segment ranges, and the checksum.

- [ ] **Step 5: Run tests and commit**

Run the Step 2 compile and executable. Expected: `shadow auton tests passed`.

```powershell
git add -- include/aon/shadow/codec.hpp src/aon/shadow/codec.cpp tests/shadow-auton-test.cpp
git commit -m "Add versioned shadow recording codec"
```

---

### Task 4: SD Storage and Recording Service

**Files:**
- Create: `include/aon/shadow/storage.hpp`
- Create: `src/aon/shadow/storage.cpp`
- Create: `src/aon/shadow/storage-pros.cpp`
- Create: `include/aon/shadow/service-state.hpp`
- Create: `src/aon/shadow/service-state.cpp`
- Create: `include/aon/shadow/service.hpp`
- Create: `src/aon/shadow/service.cpp`
- Modify: `src/aon/core/robot.cpp`

**Interfaces:**
- Consumes: Tasks 1–3, an injected `FileStore`, LemLib pose, controller axes, and robot identity.
- Produces: slot summaries, dual-generation save/load/delete, a PROS `/usd` file store, recorder task, and thread-safe service state.

- [ ] **Step 1: Add host tests for service state policy**

Use an in-memory `FileStore` with injectable write/flush/close failures. Test that a failed second-generation save leaves generation 1 loadable. Test invalid slots, overwrite confirmation, `RECORDING -> PROCESSING -> SAVED`, failure retention, play lock, and one-shot arming:

```cpp
MemoryFileStore files;
Storage storage(files);
CHECK(storage.save(1, RobotIdentity::Small, capture, route) == ResultCode::Ok);
files.failNextWrite = true;
CHECK(storage.save(1, RobotIdentity::Small, capture2, route2) == ResultCode::WriteFailed);
DecodedRecording loaded{};
CHECK(storage.load(1, RobotIdentity::Small, loaded) == ResultCode::Ok);
CHECK(loaded.generation == 1);

ServiceStateMachine state;
CHECK(state.beginRecord(0, true) == ResultCode::InvalidSlot);
CHECK(state.beginRecord(1, false) == ResultCode::Ok);
CHECK(state.status().mode == ServiceMode::Recording);
CHECK(state.authorizePlay(false, true, true) == ResultCode::PlayLocked);
CHECK(state.armPlay(1) == ResultCode::Ok);
CHECK(state.consumeArm(1));
CHECK(!state.consumeArm(1));
```

- [ ] **Step 2: Define the storage and service APIs**

```cpp
struct SlotSummary {
  ResultCode result = ResultCode::EmptyRecording;
  bool valid = false;
  std::uint32_t generation = 0, durationMs = 0;
  float startX = 0, startY = 0, startHeading = 0;
};
class FileStore {
 public:
  virtual ~FileStore() = default;
  virtual ResultCode read(const char* path, EncodedRecording& out) const = 0;
  virtual ResultCode write(const char* path, const std::uint8_t* data,
                           std::size_t size) = 0;
  virtual ResultCode erase(const char* path) = 0;
};
class Storage {
 public:
  explicit Storage(FileStore& files);
  SlotSummary inspect(std::uint8_t slot, RobotIdentity robot) const;
  ResultCode load(std::uint8_t slot, RobotIdentity robot, DecodedRecording& out) const;
  ResultCode save(std::uint8_t slot, RobotIdentity robot, const Capture&, const ProcessedRoute&);
  ResultCode erase(std::uint8_t slot);
 private:
  FileStore& files_;
};
enum class ServiceMode : std::uint8_t { Idle, Recording, Processing, Saved, Invalid, Playing, Cancelled };
struct Status { ServiceMode mode; ResultCode result; std::uint8_t slot; std::uint32_t changedAt; };
class Service {
 public:
  ResultCode beginRecording(std::uint8_t slot, bool overwriteConfirmed);
  ResultCode stopAndSave();
  ResultCode erase(std::uint8_t slot, bool confirmed);
  SlotSummary slot(std::uint8_t slot) const;
  Status status() const;
  void pollRecorder();
  ResultCode armPlayback(std::uint8_t slot, bool startConfirmed);
  bool consumePlaybackArm(std::uint8_t slot);
  void cancel();
};
Service& service();
```

Define `ServiceStateMachine` in `service-state.hpp` with `beginRecord`,
`finishSave`, `authorizePlay`, `armPlay`, `consumeArm`, `cancel`, and `status`.
It contains no PROS types. The PROS-facing `Service` composes this state machine
rather than duplicating transition rules.

- [ ] **Step 3: Implement dual-generation stdio storage**

Validate slot 1–3 before formatting paths. Implement generation policy in
`storage.cpp` exclusively through `FileStore`. Implement `SdFileStore` in
`storage-pros.cpp`; check `pros::usd::is_installed()` before every operation.
Use exactly `/usd/aon-shadow-slot-<slot>-a.bin` and
`/usd/aon-shadow-slot-<slot>-b.bin` after validating `<slot>` as 1, 2, or 3.
Load and decode both `a` and `b`, select the newest valid generation, and report
the most specific failure only when neither is valid. Save to the invalid or
older generation, verify every `fwrite`, `fflush`, and `fclose`, reopen and
decode the just-written bytes, and only then report `Ok`. Delete both files;
`ENOENT` is success, other errors are `DeleteFailed`.

- [ ] **Step 4: Implement the process-lifetime service and recorder task**

Protect state with one `pros::Mutex`. Keep SD I/O outside the held lock by snapshotting the capture and switching to `Processing` first. `beginRecording` succeeds only when `!pros::competition::is_disabled()` and `!pros::competition::is_autonomous()`. `pollRecorder` reads LemLib pose, controller axes, and the most recent effective drive command every 20 ms only while recording; it stops and begins saving when driver control ends or 60 seconds is reached. Add one task in `Robot::initialize`:

```cpp
pros::Task shadowRecorderTask([] {
  while (true) {
    aon::shadow::service().pollRecorder();
    pros::delay(aon::shadow::kSamplePeriodMs);
  }
});
```

- [ ] **Step 5: Build both robot variants, restore small, and commit**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp src\aon\shadow\processor.cpp src\aon\shadow\codec.cpp src\aon\shadow\storage.cpp src\aon\shadow\service-state.cpp -o bin\host-tests\shadow-auton-test.exe
& .\bin\host-tests\shadow-auton-test.exe
```

Expected: `shadow auton tests passed`. Then set `USING_BIG_ROBOT true`, run `make clean` and `make`; restore `false`, run `make clean` and `make`. Expected: both builds complete; only the existing vendored `json.hpp` deprecation warning is accepted.

```powershell
git add -- include/aon/shadow/storage.hpp src/aon/shadow/storage.cpp src/aon/shadow/storage-pros.cpp include/aon/shadow/service-state.hpp src/aon/shadow/service-state.cpp include/aon/shadow/service.hpp src/aon/shadow/service.cpp src/aon/core/robot.cpp tests/shadow-auton-test.cpp include/aon/constants.hpp
git commit -m "Add resilient SD shadow recording service"
```

---

### Task 5: Semantic Driver and Mechanism Capture

**Files:**
- Create: `include/aon/shadow/mechanisms.hpp`
- Create: `src/aon/shadow/mechanisms.cpp`
- Modify: `include/aon/competition/operator-control.hpp`
- Modify: `include/aon/lemlib/drive-io.hpp`
- Modify: `src/aon/lemlib/chassis.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: the active `Service`, existing driver decisions, intake API, and piston API.
- Produces: one semantic capture call per effective state transition and a playback adapter for Task 8.

- [ ] **Step 1: Add failing mechanism vocabulary and deduplication tests**

Assert each semantic state round-trips through the codec and that repeated held-button iterations create one event, while release to idle creates a second event. Cover small robot: store, corridor, reject, score-bottom, lever, scorer height, cart, trapdoor, Brooks, Arrow. Cover big robot: store, score-bottom, normal/inverted sort, cart, Brooks, Sem.

- [ ] **Step 2: Add a narrow capture API**

```cpp
enum class IntakeIntent : std::int16_t {
  Idle, Store, Corridor, Reject, ScoreBottom, ScoreMiddle, ScoreTop,
  SortNormal, SortInverted
};
void captureDrive(int left, int right);
void captureMechanism(MechanismKind kind, std::int16_t value);
void applyDriverIntakeIntent(IntakeIntent intent);
ResultCode applyMechanism(const MechanismEvent& event);
```

`captureDrive` stores normalized `[-127,127]` effective left/right intent in the service without touching hardware. `captureMechanism` timestamps through the active recording and lets `Recorder::event` deduplicate.

- [ ] **Step 3: Instrument operator control after decisions are made**

For each held-mode chain, compute one semantic intent, execute the existing command unchanged, then report it. For toggles, report the resulting state, not merely the button press. For LemLib curvature, report clamped `throttle + turn` and `throttle - turn`. For H-drive, report forward left/right intent and retain all four raw axes in samples; do not claim H-drive playback support.

Use this shape so inactive recording is one cheap branch:

```cpp
const auto intent = mainController.get_digital(DIGITAL_R2)
    ? IntakeIntent::Store
    : mainController.get_digital(DIGITAL_L2)
        ? IntakeIntent::Reject
        : mainController.get_digital(DIGITAL_L1)
            ? IntakeIntent::ScoreBottom
            : IntakeIntent::Idle;
applyDriverIntakeIntent(intent);
aon::shadow::captureMechanism(MechanismKind::IntakeMode,
                              static_cast<std::int16_t>(intent));
```

- [ ] **Step 4: Implement the playback mechanism adapter**

Validate each kind/value pair before calling existing intake/piston functions. Return `WrongRobot` for Sem on small or Arrow/scorer/trapdoor/lever on big. `IntakeMode::Idle` must stop intake output. Never toggle during playback; map events to explicit activate/deactivate or explicit target states.

- [ ] **Step 5: Run host tests, build both variants, restore small, and commit**

Expected: host tests pass; both PROS builds pass with no new warnings.

```powershell
git add -- include/aon/shadow/mechanisms.hpp src/aon/shadow/mechanisms.cpp include/aon/competition/operator-control.hpp include/aon/lemlib/drive-io.hpp src/aon/lemlib/chassis.cpp tests/shadow-auton-test.cpp include/aon/constants.hpp
git commit -m "Capture semantic shadow mechanism events"
```

---

### Task 6: Three-Slot Brain UI with Playback Locked

**Files:**
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `include/aon/tools/gui/ui/gui-layout.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`
- Modify: `src/aon/tools/gui/ui/gui-displays.cpp`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: `shadow::Service`, `SlotSummary`, and `Status`.
- Produces: normal-GUI Shadow navigation, slot selection, overwrite/delete confirmation, record/stop/save, and visible errors.

- [ ] **Step 1: Add the Shadow screen and non-overlapping button geometry**

Add `ShadowMenu` to `GuiScreen`. Define three 140x42 slot buttons across the top, and bottom-row `BACK`, `RECORD`/`STOP SAVE`, `DELETE`, and disabled `PLAY` rectangles. Keep all coordinates within 480x240 and outside the competition field-control status bar.

- [ ] **Step 2: Render complete state without enabling debug mode**

Add `displayShadowMenu()` that renders slot number, `EMPTY/READY/INVALID`, duration, and start pose. Map result codes to short labels through one exhaustive `shadowResultName(ResultCode)` function. Render `PLAY LOCKED` whenever config authorization is false. Add a `SHADOW` navigation button to the normal main menu.

- [ ] **Step 3: Implement confirmation and touch behavior**

First tap on RECORD for a valid slot shows `OVERWRITE?`; second tap within five seconds begins. DELETE follows the same two-tap confirmation. STOP transitions to `PROCESSING` and disables buttons until save finishes. PLAY may show metadata and placement instructions but must not arm or move while authorization is false. Refresh only when `Status::changedAt` or a slot generation changes.

- [ ] **Step 4: Record the physical gate and commit**

Update the handoff with exact upload configuration and this mandatory stop:

```markdown
Shadow recording implementation is complete with playback authorization false.
Do not implement or expose playback until the existing LemLib/native baseline
gate passes and all three SD slots survive a full-reboot save/load test.
```

Run host tests and both builds, restore small, then commit:

```powershell
git add -- include/aon/tools/gui/gui.hpp include/aon/tools/gui/ui/gui-layout.hpp src/aon/tools/gui/gui.cpp src/aon/tools/gui/ui/gui-displays.cpp docs/CURRENT_HANDOFF.md include/aon/constants.hpp
git commit -m "Add locked shadow recording controls"
```

---

### Physical Gate A: Recording Only

Stop implementation and have the user perform and report all of these:

- Existing AUT3 `TEST LemLib 12in`: physical distance and final X/Y/heading.
- Separately rebooted native Kevin routine: GUI, drivetrain, and mechanisms unchanged.
- SD slots 1, 2, and 3: record, save, full power cycle, and load metadata.
- No SD, read-only SD, card removed during save, and corrupted generation: no crash and no false `SAVED` state.
- Mechanism-only recording plus straight, L-shaped, reverse, and paused routes: processed metadata valid without autonomous movement.

Commit the measured results to `docs/CURRENT_HANDOFF.md`. Do not continue to Task 7 until Gate A passes.

---

### Task 7: Pure Playback Policy and Event Scheduler

**Files:**
- Create: `include/aon/shadow/player.hpp`
- Create: `src/aon/shadow/player.cpp`
- Modify: `tests/shadow-auton-test.cpp`

**Interfaces:**
- Consumes: validated `DecodedRecording`, processed segments/events, and injected motion/mechanism callbacks.
- Produces: testable playback state transitions and the PROS-facing `Player::run` used by Task 8.

- [ ] **Step 1: Add failing policy, progress, dwell, and cancellation tests**

Use fake callbacks and assert: unauthorized playback makes zero calls; wrong robot makes zero calls; start not armed makes zero calls; movement events fire in order at crossing progress; dwell events fire by offset; a failed motion suppresses remaining events; cancellation stays latched; and a consumed arm cannot run twice.

- [ ] **Step 2: Define injected playback contracts**

```cpp
struct PlaybackPolicy {
  bool authorized = false, autonomous = false, testing = false;
  bool startArmed = false, odometryHealthy = false, cancelled = false;
  RobotIdentity activeRobot = RobotIdentity::Small;
};
struct PlaybackCallbacks {
  std::function<ResultCode(
      const RouteSegment&, const PathPoint*, std::size_t,
      const std::function<void(float)>& onProgress)> follow;
  std::function<ResultCode(const MechanismEvent&)> mechanism;
  std::function<bool()> cancelled;
  std::function<bool()> disabled;
  std::function<bool()> odometryHealthy;
  std::function<void()> stopAll;
  std::function<void(std::uint32_t)> delay;
};
ResultCode validatePlayback(const PlaybackPolicy&, const DecodedRecording&);
ResultCode playRecording(const DecodedRecording&, PlaybackCallbacks&);
```

- [ ] **Step 3: Implement fail-closed scheduling**

Validate policy before any callback. Pass an `onProgress` observer to each motion callback and dispatch newly crossed progress anchors monotonically, never twice. During dwell, delay in 10 ms slices and recheck cancellation, disable, and odometry. After every motion callback, stop immediately unless it returned `Ok`. Call `stopAll` on every exit path, including success, and return the original failure category.

- [ ] **Step 4: Run host tests and commit**

Compile the host binary with recorder, processor, codec, and player sources. Expected: `shadow auton tests passed`.

```powershell
git add -- include/aon/shadow/player.hpp src/aon/shadow/player.cpp tests/shadow-auton-test.cpp
git commit -m "Add fail-closed shadow playback scheduler"
```

---

### Task 8: LemLib Playback, Autonomous Registration, and Start Arming

**Files:**
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `src/aon/shadow/player.cpp`
- Modify: `include/aon/auton/actions.hpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`
- Modify: `src/aon/tools/gui/gui.cpp`
- Modify: `src/aon/tools/gui/ui/gui-displays.cpp`
- Modify: `src/aon/core/robot.cpp`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: Task 7 scheduler, `Actions::followPath`, motion cancellation, mechanism adapter, service arm, and competition state.
- Produces: locked-by-default Shadow playback from testing UI and autonomous dispatch.

- [ ] **Step 1: Add independent playback authorization**

Append `bool shadowPlaybackAuthorized;` to `RobotConfig` and initialize it to `false` for small and big configurations. Do not reuse `automaticFallbackAuthorized` or `forcedEncoderTestingAuthorized`.

- [ ] **Step 2: Convert each processed motion segment into a runtime LemLib asset**

Serialize validated points as `x, y, speed\n` into a 64 KiB byte buffer, append a terminating newline, then construct `asset{buffer.data(), used}`. Extend `Actions::followPath` with an optional `std::function<void()> onPoll = {}` argument and call it once per iteration of the existing health/cancellation loop. The Shadow adapter's poll callback projects the current LemLib pose onto the active polyline, calculates monotonic `[0,1]` progress, and invokes the Task 7 observer so mechanism events occur while the path is moving. Call `Actions::followPath` with recorded direction, a 6-inch lookahead, and timeout `max(segment.durationMs * 2 + 1000, 2000)`. The buffer must remain alive until the synchronous monitored action returns. Reject big robot playback with `UnsupportedRobot` before commanding hardware.

- [ ] **Step 3: Wire policy and emergency paths**

Build policy from config, `pros::competition::is_autonomous()`, the explicit testing invocation flag, the consumed start arm, current robot identity, and an odometry-health snapshot. Use `actions().cancelMotion()`, drivetrain stop, and intake stop for `stopAll`. Call `service().cancel()` from `Robot::disabled()` and the existing Controller X safety path.

- [ ] **Step 4: Expose slots without displacing current routines**

Add `RunShadowSlot1/2/3()` declarations and one shared `runShadowSlot(std::uint8_t)`. Add a separate Shadow slot-selection row or submenu; do not replace the three alliance routine arrays during validation. In testing, placement confirmation arms and immediately dispatches. For competition, confirmation arms the slot and selecting it registers the corresponding routine. Changing selection, disabling, cancellation, or reboot clears the arm.

- [ ] **Step 5: Build with playback still false and commit**

Run host tests, clean small and big builds, restore and rebuild small. Verify Brain rendering says `PLAY LOCKED`. Commit without changing either authorization initializer:

```powershell
git add -- include/aon/config/robot-config.hpp src/aon/config/robot-config.cpp src/aon/shadow/player.cpp include/aon/auton/actions.hpp src/aon/auton/actions.cpp include/aon/auton/routines.hpp src/aon/auton/routine-selectors.cpp include/aon/tools/gui/gui.hpp src/aon/tools/gui/gui.cpp src/aon/tools/gui/ui/gui-displays.cpp src/aon/core/robot.cpp docs/CURRENT_HANDOFF.md include/aon/constants.hpp
git commit -m "Integrate locked shadow path playback"
```

---

### Physical Gate B: Progressive Playback Authorization

Authorization changes are separate checkpoints; never enable all route types at once.

- [ ] Set `shadowPlaybackAuthorized = true` only for the small robot and build/upload.
- [ ] Place the robot at the recorded start, confirm, and play a 6-inch straight path with mechanisms absent.
- [ ] Record physical endpoint error and final LemLib X/Y/heading.
- [ ] Test Controller X cancellation and field disable; confirm no later drive or mechanism command occurs.
- [ ] Validate an L-shaped path, reverse path, and stationary dwell separately.
- [ ] Add one mechanism kind per run and verify explicit final state after cancellation.
- [ ] Validate a complete 15-second route at least three times from the same start.
- [ ] Validate a 60-second skills route only after the 15-second route is repeatable.
- [ ] Relock playback immediately if odometry, path direction, event ordering, cancellation, or endpoint accuracy fails.

Record measurements and the exact authorized configuration in `docs/CURRENT_HANDOFF.md`, then commit the authorization checkpoint. Automatic motor-encoder fallback remains false regardless of Shadow results.

---

### Task 9: Final Verification and Handoff

**Files:**
- Modify: `docs/CURRENT_HANDOFF.md`
- Modify: `include/aon/tools/gui/GUIDE_GUI.md`

**Interfaces:**
- Consumes: completed tasks and physical results.
- Produces: reproducible operator instructions and verified repository state.

- [ ] **Step 1: Run all automated verification**

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\motion-fallback-test.cpp src\aon\auton\motion-health.cpp src\aon\auton\fallback-geometry.cpp -o bin\host-tests\motion-fallback-test.exe
& .\bin\host-tests\motion-fallback-test.exe
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\shadow-auton-test.cpp src\aon\shadow\recorder.cpp src\aon\shadow\processor.cpp src\aon\shadow\codec.cpp src\aon\shadow\storage.cpp src\aon\shadow\service-state.cpp src\aon\shadow\player.cpp -o bin\host-tests\shadow-auton-test.exe
& .\bin\host-tests\shadow-auton-test.exe
```

Expected: both executables report passed. Clean-build big, restore `USING_BIG_ROBOT false`, clean-build small, and verify no new warnings.

- [ ] **Step 2: Document exact Brain workflow and recovery**

Document slot selection, overwrite/delete confirmation, start placement, one-shot arming, testing versus competition playback, Controller X cancellation, SD error labels, dual-generation recovery, and all physical gates. State explicitly that big H-drive playback is unsupported and encoder fallback is independently locked.

- [ ] **Step 3: Verify repository state and commit**

```powershell
git diff --check
git status --short --branch
git add -- docs/CURRENT_HANDOFF.md include/aon/tools/gui/GUIDE_GUI.md
git commit -m "Document shadow auton operation and validation"
```

Expected: clean `Testing` branch, small-robot upload configuration, all checkpoints and physical results recorded, and no claim that unperformed physical tests passed.
