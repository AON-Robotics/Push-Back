# VEX U Performance and Real-Time Systems Audit

Audit date: 2026-08-10  
Repository: `AON-Robotics/Push-Back`  
Branch: `Testing`  
Audited source commit: `ac0fedcb1627a7fd0208ca8eaf62a3e2f9117f3c`  
Default build configuration: small robot (`USING_BIG_ROBOT=false`), normal GUI, LemLib driver enabled

## 1. Executive summary

This is a static engineering audit, not a claim that a particular loop is already missing deadlines. No competition source was changed. Costs called **observed** are directly present in source; costs called **likely** or **candidate** need on-robot profiling before students change behavior.

The highest-leverage findings are:

1. **P0 — native odometry snapshots are not coherent.** `Odometry::getPose()` in `src/aon/odometry.cpp:108` obtains X, Y, and heading through three independent mutex acquisitions. Every getter ignores the return value of a 1 ms timed `take()`, then reads and gives the mutex anyway. A reader can therefore receive coordinates from different updates or access state without owning the lock after a timeout. `Odometry::update()` also reads its own state through repeated getters and locks while publishing position and heading separately.
2. **P0 — several `volatile` fields are cross-task data races in C++.** Intake `scanning`/`scoreDown`, hardware `alliance`, and Orbit `following`/`braking`/`scanning` are not made thread-safe by `volatile`. The GUI/autonomous/opcontrol tasks write some of these while background tasks read them. This is a correctness and determinism issue before it is a performance issue.
3. **P1 — the safety task becomes an unthrottled loop while X is held.** `autonSafety()` in `include/aon/globals.hpp:128-134` repeatedly invokes `STOP()` with no delay inside the inner loop. That can consume a core and repeatedly issue stop/cancel motor operations exactly during an abnormal event.
4. **P1 — two drivetrain/odometry stacks coexist.** LemLib is calibrated at boot and owns the normal driver/autonomous motion path, while `legacy_motion::prepare()` can start the native 10 ms odometry task for native routes. Both stacks wrap the same physical drive and tracking ports. This is deliberate migration compatibility, but it adds duplicate sensor work and creates a motor-ownership hazard unless selection guarantees that only one motion stack commands at a time.
5. **P1 — relative delays make critical periods equal to work time plus delay.** Odometry, opcontrol, Shadow sampling, fallback motion, and native path loops use `pros::delay()`. Their actual start-to-start period drifts with sensor, mutex, logging, and motor API latency. Deadline scheduling is most valuable in continuous controllers and samplers.
6. **P1 — native path following logs four formatted displays per 10 ms iteration.** `Drivetrain::follow()` and the derived `goToPose()`/`follow()` implementations print to LCD and controller from control loops, while repeatedly rebuilding pose snapshots. These diagnostics should remain in source but be compiled out or rate-limited in competition builds.
7. **P1 — current small-robot intake tasks intentionally block within periodic loops.** `Intake::scan()` pauses 500 ms after detection; `Intake::sort()` pauses 100/125 ms during eject/accept. During those delays the task cannot observe cancellation or state changes. The big-robot sorter is already nonblocking, but it reads optical hue every 10 ms even when releasing is inactive and repeatedly sends the same motor command in `IDLE`.
8. **P2 — Shadow recording is already allocation-bounded.** Capture, processing, path serialization, and playback buffers are fixed-size arrays. The 20 ms recorder path does pose/controller reads plus one service mutex only while recording. Measure it, but do not replace this safe bounded design speculatively.
9. **P2 — the build already optimizes for size and dead-section removal.** `common.mk` uses Cortex-A9 hard-float/NEON flags, `-Os`, `-ffunction-sections`, `-fdata-sections`, and linker `--gc-sections`. `-O2` and LTO are experiments, not automatic upgrades; retain whichever configuration wins repeatability and maximum-loop-time tests.

Recommended order: fix/validate snapshot and synchronization correctness; establish measurements; move critical loops to deadline scheduling; remove or gate loop logging; then reduce sensor/motor calls based on measurements. Do not start with trig substitutions, fixed-point math, STL removal, or comment deletion.

## 2. Scope, method, and baseline limitations

### 2.1 What was inspected

- All first-party `src/**/*.cpp` files and `include/aon/**/*.hpp` headers.
- Task construction, `while`/`do` control loops, `pros::delay`, mutexes, atomics, sensors, motor calls, formatted output, dynamic containers, and floating-point math.
- `Makefile`, `common.mk`, `project.pros`, linked archives, and existing build artifacts.
- Both compile-time robot variants where behavior differs, with the current `USING_BIG_ROBOT=false` path clearly distinguished.

Vendored PROS, LemLib, fmt, nlohmann JSON, and LVGL implementation headers were not treated as student-owned optimization targets. LemLib 0.5.6 is supplied as `firmware/LemLib.a`, so its internal loop periods and synchronization cannot be proven from this repository. The report therefore treats LemLib internals as an opaque library and measures the first-party calls around it.

### 2.2 What was not measured

No V5 brain was attached, so this audit does not invent execution-time numbers, mutex wait times, sensor transaction times, or autonomous error. Existing `bin/monolith.elf` and `bin/monolith.bin` were dated 2026-08-04 and are not a fresh controlled baseline. Their host file sizes (13,774,392 and 1,152,560 bytes) must not be confused with ELF section usage or user-program RAM. A fresh `arm-none-eabi-size`/PROS build report is required for baseline numbers.

### 2.3 Cost and priority notation

- **Low:** a few local operations or one cheap state check.
- **Medium:** several device calls, formatted output, container/string copies, or nontrivial math.
- **High:** multiple device calls plus formatting/locking, unbounded/busy work, or whole-path processing.
- **P0:** correctness or serious real-time hazard.
- **P1:** high performance/reliability impact.
- **P2:** moderate impact.
- **P3:** minor/speculative; profile first.

## 3. Performance baseline: periodic and high-frequency paths

Periods below are source-intended minimum delays. With `pros::delay(N)`, actual start-to-start period is `work + N ms` and can be much longer after blocking branches.

| Subsystem | File | Function | Intended / source loop frequency | Likely CPU/device cost | Possible jitter source | Synchronization cost | Priority | Reason |
|---|---|---|---|---|---|---|---|---|
| Driver control | `src/aon/core/robot.cpp:63` + `include/aon/competition/operator-control.hpp:314` | `Robot::opcontrol()` / `Run()` | 10 ms delay; less than 100 Hz actual | Medium: controller reads, LemLib drive call, motor telemetry for lever, Shadow atomics/events | Controller/device calls, mechanism branches, Shadow mutex while recording | Shadow event capture takes service mutex only when recording | P1 | Primary driver latency path; relative scheduling and repeated unchanged motor commands |
| LemLib odometry/motion | `src/aon/lemlib/chassis.cpp:146` | `chassis()` / library internals | Opaque in static LemLib 0.5.6 archive | Likely medium-high | Library sensor/motion task scheduling, device calls | Library-internal, not visible here | P1 | Governs current driver/autonomous pose and motion; instrument externally rather than guess |
| Autonomous monitor | `src/aon/auton/actions.cpp:179` | `runMonitored()` | 20 ms delay while LemLib reports motion | High: pose + two motor vectors + 3 rotation + IMU/status per iteration | Device calls, `std::function`, monitor work, logging on transitions | `MotionControl` mutex calls; LemLib internals | P1 | Safety-critical supervisor samples far more telemetry than ordinary controller loops |
| Timed autonomous drive | `src/aon/auton/actions.cpp:373` | `Actions::arcadeFor()` | 20 ms delay | Medium-high: drive telemetry or full motion sample | Same sensor/device latency, relative delay | Motion-control mutex | P1 | Deadline and duration accuracy affect autonomous behavior |
| Encoder fallback drive | `src/aon/auton/encoder-motion.cpp:46` | `driveDistance()` | 20 ms delay | High: both motor position vectors, all 3 trackers, IMU and status even though drive uses motor averages | Device calls, motor commands, relative delay | Motion-control calls | P1 | Safety fallback should be predictable; sensor sampling is broader than each decision needs |
| Encoder fallback turn | `src/aon/auton/encoder-motion.cpp:94` | `turn()` | 20 ms delay | High: same full `sampleDriveSensors()` set | Device calls, motor commands, relative delay | Motion-control calls | P1 | Same as fallback drive; IMU/encoder switchover must remain reliable |
| Legacy native odometry | `src/aon/odometry.cpp:124` | `Odometry::initialize()` / `update()` | 10 ms delay; less than 100 Hz actual; starts lazily | High: 2 rotations + IMU, repeated locks/getters, 6 active trig calls, duplicate model | Sensor reads, mutex timeouts, scheduling drift | Multiple position/orientation lock operations per update | P0 | Incoherent publishing and lock misuse; duplicate calculations are secondary |
| Legacy pure pursuit | `include/aon/drivetrain/drivetrain.hpp:764` | `Drivetrain::follow()` | 10 ms delay | Very high: path copied inside controller, repeated poses, trig/hypot, four formatted displays, motor calls | LCD/controller I/O, pose locks, path scan, relative delay | Multiple odometry locks | P1 | Debug I/O can dominate the 10 ms control budget |
| Derived legacy pose/path | `src/aon/drivetrain/{differential-drive,h-drive,mecanum,x-drive}.cpp` | `goToPose()` / `follow()` | 10 ms or caller-provided delay | High for same reasons as base path follower | Formatted display, pose reads, relative delay | Multiple odometry locks | P1 | Same hot-loop pattern appears in multiple implementations |
| Legacy profile/PID motion | `include/aon/drivetrain/drivetrain.hpp:312-517,611` | `drivePID()`, `turnPID()`, profiled moves, `driveAngleOfArc()` | 10 or 20 ms delay | Medium-high: sensors, profile/PID math, motor calls | Device latency, variable work, timeout checks | Odometry reads where used | P1 | Blocking autonomous controllers need stable sample time |
| Intake scan (small, active build) | `src/aon/intake.cpp:374` | `Intake::scan()` | 50 ms idle; 500 ms pause after detection | Low idle; device + motors when active | 500 ms debounce block, distance read | `scanning` is an unsynchronized volatile | P1 | Cancellation/state response can be delayed by half a second |
| Intake sort (small, active build) | `src/aon/intake.cpp:398` | `Intake::sort()` | 25 ms idle; 100/125 ms action delays | Low idle; optical + judge command while scanning | Blocking accept/reject pulse | `scanning`/alliance data races; no motor ownership lock | P0/P1 | Background task and autonomous/opcontrol can command same intake motors |
| Intake sort (big, compiled out) | `src/aon/intake.cpp:105` | `Intake::sort()` | 10 ms delay | Medium: hue every cycle, atomics, state machine, repeated motor APIs | Mutex wait/device calls/relative delay | Atomics plus `sortMotorMutex` | P1 | Nonblocking design is good; inactive states still poll hue and active `IDLE` resends commands |
| Intake scan (big, compiled out) | `src/aon/intake.cpp:82` | `Intake::scan()` | 50 ms plus 500 ms after detection | Low-medium | Blocking debounce, distance read | atomics plus unsynchronized `scanning` | P1 | Can race sorter/foreground commands to elevator |
| Shadow recorder | `src/aon/core/robot.cpp:33` + `src/aon/shadow/service.cpp:357` | recorder task / `Service::pollRecorder()` | 20 ms delay | Low when inactive; medium-high while recording (pose, 4 axes, atomic command, sample validation) | LemLib pose/controller calls, service mutex, stop/save I/O transition | One short status lock, then one sample lock | P1 | Timing quality directly determines replay fidelity; existing fixed buffers are good |
| Shadow playback dwell | `src/aon/shadow/player-pros.cpp:168` | dwell callback | 20 ms delay while dwelling | Low-medium | callback work, cancellation/device check | playback cancellation atomic | P2 | Fixed-period timing improves dwell/event alignment |
| GUI normal | `src/aon/tools/gui/gui.cpp:584` | `Gui::mainLoop()` | 100 ms delay | Low normally; medium/high on redraw or SD slot refresh | screen rendering, SD inspect, two status snapshots/string copies | autonomous/fallback/service mutexes | P2 | Correctly redraws mostly on changes; keep it below control-task importance |
| GUI debug (compiled out) | `src/aon/tools/gui/gui-debug.cpp:271` | `GuiDebug::mainLoop()` | 30 ms delay | Medium-high depending on active debug screen | screen drawing, graph/map callbacks | status mutex and shared debug data | P2 | Development-only workload should not enter competition mode |
| Safety task | `include/aon/globals.hpp:128` | `autonSafety()` | 50 ms normally; unbounded while X held | Low normally; extremely high when held | Inner busy loop repeatedly cancels/stops | motion-control/LemLib interactions | P0/P1 | Must yield while held and should not flood device commands |
| Orbit follow | `src/aon/orbit.cpp:65` | `Orbit::follow()` | 10 ms delay | Medium-high: vision + rotation + PID + motor | vision latency; blocking `rotateAbsolute()` branch | unsynchronized volatile flags; no motor mutex | P0/P1 | If launched, follow/scan/blocking rotate can compete for one Orbit motor |
| Orbit scan | `src/aon/orbit.cpp:117` | `Orbit::scan()` | 20 ms delay | Medium: vision + rotation + motor | vision latency; blocking `rotateAbsolute()` | same unsynchronized state/motor | P0/P1 | No production startup found, but native tests can exercise it |
| LemLib sensor-test display | `src/aon/lemlib/chassis.cpp:259` | `startSensorTest()` display task | 50 ms, terminal every 250 ms | High for diagnostic mode | LCD and formatted terminal output | library pose synchronization | P3 | Correctly isolated behind `LEMLIB_SENSOR_TEST`; do not optimize for competition |

## 4. PROS task and shared-resource map

`Robot::initialize()` starts five explicit first-party tasks in the normal build. LemLib and PROS/LVGL may create internal tasks not enumerable from the student source. The legacy odometry task is started later only when a native routine selector calls `legacy_motion::prepare()`.

| Task | Source | Approx. period | Shared resources | Motors controlled | Sensors accessed | Mutexes / atomics | Potential conflict | Risk |
|---|---|---|---|---|---|---|---|---|
| GUI | `src/aon/core/robot.cpp:28`; `Gui::mainLoop()` | 100 ms | selected auton, routine/fallback state, Shadow service, brain screen, SD summaries | none directly | touch screen | routine/fallback/service mutexes; string snapshot copies | SD/screen work can delay GUI only, but priority should remain below control | Medium |
| Autonomous safety | `src/aon/core/robot.cpp:30`; `autonSafety()` | 50 ms idle; no yield while X held | global motion services and mechanisms | both drivetrain abstractions, intake, Orbit | controller X | motion-control and LemLib internal sync | deliberately preempts every command owner; busy repeat can starve them | High |
| Intake scanning | `src/aon/core/robot.cpp:31`; `Intake::scan()` | 50 ms + conditional 500 ms | `scanning`, intake motor groups | corridor/elevator (small); elevator (big) | distance | small: none; big: release atomic only | conflicts with sort, opcontrol, autonomous mechanism calls | High |
| Intake sorting | `src/aon/core/robot.cpp:32`; `Intake::sort()` | 25 ms small / 10 ms big | scan/alliance/sort state | judge (small); elevator + judge (big) | optical; proximity in big release states | small: none; big: atomics + motor mutex | small variant has no explicit motor arbitration; big foreground calls can bypass sort mutex | High |
| Shadow recorder | `src/aon/core/robot.cpp:33`; `Service::pollRecorder()` | 20 ms | Shadow state/capture, effective drive, controller | none | LemLib pose, 4 controller axes | service mutex + recording/drive atomics | competes for pose/controller/device time; save transition can perform SD work | Medium |
| Competition opcontrol callback | `src/aon/core/robot.cpp:63` | 10 ms delay | driver globals, mechanisms, Shadow | LemLib drive; all mechanisms | controller, scorer motor position | Shadow atomics/service mutex while recording | commands same intake motors as background tasks; LemLib drive shares ports with legacy drive | High |
| Competition autonomous callback | `src/aon/core/robot.cpp:50` | routine-defined | action service or legacy drivetrain, mechanisms | drive and mechanisms | LemLib or legacy sensors | action/status/fallback locks | route selection must ensure one drivetrain owner | High |
| Legacy odometry (lazy) | `src/aon/drivetrain/legacy-motion.cpp:9`; `Odometry::initialize()` | 10 ms delay | native pose/model state | none | 3 rotations initialized; 2 sampled; IMU | position + orientation mutexes | duplicates LemLib tracking reads; debug can call `update()` concurrently | High |
| LemLib internal odometry/logger/motion | `firmware/LemLib.a` via `initializeChassis()` | not visible | LemLib chassis/pose | drive during motion/driver calls | 3 rotation + IMU | opaque | same physical ports as legacy wrappers | High / measure |
| Orbit follow/scan (test-started only) | `src/aon/orbit.cpp` | 10/20 ms | Orbit flags/PID | Orbit motor | vision + rotation | none | two loops and blocking rotate can command same motor | High if enabled |
| LemLib sensor display (test build only) | `src/aon/lemlib/chassis.cpp:269` | 50 ms | LCD/terminal, LemLib pose | none | pose | opaque LemLib pose sync | diagnostic I/O load | Low in competition |

### 4.1 Lock ordering and priority inversion

No nested acquisition of two first-party mutexes was found in the normal paths, so there is no demonstrated lock-order cycle. The main risks are instead:

- **Ignored acquisition failure:** all native odometry getters/setters use `take(1)` without checking success.
- **Unbounded waits:** Shadow's RAII `Lock` and most status/motion locks wait indefinitely. Their critical sections are generally short, but SD I/O is correctly moved outside the Shadow service lock in the inspected paths. Preserve that property.
- **Device calls under lock:** big-robot `commandSortMotors()` holds `sortMotorMutex` across motor API calls and rechecks atomics. This is defensible for ownership but should be timed; `faultAndStopSortMotors()` contains a fallback that sends stop commands without owning the mutex after a 10 ms timeout, intentionally favoring safety over serialization.
- **String copies under lock:** `routineStatus()` copies `std::string` while holding `statusMutex`; GUI calls it every 100 ms. Names likely fit small-string optimization, but measure wait time before redesigning.
- **Priority inversion:** explicit task priorities are not provided, so PROS defaults apply. If a high-priority controller is later introduced, it must not wait on a lower-priority task that holds a mutex across screen, SD, formatting, or sensor I/O.

## 5. Detailed odometry audit

### 5.1 Existing native architecture

`src/aon/odometry.cpp` implements the lazy-started legacy odometry used by native routes. LemLib maintains a separate pose for current LemLib routes and driver control.

One native iteration currently does this:

1. Read right and left rotation sensors (`update():148-149`). The back rotation read is commented out.
2. Convert cumulative angles to distance, then derive both angle and distance deltas.
3. Compute an encoder heading delta into a **local** `double deltaTheta` (`176`), shadowing the class member with the same name.
4. Read IMU heading, normalize it, and blend gyro/encoder heading. With current `GYRO_CONFIDENCE=1`, the encoder-heading calculation still executes but contributes zero.
5. Call `getRadians()` once, then call it again inside `setRadians(getRadians() + deltaTheta)` (`206-207`): three orientation lock operations total (two reads and one write).
6. Calculate the local arc displacement with one `sin` and one `cos` when turning.
7. Independently update `changeWeb` with another `sin` and `cos` (`245-249`). No production consumer of `changeWeb` was found; only `Odometry::debug()` displays it.
8. Publish global position through `SetPosition(...)`, but first call `getX()`, `getY()`, and `getRadians()` four times (`253-256`). That adds six lock operations (four orientation, two position) before the final position write.
9. Update previous encoder/gyro fields without a lock.

This is more synchronization and trig than the algorithm requires, but correctness is the first concern.

### 5.2 Confirmed correctness and coherence risks

- `getX()`, `getY()`, `getPosition()`, `SetPosition()`, `getDegrees()`, `setDegrees()`, `getRadians()`, and `setRadians()` call `Mutex::take(1)` and ignore its Boolean result. They then access state and call `give()` even if ownership was not obtained. Students should first confirm PROS mutex return semantics on target, then treat failed acquisition explicitly.
- X/Y share one mutex and orientation uses another. `getPose()` calls three getters, so a task switch between them can mix two odometry iterations. A pose used for path following, GUI, or a terminal condition should be one coherent sample.
- Publishing heading before position creates an interval where readers observe new heading with old position. Publishing position later through a different lock cannot make the tuple atomic.
- The unprotected class data (`encoder*_data`, `gyro_data`, `deltaDlocal`, `changeWeb`, and member `deltaTheta`) can race with `resetCurrent()` or `debug()` if those are invoked while the odometry task is running.
- `debug()` calls `update()` itself every 10 ms. If the legacy odometry task is also active, two writers can consume/update the same previous-sample fields, corrupting deltas.
- `resetCurrent()` reads all three rotation sensors and IMU, mutates filter/history fields, changes pose, tares the IMU, and blocks three seconds. There is no pause/handshake with the update task, so a reset during active native odometry can interleave with an iteration.
- `setDegrees()` writes member `deltaTheta = 0.0` after releasing the orientation mutex. In `update()`, the local `deltaTheta` shadows that member, so this reset does not affect the local delta used by the current algorithm. Students should determine whether the member is obsolete or intended state.

### 5.3 Duplicate and unnecessary work

- `encoder*_data.delta` is calculated but no consumer was found outside odometry state/debug; the algorithm uses `deltaDistance`.
- Back encoder state is initialized/reset, but the reads and calculations in `update()` are commented out.
- `gyro_data.currentRadians` is assigned but not subsequently used by `update()`.
- `gyro_data.prevDegrees` is assigned at `198` and again at `265`.
- `changeWeb` is a full second odometry integration performed every cycle only for comparison in `debug()`.
- Because `GYRO_CONFIDENCE` is the compile-time value 1 in both robot configurations, encoder-derived `deltaTheta` is dead in the current build after optimization only if the compiler can fully propagate the macro expression. Keep fallback capability if needed, but measure whether a competition-mode conditional can make the intended model explicit.
- The global transform calls `deltaDlocal.GetX()`/`GetY()` repeatedly. Whether those getters recalculate vector polar state should be checked; a local X/Y snapshot is clearer regardless.

### 5.4 Target architecture to investigate, not a replacement implementation

The proposed conceptual flow is the right one:

`read sensors once -> compute deltas -> select/fuse heading -> compute sin/cos once -> compute next pose locally -> publish one complete snapshot`

Students should prototype this behind tests with these invariants:

- One iteration has an immutable sensor sample and one timestamp.
- All intermediate calculations are local; no getter reacquires the object's own mutex.
- Heading convention and wrap behavior remain exactly documented and tested across 0/360 and ±180 boundaries.
- One publication critical section writes `{x, y, heading, timestamp/sequence}` together. Readers copy that whole snapshot under the same synchronization mechanism.
- Reset cannot interleave halfway through an update; use a clear reset/update ownership protocol.
- Debug comparison models are build-gated or run on copied samples outside the publisher's critical path.
- Failed sensor reads and failed lock acquisition have observable counters and defined behavior rather than silently publishing mixed data.

Benchmark existing and candidate models with recorded sensor traces before field deployment. Compare numerical output, maximum update time, deadline misses, and autonomous endpoint distribution—not just average loop time.

### 5.5 LemLib versus native odometry

`initializeChassis()` calibrates LemLib at every normal boot. Native selectors in `src/aon/auton/routine-selectors.cpp:21,86` separately call `legacy_motion::prepare()`, which creates the 10 ms legacy odometry task. The hardware map deliberately validates that legacy and LemLib tracking ports/reversal match (`src/aon/config/hardware-map.cpp:39-69`). Thus both models can read the same sensors concurrently after the first native route.

Do not delete either implementation during migration. Instead:

- Record which pose source each autonomous route and diagnostic uses.
- Establish a single drivetrain command owner for each competition mode/routine.
- Decide whether legacy odometry needs to keep running after leaving a native route; if not, investigate lifecycle control.
- In competition builds, exclude `changeWeb` comparison work and inactive debug paths while keeping their source available in development builds.
- Compare both models from the same timestamped sensor trace or controlled 20-run test, not from separately timed getter calls.

## 6. Deterministic loop timing

### 6.1 Relative delay versus fixed-period execution

With relative delay:

`work (for W ms) -> delay(10) -> work -> delay(10)`

the start-to-start period is approximately `W + 10 ms`. If work varies from 1 to 6 ms, the period varies from about 11 to 16 ms, and all subsequent starts drift. A fixed-period loop keeps a next-release deadline and sleeps only until it. Work that completes early still starts the next iteration on the planned boundary; work that finishes late increments a deadline-miss counter and advances to the next valid boundary rather than hiding overrun in a fresh full delay.

Use PROS's monotonic deadline API (`pros::delay_until()` where its signature and wrap semantics fit the installed kernel) with an integer millisecond wake anchor. Use `pros::micros()` around work for execution-time measurement. Do not busy-wait for microsecond alignment.

### 6.2 High-value deadline candidates

| Candidate | Current delay | Recommended intent | Why / caveat |
|---|---:|---:|---|
| `Odometry::initialize()` | 10 ms | 10 ms release grid | Highest-value native sampler; count overruns and do not run `debug()` as a second writer |
| `Robot::opcontrol()` | 10 ms | 10 ms release grid | Stabilizes input-to-command latency; avoid replaying an entire autonomous function inside this loop when `TESTING_AUTONOMOUS` is enabled |
| Shadow recorder task | 20 ms | exactly `kSamplePeriodMs=20` | File format and capacity assume 20 ms samples; deadline misses should be stored/reported |
| `Actions::runMonitored()` / `arcadeFor()` | 20 ms | 20 ms monitor grid | Stabilizes health observation; LemLib motion itself remains library-controlled |
| `EncoderMotionController::{driveDistance,turn}` | 20 ms | 20 ms controller grid | Profile/safety fallback depends on sample interval and settle time |
| Native PID/profile/pure-pursuit loops | 10/20 ms | match controller design dt | Their measured `dt` is good, but fixed releases reduce dt variance; remove loop printing first |
| Big intake sorter | 10 ms | 10–20 ms state-machine grid | Preserve state deadlines; poll optical only in decision states |
| Small intake scan/sort | 50/25 ms plus blocking | state machine with 20–50 ms grid | Converting delay API alone does not solve 100–500 ms blocking branches |
| Orbit follow/scan | 10/20 ms | 20 ms unless vision data rate proves benefit at 10 ms | Avoid polling faster than useful camera refresh; blocking rotate must first be removed from task loop design |
| GUI | 100 ms normal / 30 ms debug | relative delay is acceptable | Human-facing UI does not need hard periodicity; keep screen work low priority |
| Safety idle poll | 50 ms | relative delay acceptable | Event latency of 50 ms should be validated; the inner held loop must yield or act once per state change |

Long autonomous dwell delays (200 ms to 8 s), GUI initialization delays, short motor pulse helpers, and calibration waits do not become more correct merely by using `delay_until()`. They are sequencing operations, not periodic controllers. Their issue is blocking/cancellation policy, covered below.

## 7. Sensor polling audit and recommended rates

Rates are starting ranges to validate against device update rates and robot dynamics; they are not vendor guarantees.

| Sensor / telemetry | Observed use | Unused or redundant read | Intentional update-rate investigation |
|---|---|---|---|
| Native rotation sensors | `Odometry::update()` reads left/right every ~10+ ms; back is configured but not sampled | Back is read only during reset; `encoder*.delta` not consumed | 100 Hz for active odometry is reasonable; one read per sensor per iteration |
| Native IMU | `update()` reads heading every ~10+ ms | encoder heading still computed with confidence fixed at 1; reset reads heading then zeroes gyro struct | 50–100 Hz control/odometry; measure actual new-data rate and errors |
| LemLib tracking + IMU | library odometry is opaque; first-party `sampleDriveSensors()` reads 3 rotations, IMU rotation/status | Fallback linear drive uses only motor averages but receives full sample; monitor already has LemLib pose plus all raw sensors | Keep library defaults; for first-party health checks start at 50 Hz, then split minimal samples per decision if profiling supports it |
| Motor positions | `sampleDriveSensors()` calls `get_position_all()` for left and right, returning two vectors | Full monitor samples also read tracking sensors/IMU; small driver reads scorer position every 10+ ms | 50 Hz for motion health; 20–50 Hz for mechanism settled checks unless response tests require faster |
| Small intake distance | Only `scan()` while `scanning`, every ~50 ms; then no read for 500 ms | No idle read when disabled—good | 20–50 Hz is appropriate for block detection; replace fixed 500 ms blindness with timestamped debounce if cancellation matters |
| Small intake optical hue | Only `sort()` while `scanning`, every ~25 ms, except during 100/125 ms pulse | No idle read when disabled—good | 20–40 Hz; coordinate with distance/position so one block is not repeatedly acted upon |
| Big intake optical hue | `sort()` reads at top of every 10 ms iteration, before checking `requestedReleasing` or state | Unused while release inactive, `INIT`, `KICKBACK`, `WAIT_*`, `CONFIRM_*`, and `SETTLE` | Read only when `IDLE` needs a color decision; start 20–50 Hz |
| Big intake proximity | Read only in `WAIT_ACCEPT/CONFIRM_ACCEPT/WAIT_REJECT/CONFIRM_REJECT` | State-gated correctly | 50–100 Hz while waiting, zero otherwise; measure bounce and transport speed |
| Orbit vision | `follow()` 10 ms; `scan()` 20 ms; `getDistanceToRing()` 100 reads at 5 ms | `getDistanceToRing(Colors color)` overwrites its `color` parameter with current color, then sets color to itself; the parameter has no effect | Start at 20–50 Hz based on new-frame rate. Sampling a stale frame faster adds CPU/device traffic, not information |
| Orbit rotation | Each follow/scan/rotate iteration | `difference()` calls `getAngle()` up to three times; `groundDistanceToDisk()` calls `getHeight()` repeatedly (cheap) | 50–100 Hz while moving; one snapshot per calculation |
| GPS | `gpsPosition()` blocks 2 s, then reads once | The fixed wait occurs on every call, even if GPS is already ready | Read on demand after readiness/quality check; often 10–20 Hz is ample for supervisory correction, not the inner drivetrain loop |
| Controller | opcontrol reads active axes/buttons each ~10+ ms; Shadow recorder rereads all 4 axes every 20+ ms | During recording, axes already used by driver could be captured from one input snapshot | 50–100 Hz driver snapshot, shared read-only for drive/mechanisms/recording |

Important: PROS device APIs may return cached values, but calls still have software/locking overhead and coherence implications. Measure per-call latency on the V5 brain before restructuring a clear design.

## 8. Blocking operations and command architecture

| Function/group | Classification | Evidence and investigation |
|---|---|---|
| `Actions::moveToPoint/moveToPose/turnToHeading/followPath()` | Acceptable synchronous autonomous API, with caveat | `runMonitored()` yields every 20 ms, has timeout/cancel/health checks, and lets other tasks run. A command scheduler would help compose parallel mechanisms, but the blocking call itself is not CPU spinning |
| `Actions::arcadeFor()` and Shadow dwell | Acceptable | Both yield at 20 ms and check cancellation; use fixed release timing for accuracy |
| Encoder fallback controller | Acceptable/questionable | Yields and times out, but performs broad sensor reads and blocks the autonomous caller. Preserve fail-safe behavior; profile and consider commands only for composition |
| Native profile/path loops with `Timer` | Questionable | Most have timeout and yield, but loop printing and relative timing add jitter; they block the autonomous caller by design |
| `Drivetrain::drivePID()` | Harmful if enabled for competition | No timeout or cancellation path; a failed/frozen odometry condition can run forever. It prints three lines per 10 ms iteration |
| H-drive/Mecanum/X-drive `goToPose()` | Harmful if enabled | Source comments already note missing hard timeout; termination expression can run indefinitely and has no cancellation |
| `Orbit::rotateRelative()` / `rotateAbsolute()` | Harmful to concurrent Orbit execution | No timeout/cancellation; called from scan/follow task branches while manipulating the same motor and flags |
| `Orbit::getDistanceToRing()` | Questionable | Deliberately blocks ~500 ms for 100 samples; acceptable as an explicit diagnostic/query, harmful inside a control task. Parameter is currently ineffective |
| Small `Intake::scan()` / `sort()` branches | Harmful in background task | 500/100/125 ms delays prevent prompt state/cancel observation and leave motor ownership implicit. A timestamped nonblocking state machine is justified |
| `Intake::lever(timeout)` | Acceptable synchronous mechanism action | Yields every 5 ms and has a timeout; 5 ms motor-position polling may be faster than needed |
| `Intake::{pickUp,store,reject,score,kickBack}` with duration | Questionable | Convenient sequential autonomous primitives, but cancellation cannot interrupt their delays. Command/state ownership would enable safe parallel autonomous actions |
| `Odometry::resetCurrent()` / `gpsPosition()` | Questionable/harmful while tasks run | Fixed 3 s IMU tare and 2 s GPS wait block callers and can race native update. Restrict to an initialization/reset protocol |
| Long delays in native autonomous routines | Acceptable for early sequencing, questionable for reliability | They yield to other tasks, so they do not monopolize CPU, but fixed 3–8 s waits are open-loop and cancellation-insensitive within the helper. Prefer explicit timed mechanism commands with safety cancellation as architecture matures |
| GUI typewriter/startup delays | Acceptable | Isolated in GUI task during startup; no control-loop role, though the boot sequence itself waits 3 s before starting safety/intake/Shadow tasks |
| `autonSafety()` held-X loop | Harmful | No yield and repeats stop operations. Investigate edge-triggered stop plus a low-rate held-state watchdog |
| Infinite diagnostic loops in `native-tests.cpp` and `Odometry::debug()` | Acceptable only in development builds | Keep useful source, but make entry impossible in competition selection and never run debug as a second odometry writer |

A nonblocking command architecture has the highest payoff for intake/Orbit motor ownership and parallel autonomous mechanisms. It is less urgent for already timeout-bounded LemLib motion wrappers. Any command design should provide exclusive subsystem ownership, start/update/cancel/finish transitions, monotonic deadlines, and a stop-on-owner-loss rule.

## 9. Memory, allocation, and data-copy audit

### 9.1 Allocation classes

| Class | Sites | Assessment |
|---|---|---|
| A — initialization/process lifetime | `Hardware` constructs drivetrain odometry/profiles with `std::make_unique` (`src/aon/core/hardware.cpp:43-46,71-73`); GUI and function reader globals use `make_unique` (`src/aon/tools/gui/gui.cpp:93-100`); LemLib motor port vectors are function-local statics (`src/aon/lemlib/chassis.cpp:41-54`) | Safe, bounded, and outside control loops. Do not replace without measured startup/RAM benefit |
| A — lazy one-time | `legacy_motion::prepare()` allocates one PROS task; LemLib sensor-test allocates one display task | Acceptable. Confirm task-stack RAM and lifecycle, but not a recurring heap risk |
| A/B — registries and debug GUI | `std::map`, `std::vector`, `std::string`, and `std::function` in function/auton/debug registries | Registration/menu construction can allocate. Normal execution only copies selected callable/status. Keep debug registration out of competition where possible, not because STL is inherently bad |
| C — motion health hot loop | `sampleDriveSensors()` calls `MotorGroup::get_position_all()` twice and stores returned `std::vector<double>` (`src/aon/lemlib/chassis.cpp:193-209`) | P1 profiling target. Determine whether PROS allocates on each call and whether a nonallocating per-motor/sample API exists in installed PROS before changing it |
| C — native pure pursuit hot loop | `PurePursuit::follow(std::vector<Pose> path, ...)` (`include/aon/controls/pure-pursuit.hpp:98`) | P1 confirmed full path copy per controller iteration; likely heap allocation and O(N) copy, followed by O(N) closest-point scan. Change investigation should start with const-reference semantics and retained progress index, with tests |
| B/C — GUI status | `routineStatus()` copies a `RoutineStatus` containing `std::string`; normal GUI copies/assigns it every 100 ms | P2. Small-string optimization may avoid heap for current names; measure before replacing a clear API |
| B — routine dispatch | `FunctionReader::ExecuteFunction()` copies `std::function` while locked | Appropriate ownership: it releases the registry mutex before a long routine. One copy per dispatch is preferable to holding the mutex |
| B — Shadow processing/save | fixed arrays are copied/cleared/processed after recording; `captureSnapshot = recorder.capture()` occurs under service mutex | No heap fragmentation, but large deterministic memory bandwidth and lock hold. Measure save latency and mutex wait; avoid unsafe aliasing just to remove the copy |

No first-party `new`, `delete`, `malloc`, or `free` calls were found outside smart-pointer factories. No `std::unordered_map` was found. The most important allocation question is therefore returned vectors/path copies, not initialization-time smart pointers.

### 9.2 Static RAM pressure

Shadow deliberately trades heap risk for large static workspaces:

- `Storage` owns three 256 KiB `EncodedRecording` buffers: at least 768 KiB before decoded data.
- It also owns three `DecodedRecording` values, each containing a 3,000-sample capture and a route of up to 1,000 points/512 events.
- `ServiceStorage` additionally owns recorder capture, capture snapshot, route snapshot, processor workspace, and playback snapshot.
- `player-pros.cpp` owns a 64 KiB runtime path; `storage-pros.cpp` owns a 64 KiB directory buffer.

This can still be the correct safety design on V5: capacity is known and runtime allocation is avoided. But it is likely the dominant application BSS/RAM consumer. Use the linked ELF section report and map file to quantify `.bss`, then evaluate buffer lifetime overlap. Do not merge/reuse buffers until tests prove save, inspect, cancel, and playback cannot overlap unsafely.

### 9.3 Copying recommendations

- **High value:** pass the native pure-pursuit path by const reference and verify no lifetime escape; avoid starting the closest-point scan at zero after progress is monotonic. Measure separately because they change different behavior/performance properties.
- **High value:** take one coherent pose snapshot per control iteration and reuse it for termination, controller input, and diagnostics. Current native followers call pose getters repeatedly.
- **Medium:** investigate an allocation-free motor telemetry snapshot, but only if PROS offers a safe API and profiling confirms vector allocation.
- **Medium:** time the large Shadow capture copy under `serviceData.mutex`; a snapshot handoff/double-buffer design is only justified by observed GUI/recording stalls.
- **Low:** `Pose` is three doubles and safe by value in `PurePursuit::go/turn`; references would add aliasing complexity for tiny gains.
- **Low:** legacy `Vector` is larger and many operators take it by value. Optimize only in demonstrated hot geometry paths, not across the API wholesale.
- **Low:** `std::function` copies at autonomous dispatch are deliberately outside the periodic loop. Keep the safe ownership model.

## 10. Logging and GUI cost

### 10.1 Observed critical-loop logging

- `Drivetrain::follow()` prints three LCD lines plus one controller line every 10 ms (`include/aon/drivetrain/drivetrain.hpp:785-788`).
- Differential/H/Mecanum/X drive `goToPose()`/`follow()` repeat the same pattern in their 10 ms loops.
- `drivePID()` prints three LCD lines every 10 ms; profile loops print every 20 ms.
- Native diagnostic loops print multiple LCD lines every 5–20 ms.
- LemLib motion actions log start/finish and exceptional fallback transitions, not every 20 ms. This is a useful bounded pattern.
- Normal GUI polls at 100 ms and redraws main status only on changes. Shadow slot scans are rate-limited to 1 s. Those are good decisions; full-screen work still needs lower task priority than control.

Formatted floating-point output and controller/screen device calls have variable, often much larger latency than arithmetic. The first experiment should compile-gate or rate-limit native loop telemetry, while preserving source and adding loop statistics counters.

### 10.2 Proposed compile-time modes

| Mode | Keep | Suppress/rate-limit |
|---|---|---|
| `DEBUG` | all assertions, sensor screens, per-loop trace when explicitly selected, GUI tools, Shadow diagnostics | Still cap terminal/screen refresh (for example 5–10 Hz) so debugging does not accidentally change controller stability beyond recognition |
| `DEVELOPMENT` | motion start/finish, failure reasons, profiler summaries, deadline misses, sensor validity, autonomous steps, GUI status | Per-iteration pose/motor printing; debug map/graph work unless opened |
| `COMPETITION` | boot identity/config, selected routine, failures/cancellations, final autonomous metrics, compact deadline-miss summary | Success spam, per-loop floating-point output, sensor-test task, debug GUI tools, comparison odometry work |

The mode should remove runtime calls with preprocessor/`if constexpr` compile-time policy, not merely make a logger format and discard messages. Preserve source strings where useful; measure resulting code size separately.

## 11. Dead, duplicate, and debug-only runtime work

| Work | Status | Student investigation |
|---|---|---|
| Native `changeWeb` integration | Runs every native odometry cycle; only debug display consumes it | Exclude computation from competition mode, keep source/test comparison available |
| Native encoder heading with gyro confidence 1 | Calculated, then multiplied by zero in source expression | Confirm generated code removes it under `-Os`; make selected model explicit only if it improves clarity/measurement |
| Native `encoder*.delta`, `gyro.currentRadians`, duplicate `prevDegrees` assignment | No algorithm consumer found | Remove runtime assignments only after trace/tests confirm no diagnostic dependency |
| Back encoder native update | Configured and reset but update code is commented | Decide whether it is a required three-wheel model. Avoid paying reset/device ownership cost for an intentionally unused model, but correctness decision comes first |
| Repeated big-sort motor commands | `IDLE` calls `commandSortMotors(..., 2/3 speed, 0)` every 10 ms | Send on state/command change if motor API cost is measurable; retain watchdog refresh if hardware semantics require it |
| Big-sort hue in inactive/nondecision states | Read every 10 ms but only `IDLE` makes color decision | Gate by state and active release after validating detection latency |
| Orbit color parameter | `getDistanceToRing(Colors color)` overwrites the parameter and then sets current color to itself | Correct the contract before optimizing its 100-sample math; this is a correctness smell |
| Repeated pose/distance reads in native followers | Same iteration reconstructs pose for condition, controller, LCD, distance, controller print | One coherent local pose/distance snapshot; diagnostics consume that snapshot |
| Driver motor refresh | opcontrol sends drive and idle intake/motor position commands every ~10 ms | Usually acceptable watchdog behavior; only suppress unchanged commands after verifying PROS/V5 command semantics and safety |
| Debug GUI compilation | Debug `.cpp` files compile into normal build, but reachability and section GC determine final inclusion | Use map/`nm` to see what survives. Compile-time source exclusion may reduce build time even when linker removes runtime code |

## 12. Math optimization classification

| Site | Classification | Reason |
|---|---|---|
| Native odometry heading/global transform trig | **Cache result / simplify representation** | Same heading is passed to `sin/cos` multiple times. Compute a consistent angle and one sine/cosine pair per chosen model |
| `Vector::SetPosition()` in odometry | **High-value investigation** | Every call performs `hypot` and `atan2` to maintain polar fields (`include/aon/tools/vector.hpp:442-446`). Odometry invokes it for local delta, comparison model, and global position even where only X/Y are needed. A Cartesian pose/snapshot type may avoid hidden math and improve coherence |
| `PurePursuit::go()` `hypot`/`atan2` | **Already fine after structural fixes** | These are the controller's geometry. Remove path copy/logging/redundant snapshots before approximating math |
| Repeated `Pose::distanceTo()` in loop conditions/logs | **Cache result** | One `hypot` result per pose snapshot is enough |
| Profile `pow(value, 2.0)` | **Simplify algebra / P3** | Multiplication expresses square and may generate simpler code, but inspect compiler output first; it is not the leading loop cost |
| `sqrt` used to calculate a timeout | **Already fine** | Outside the loop; not timing-critical |
| Orbit `sqrt` distance geometry | **Already fine** | One calculation per requested sample; vision calls and 500 ms sequence dominate |
| Shadow `hypot`/`sqrt`/`remainder` processing | **Only optimize after profiling** | Runs after recording in bounded arrays, not in control loop; numerical clarity matters |
| GUI field mapper trig/sqrt | **Development-only** | Rate-limit rendering; do not complicate math for competition |
| Repeated degree/radian constants | **Precompute constants** | Use named `constexpr` conversion constants for clarity and compiler folding; expected runtime effect is minor |

The Cortex-A9 build uses hard-float and NEON (`-mfpu=neon-fp16 -mfloat-abi=hard`). Do not introduce fixed-point arithmetic unless on-brain profiling proves math is the bottleneck and error/repeatability tests justify the added risk.

## 13. Compiler optimization and code-size audit

### 13.1 Current build

`common.mk:4-6,37-42` currently supplies:

- `-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -mthumb`
- `-Os -g`
- `-ffunction-sections -fdata-sections`
- linker `--gc-sections`
- no `-flto`

`Makefile` leaves `EXTRA_CFLAGS` and `EXTRA_CXXFLAGS` empty and uses monolithic linking (`USE_PACKAGE=0`). `-g` explains much of the 13.8 MB ELF host file size; debug information is not the same as flash-resident code. The binary was 1,152,560 bytes in the existing stale artifact, but a fresh build/section report is required.

### 13.2 Where code size comes from

- **Application:** every first-party `.cpp` is compiled, including native tests and debug GUI. Reachable registrations/data may keep some code even when direct calls look absent.
- **PROS/kernel:** `firmware/libpros.a` provides RTOS, devices, controller, screen, SD, and competition runtime.
- **LemLib:** `firmware/LemLib.a` supplies chassis/odometry/motion. It is the only separate motion library listed in `project.pros` (version 0.5.6).
- **Okapi:** there is no Okapi archive in `firmware` or template metadata. `include/aon/compat/okapi.hpp` is a local compatibility facade over PROS types; count its instantiated application code, not an imagined Okapi static library.
- **LVGL:** `firmware/liblvgl.a` is linked through the PROS template. The code uses PROS screen APIs rather than direct LVGL calls, but library archive size does not equal linked contribution.
- **libc/libm/libstdc++:** formatted I/O, strings/containers, math, and C++ runtime pull reachable archive members.
- **Static Shadow buffers:** primarily `.bss` RAM rather than binary payload when zero-initialized; verify section placement.

`-ffunction-sections`/`-fdata-sections` put functions/data in independent input sections, and `--gc-sections` discards unreachable sections at link time. Therefore deleting comments, whitespace, Doxygen, or renaming variables cannot shrink the runtime binary. Even deleting unused source functions may make no binary difference if they were already unreachable; use a link map to prove contribution.

### 13.3 Benchmark matrix

Build every row from a clean tree with identical source/config and record the actual flags in the result file.

| Configuration | Binary/sections | Avg control-loop time | Max control-loop time | Autonomous runtime | Final position error | Stability |
|---|---:|---:|---:|---:|---|---|
| Current `-Os`, no LTO | baseline | 20-run metrics | 20-run max/p95 | mean/SD | X/Y/heading mean, SD, worst | crashes, stalls, deadline misses |
| `-O2`, no LTO | measure delta | same | same | same | same | same |
| `-Os` + LTO | measure delta | same | same | same | same | same |
| `-O2` + LTO | measure delta | same | same | same | same | same |

Do not default to `-O3`. `-O2` can increase code size but improve hot-loop maximum time; `-Os` may improve instruction-cache behavior; LTO may remove cross-translation-unit abstractions but adds toolchain/archive compatibility risk. Prebuilt PROS/LemLib/LVGL archives may limit whole-program LTO benefit. The winning row is the stable one with best worst-case timing and unchanged/improved endpoint distribution, not merely the smallest or fastest average.

For each experiment capture:

- clean build log and exact command flags;
- `arm-none-eabi-size` (or the PROS section-size output) for text/data/BSS;
- linker map plus largest symbols by section;
- `monolith.bin` size, not only unstripped ELF file size;
- task stack high-water marks or PROS task information if available;
- free-heap/free-memory API readings at boot, after GUI initialization, during recording, and after autonomous, if the installed PROS exposes a reliable API.

## 14. Concurrency conclusions

1. Treat `volatile` flags as P0 correctness work. Use synchronization whose memory semantics match one-writer/multi-reader state; first inventory every writer and preserve interrupt/device requirements.
2. Define one command owner per motor subsystem. The current small intake has background and foreground writers without a common lock; Orbit has multiple potential loop writers; drivetrain has LemLib and legacy wrappers over identical ports.
3. Preserve the good existing patterns: `MotionLease` serializes autonomous motion; function dispatch releases registry lock before executing; Shadow performs SD/process work outside its service lock; big-sort commands revalidate request ownership around motor calls.
4. Add lock-wait instrumentation before changing mutex types/priorities. Report count, total wait, maximum wait, and timeout/failure count by lock site.
5. Do not raise every control task priority. First remove busy looping and long critical sections; then establish a documented priority order: emergency stop/event handling, odometry/control, mechanism state machines/recorder, GUI/diagnostics.
