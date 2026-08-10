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
2. **P0 — confirmed cross-task state uses `volatile` instead of synchronization.** Intake `scanning` and hardware `alliance` have demonstrated writers and background-task readers; `volatile` does not make those accesses thread-safe in C++. Intake `scoreDown` has no reader/caller at the audited commit, and Orbit has no production startup, so those fields are dormant/conditional hazards rather than confirmed active races. Inventory their owners before enabling those paths.
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

“Source-selected target” below means the period implied by a loop constant or comment; no separate control-rate requirements document was found. “Not stated” means static inspection cannot establish design intent. With `pros::delay(N)`, actual start-to-start period is `work + N ms` and can be much longer after blocking branches.

| Subsystem | File | Function | Intended period | Actual delay mechanism / achievable rate | Likely CPU/device cost | Possible jitter source | Synchronization cost | Priority | Reason |
|---|---|---|---|---|---|---|---|---|---|
| Driver control | `src/aon/core/robot.cpp:63` + `include/aon/competition/operator-control.hpp:314` | `Robot::opcontrol()` / `Run()` | 10 ms source-selected target | Relative `pros::delay(10)`; less than 100 Hz start-to-start | Medium: controller reads, LemLib drive call, motor telemetry for lever, Shadow atomics/events | Controller/device calls, mechanism branches, Shadow mutex while recording | Shadow event capture takes service mutex only when recording | P1 | Primary driver latency path; relative scheduling and repeated unchanged motor commands |
| LemLib odometry/motion | `src/aon/lemlib/chassis.cpp:146` | `chassis()` / library internals | Not stated in first-party source | Opaque in static LemLib 0.5.6 archive | Likely medium-high | Library sensor/motion task scheduling, device calls | Library-internal, not visible here | P1 | Governs current driver/autonomous pose and motion; instrument externally rather than guess |
| Autonomous monitor | `src/aon/auton/actions.cpp:179` | `runMonitored()` | 20 ms source-selected target | Relative `pros::delay(20)` while LemLib reports motion; less than 50 Hz | High: pose + two motor vectors + 3 rotation + IMU/status per iteration | Device calls, `std::function`, monitor work, logging on transitions | `MotionControl` mutex calls; LemLib internals | P1 | Safety-critical supervisor samples far more telemetry than ordinary controller loops |
| Timed autonomous drive | `src/aon/auton/actions.cpp:373` | `Actions::arcadeFor()` | 20 ms source-selected target | Relative `pros::delay(20)`; less than 50 Hz | Medium-high: drive telemetry or full motion sample | Same sensor/device latency, relative delay | Motion-control mutex | P1 | Deadline and duration accuracy affect autonomous behavior |
| Encoder fallback drive | `src/aon/auton/encoder-motion.cpp:46` | `driveDistance()` | 20 ms source-selected target | Relative `pros::delay(20)`; less than 50 Hz | High: both motor position vectors, all 3 trackers, IMU and status even though drive uses motor averages | Device calls, motor commands, relative delay | Motion-control calls | P1 | Safety fallback should be predictable; sensor sampling is broader than each decision needs |
| Encoder fallback turn | `src/aon/auton/encoder-motion.cpp:94` | `turn()` | 20 ms source-selected target | Relative `pros::delay(20)`; less than 50 Hz | High: same full `sampleDriveSensors()` set | Device calls, motor commands, relative delay | Motion-control calls | P1 | Same as fallback drive; IMU/encoder switchover must remain reliable |
| Legacy native odometry | `src/aon/odometry.cpp:124` | `Odometry::initialize()` / `update()` | 10 ms source-selected target | Relative `pros::delay(10)`; less than 100 Hz; starts lazily | High: 2 rotations + IMU, repeated locks/getters, 6 active trig calls, duplicate model | Sensor reads, mutex timeouts, scheduling drift | Multiple position/orientation lock operations per update | P0 | Incoherent publishing and lock misuse; duplicate calculations are secondary |
| Legacy pure pursuit | `include/aon/drivetrain/drivetrain.hpp:764` | `Drivetrain::follow()` | 10 ms source-selected target | Relative `pros::delay(10)`; less than 100 Hz | Very high: path copied inside controller, repeated poses, trig/hypot, four formatted displays, motor calls | LCD/controller I/O, pose locks, path scan, relative delay | Multiple odometry locks | P1 | Debug I/O can dominate the 10 ms control budget |
| Derived legacy pose/path | `src/aon/drivetrain/{differential-drive,h-drive,mecanum,x-drive}.cpp` | `goToPose()` / `follow()` | 10 ms source-selected or caller-selected target | Relative delay; start-to-start exceeds selected period | High for same reasons as base path follower | Formatted display, pose reads, relative delay | Multiple odometry locks | P1 | Same hot-loop pattern appears in multiple implementations |
| Legacy profile/PID motion | `include/aon/drivetrain/drivetrain.hpp:312-517,611` | `drivePID()`, `turnPID()`, profiled moves, `driveAngleOfArc()` | 10 or 20 ms source-selected targets | Relative delays; less than 100 or 50 Hz respectively | Medium-high: sensors, profile/PID math, motor calls | Device latency, variable work, timeout checks | Odometry reads where used | P1 | Blocking autonomous controllers need stable sample time |
| Intake scan (small, active build) | `src/aon/intake.cpp:374` | `Intake::scan()` | 50 ms idle target; detection-hold intent not documented | Relative 50 ms idle delay plus 500 ms blocking pause after detection | Low idle; device + motors when active | 500 ms debounce block, distance read | `scanning` is an unsynchronized volatile | P1 | Cancellation/state response can be delayed by half a second |
| Intake sort (small, active build) | `src/aon/intake.cpp:398` | `Intake::sort()` | 25 ms idle target; pulse-duration intent not documented | Relative 25 ms idle delay plus blocking 100/125 ms action delays | Low idle; optical + judge command while scanning | Blocking accept/reject pulse | `scanning`/alliance data races; no motor ownership lock | P0/P1 | Background task and autonomous/opcontrol can command same intake motors |
| Intake sort (big, compiled out) | `src/aon/intake.cpp:105` | `Intake::sort()` | 10 ms source-selected target | Relative `pros::delay(10)`; less than 100 Hz | Medium: hue every cycle, atomics, state machine, repeated motor APIs | Mutex wait/device calls/relative delay | Atomics plus `sortMotorMutex` | P1 | Nonblocking design is good; inactive states still poll hue and active `IDLE` resends commands |
| Intake scan (big, compiled out) | `src/aon/intake.cpp:82` | `Intake::scan()` | 50 ms idle target; detection-hold intent not documented | Relative 50 ms delay plus 500 ms blocking pause after detection | Low-medium | Blocking debounce, distance read | atomics plus unsynchronized `scanning` | P1 | Can race sorter/foreground commands to elevator |
| Shadow recorder | `src/aon/core/robot.cpp:33` + `src/aon/shadow/service.cpp:357` | recorder task / `Service::pollRecorder()` | 20 ms source-selected capture target | Relative `pros::delay(20)`; less than 50 Hz | Low when inactive; medium-high while recording (pose, 4 axes, atomic command, sample validation) | LemLib pose/controller calls, service mutex, stop/save I/O transition | One short status lock, then one sample lock | P1 | Timing quality directly determines replay fidelity; existing fixed buffers are good |
| Shadow playback dwell | `src/aon/shadow/player-pros.cpp:168` | dwell callback | 20 ms source-selected target | Relative `pros::delay(20)` while dwelling; less than 50 Hz | Low-medium | callback work, cancellation/device check | playback cancellation atomic | P2 | Fixed-period timing improves dwell/event alignment |
| GUI normal | `src/aon/tools/gui/gui.cpp:584` | `Gui::mainLoop()` | 100 ms source-selected UI target | Relative `pros::delay(100)`; less than 10 Hz | Low normally; medium/high on redraw or SD slot refresh | screen rendering, SD inspect, two status snapshots/string copies | autonomous/fallback/service mutexes | P2 | Correctly redraws mostly on changes; keep it below control-task importance |
| GUI debug (compiled out) | `src/aon/tools/gui/gui-debug.cpp:271` | `GuiDebug::mainLoop()` | 30 ms source-selected UI target | Relative `pros::delay(30)`; less than 33.3 Hz | Medium-high depending on active debug screen | screen drawing, graph/map callbacks | status mutex and shared debug data | P2 | Development-only workload should not enter competition mode |
| Safety task | `include/aon/globals.hpp:128` | `autonSafety()` | 50 ms idle check target; held-state intent not stated | Relative 50 ms normally; unbounded inner loop while X is held | Low normally; extremely high when held | Inner busy loop repeatedly cancels/stops | motion-control/LemLib interactions | P0/P1 | Must yield while held and should not flood device commands |
| Orbit follow | `src/aon/orbit.cpp:65` | `Orbit::follow()` | 10 ms source-selected target if started | Relative `pros::delay(10)` except blocking rotation branch; less than 100 Hz | Medium-high: vision + rotation + PID + motor | vision latency; blocking `rotateAbsolute()` branch | dormant volatile flags; no motor mutex | P0/P1 conditional | If concurrently launched, follow/scan/blocking rotate can compete for one Orbit motor |
| Orbit scan | `src/aon/orbit.cpp:117` | `Orbit::scan()` | 20 ms source-selected target if started | Relative `pros::delay(20)` except blocking rotation; less than 50 Hz | Medium: vision + rotation + motor | vision latency; blocking `rotateAbsolute()` | same dormant state/motor hazard | P0/P1 conditional | No production startup found, but native tests can exercise it |
| LemLib sensor-test display | `src/aon/lemlib/chassis.cpp:259` | `startSensorTest()` display task | 50 ms display target / 250 ms terminal target | Relative `pros::delay(50)` and timestamp-gated terminal output | High for diagnostic mode | LCD and formatted terminal output | library pose synchronization | P3 | Correctly isolated behind `LEMLIB_SENSOR_TEST`; do not optimize for competition |

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
| Big-intake `Intake::stopReleasingAndWait()` | Acceptable bounded synchronization boundary, with caveat | `src/aon/intake.cpp:304-316` polls the release-request acknowledgement at the 10 ms sorter period and fails safe after 250 ms. It yields and is bounded, but foreground callers can still pause for a quarter second; record timeouts and prove the sorter always acknowledges under load |
| `stopAndSettle()` fallback handoff | Acceptable safety boundary, with caveat | `src/aon/auton/actions.cpp:133-154` deliberately samples, waits half the configured settle period, samples again, waits the remainder, then verifies motion stopped. The two waits yield and support safe fallback transfer, but synchronously delay the autonomous caller; measure total handoff latency and retain the fail-closed sensor checks |
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

- `Storage` owns three 256 KiB `EncodedRecording` buffers: at least 768 KiB before decoded data (`include/aon/shadow/storage.hpp:55-60`; capacity at `include/aon/shadow/codec.hpp:11-20`).
- It also owns three `DecodedRecording` values, each containing a capture and processed route with capacities defined by the Shadow types (`include/aon/shadow/storage.hpp:58-60`; `include/aon/shadow/codec.hpp:17-20`).
- `ServiceStorage` additionally owns recorder capture, capture snapshot, route snapshot, processor workspace, and playback snapshot (`src/aon/shadow/service.cpp:43-59`).
- The PROS player owns a 64 KiB runtime path (`include/aon/shadow/player-pros.hpp:11-15`); the SD implementation owns a 64 KiB static directory buffer (`src/aon/shadow/storage-pros.cpp:35`).

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

`Makefile` leaves `EXTRA_CFLAGS` and `EXTRA_CXXFLAGS` empty and uses monolithic linking (`USE_PACKAGE=0`). The existing ELF host file is 13.8 MB and its binary is 1,152,560 bytes, but both are stale artifacts. Debug information is one plausible contributor to the ELF-versus-binary difference, not a measured attribution. A fresh build, section report, and map are required before assigning bytes to any component.

### 13.2 Code-size contributors and current attribution limit

Actual retained-byte attribution could not be determined from this checkout: the available artifacts predate the audit base, no linker map is present, and the PROS/ARM build tools are not discoverable in the current environment. The following is therefore a link-input inventory, not a quantitative result. Do not use it to claim that any named component dominates flash. Phase 0 must generate a fresh map, section sizes, and per-symbol totals before a size change is proposed.

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

1. Treat demonstrated cross-task `volatile` access (`scanning` and `alliance`) as P0 correctness work. Inventory every writer/reader first; keep dormant `scoreDown` and Orbit fields as conditional hazards until those paths acquire concurrent owners. Use synchronization whose memory semantics match the proven ownership model and preserve interrupt/device requirements.
2. Define one command owner per motor subsystem. The current small intake has background and foreground writers without a common lock; Orbit has multiple potential loop writers; drivetrain has LemLib and legacy wrappers over identical ports.
3. Preserve the good existing patterns: `MotionLease` serializes autonomous motion; function dispatch releases registry lock before executing; Shadow performs SD/process work outside its service lock; big-sort commands revalidate request ownership around motor calls.
4. Add lock-wait instrumentation before changing mutex types/priorities. Report count, total wait, maximum wait, and timeout/failure count by lock site.
5. Do not raise every control task priority. First remove busy looping and long critical sections; then establish a documented priority order: emergency stop/event handling, odometry/control, mechanism state machines/recorder, GUI/diagnostics.

## 15. Lightweight profiling plan

### 15.1 Design goals

Instrumentation must be bounded, allocation-free in periodic loops, silent during a measured run, separately owned by each task, and removable or reducible at compile time. Use `pros::micros()` as the monotonic fine-grained clock. Store timestamps as `std::uint32_t` and calculate elapsed time with unsigned subtraction so normal wraparound remains defined. Confirm the installed PROS API and do not convert absolute microsecond timestamps to floating point.

Each loop owns a small aggregate record conceptually containing:

- iteration count;
- sum of execution microseconds in `std::uint64_t`;
- minimum execution time and maximum execution time in microseconds;
- previous iteration start;
- sum/min/max actual start-to-start period;
- maximum positive and negative period error from target;
- execution deadline misses (`execution_us > budget_us`);
- release deadline misses (iteration starts after its planned release tolerance);
- optional overrun buckets, e.g. `<25%`, `25–50%`, `50–75%`, `75–100%`, `>100%` of budget;
- sensor/lock/error counters relevant to that loop.

Do not update a shared map or print from the hot loop. Use one stat block per statically known loop. If the GUI must read a 64-bit stat while its task writes on 32-bit hardware, snapshot only after stopping the run, use a short explicit synchronization protocol, or use a double-buffered publish—not an unsynchronized read that can tear.

### 15.2 Per-iteration measurement procedure

At loop entry:

1. Read `start_us = pros::micros()` once.
2. If not the first iteration, calculate `period_us = start_us - previous_start_us` and update period aggregates.
3. Compare the start to the planned release for deadline/jitter accounting.

After all work and immediately before sleeping:

1. Read `end_us = pros::micros()` once.
2. Calculate `execution_us = end_us - start_us`.
3. Update count/sum/min/max/buckets and mark execution overrun against the configured budget.
4. Sleep to the next fixed release. If late, record the miss and advance the release anchor without busy-waiting.

Two clock reads, integer subtracts, comparisons, and aggregate stores are the base overhead. Measure that overhead with an empty instrumented loop and report it. Do not call `sqrt`, format strings, allocate, or compute percentiles in the control task.

### 15.3 Metrics and definitions

| Metric | Definition / collection |
|---|---|
| Execution time | `end_us - start_us`, excluding sleep; report count, min, mean (`sum/count`), max |
| Loop period | current iteration start minus previous start; report min/mean/max |
| Jitter | signed `actual_period_us - target_period_us`; retain worst early, worst late, and mean absolute value in offline analysis |
| Deadline miss | execution exceeds budget, or a scheduled release starts later than an agreed tolerance; report both definitions separately |
| Mutex wait | timestamp immediately before and after selected `take()`; record count/sum/max/timeouts by mutex/site, not just by mutex object |
| Autonomous time | timestamp immediately before routine dispatch and immediately after full stop/final status |
| Final error | after mechanisms and drivetrain settle, take one coherent LemLib/native pose; compute final X error and final Y error as `final - target`, and wrap final heading error with `remainder(...,360)` |
| Binary size | clean build's `.text`, `.rodata`, `.data`, `.bss`, total flash image and `monolith.bin` bytes |
| RAM | linked `.data + .bss`, task stack allocation/high-water if available, and reliable runtime free-memory samples |

Use a budget lower than the period to retain headroom. Example starting gates to validate: 10 ms loop budget 7.5 ms; 20 ms loop budget 15 ms; 50 ms task budget 30 ms. These are test thresholds, not claims about current performance.

### 15.4 Where to instrument first

1. Native `Odometry::update()` and full native odometry iteration.
2. `Robot::opcontrol()` around the entire driver iteration.
3. `Actions::runMonitored()` around `motionSample()`, monitor observation, and `onPoll` separately.
4. `sampleDriveSensors()`, with sub-timers for each motor vector, three rotation reads, and IMU/status.
5. Encoder fallback drive/turn iterations.
6. Shadow `pollRecorder()` inactive and recording modes separately.
7. Active robot's intake scan/sort iterations and blocked-state latency.
8. Native pure pursuit before/after path-copy and logging experiments.
9. Selected mutex sites: native pose publication/read, Shadow service, motion control, big-sort motor ownership, auton status.
10. GUI only after control paths, measuring redraw and SD slot inspection separately.

### 15.5 Reporting without perturbing control

- Reset stat blocks before enable/autonomous, freeze them after disable or routine completion, then print/save summaries.
- In `DEVELOPMENT`, optionally retain a fixed ring of a small number of anomalous samples (timestamp, duration, loop ID, state) only when a threshold is crossed.
- In `COMPETITION`, retain counters/maxima and one final compact summary; no per-iteration logging.
- Include a profiler schema version, git commit, robot identity, build flags, route, battery voltage, starting pose, and test-run ID with every dataset.
- To estimate profiler observer effect, run A/B with instrumentation compiled out versus aggregate-only instrumentation. Reject instrumentation whose overhead materially changes maximum time or endpoint distribution.

## 16. Performance regression and autonomous repeatability protocol

### 16.1 Controlled setup

For every candidate optimization and compiler configuration:

1. Pin commit, compiler flags, PROS/LemLib versions, robot identity, autonomous route, field layout, payload/block arrangement, tire/wheel condition, and controller mode.
2. Charge or constrain battery state to an agreed range; record start/end voltage and motor/brain temperature when practical.
3. Use a physical jig/field references to reproduce starting X, Y, and heading. Record the commanded starting pose and an independent measured start offset.
4. Warm up consistently, then run **at least 20 autonomous trials**. Interleave baseline and candidate runs (A/B/A/B) when battery/temperature drift is significant rather than running all baseline trials first.
5. After each run, wait for the same settle interval and capture coherent final X, Y, heading, total duration, per-loop aggregates, deadline misses, task-stall events, motion failure reason, and any operator intervention.
6. Preserve failed runs; do not silently discard outliers. Mark the physical cause if a trial is invalidated.

### 16.2 Required calculations

For X error, Y error, wrapped heading error, autonomous duration, loop mean, loop maximum, and deadline misses calculate:

- arithmetic mean;
- sample standard deviation;
- worst case in the unsafe direction (and absolute worst error);
- median and 95th percentile where the sample count/distribution makes it useful;
- success/failure count and unexpected task stalls.

With only 20 runs, the empirical 95th percentile is effectively near the worst observed run and is noisy. Report the raw sorted values or the percentile method; do not imply population-level confidence from one number.

### 16.3 Acceptance and rollback gates

An optimization is acceptable only when:

- no new correctness failure, motor ownership conflict, sensor invalid state, or task stall appears;
- deadline misses and worst-case loop time improve or remain within the predeclared budget;
- mean autonomous duration does not hide worse maximum duration;
- X/Y/heading mean, standard deviation, worst case, and success rate remain equal or improve within predeclared engineering tolerances;
- instrumentation mode and logging mode are identical between compared builds;
- binary/RAM growth remains within measured headroom.

Rollback if timing improves while autonomous repeatability, failure rate, cancellation response, thermal behavior, or worst-case endpoint error degrades. Never trade a narrower average loop time for a fatter error tail.

### 16.4 Suggested data record

One CSV row per run should include:

`schema, commit, build_flags, robot, route, run, battery_start, battery_end, temperature, start_x, start_y, start_heading, target_x, target_y, target_heading, final_x, final_y, final_heading, x_error, y_error, heading_error, auton_ms, loop_count, loop_mean_us, loop_max_us, period_mean_us, period_max_us, deadline_misses, mutex_wait_max_us, stall_count, result, failure_reason, notes`

Keep raw data and analysis script in version control. Do not report only screenshots or hand-copied averages.

## 17. Ranked optimization backlog

| Priority | Subsystem | File / function | Observed issue | Why it matters | How to measure | Recommended student investigation | Expected impact | Change risk |
|---|---|---|---|---|---|---|---|---|
| P0 | Native odometry | `src/aon/odometry.cpp`; getters, setters, `getPose()`, `update()` | timed mutex results ignored; X/Y/heading published/read separately | undefined ownership behavior and incoherent controller input | lock failures/wait; sensor-trace equivalence; 20-run endpoint spread | design one checked lock policy and complete pose snapshot; serialize reset/update | Very high reliability/determinism | High: pose semantics |
| P0 | Cross-task state | `include/aon/intake/intake.hpp`, `include/aon/core/hardware.hpp`; conditional inventory in `include/aon/orbit/orbit.hpp` | demonstrated `scanning`/`alliance` cross-task access uses `volatile`; `scoreDown`/Orbit are dormant at the audited commit | active races can produce stale/undefined state; dormant fields become hazardous if enabled without ownership | transition/cancel latency, writer/reader inventory, race-focused tests | synchronize demonstrated shared state; document or remove dormant state only after proving its intended owner | High reliability | Medium |
| P0 | Motor ownership | intake/Orbit/drivetrain call sites | multiple tasks/abstractions can command same motors | last-writer-wins nondeterminism and unsafe cancellation | command-owner trace; conflict counter | exclusive owner/command state machine; prove route selection prevents LemLib/legacy overlap | Very high | High |
| P0 | Unbounded controllers | `Drivetrain::drivePID()`, H/Mecanum/X `goToPose()`, Orbit rotate methods | loops lack timeout/cancel | sensor/mechanism failure can hang autonomous/task forever | fault injection, timeout tests | add architectural timeout/cancel requirement before competition enablement | Very high safety | Medium |
| P1 | Safety task | `include/aon/globals.hpp:128` `autonSafety()` | no yield while X held; repeated STOP | CPU starvation/device flood during fault | held-X loop time, command count, cancellation latency | edge-trigger stop plus bounded watchdog/yield | High | Low-medium |
| P1 | Native path following | `pure-pursuit.hpp:98`; native follow/goToPose loops | path copied/scanned and pose/diagnostics repeated every iteration | heap/O(N), jitter, incoherent reads | allocations, exec max, path size scaling | const path view; retain safe progress index; one pose/distance snapshot | High | Medium: path behavior |
| P1 | Loop telemetry | native drivetrain loops | formatted LCD/controller writes at 10/20 ms | variable I/O can dominate budget | loop max with logging on/off | compile-time levels and rate-limited snapshot diagnostics | High | Low |
| P1 | Periodic scheduling | odom/opcontrol/Shadow/actions/fallback/native control | relative delays add work time and drift | variable sample/control timing | period/jitter/deadline misses | deadline-based release grid with overrun accounting | High determinism | Medium |
| P1 | Small intake | `src/aon/intake.cpp:374,398` | task blocks 100–500 ms; no common motor lock | slow cancellation and writer conflicts | event-to-response latency, conflict traces | timestamped state machine and subsystem ownership | High | Medium-high |
| P1 | Big intake | `src/aon/intake.cpp:105` | hue read outside decision state; repeated motor commands | needless device traffic/jitter | per-state exec/sensor counts | state-gated polling and command-on-change, preserving watchdog semantics | Moderate-high | Medium |
| P1 | Motion telemetry | `src/aon/lemlib/chassis.cpp:190` `sampleDriveSensors()` | two returned vectors and all sensors every sample | possible hot-loop allocation/device load | per-sub-read time, allocation counter | decision-specific sample plan or safe nonallocating API | High if confirmed | Medium-high |
| P1 | Duplicate localization | LemLib chassis + `legacy_motion::prepare()` | both pose stacks can remain active | duplicate sensor/CPU work and ownership ambiguity | task CPU, sensor counts, route mode trace | explicit lifecycle/source-of-truth during migration | High | High |
| P1 | Shadow sampling | recorder task / `pollRecorder()` | relative 20 ms schedule | replay format/fidelity depends on sample timing | sample period jitter/gaps | fixed release and missed-sample metadata | Moderate-high | Low-medium |
| P2 | Odometry math | `odometry.cpp` + `Vector::SetPosition()` | repeated trig plus hidden hypot/atan2, debug comparison model | likely native hot-loop cost | sub-timers / generated assembly | local Cartesian next pose; one trig pair; build-gate comparison model | Moderate-high | High: numerical behavior |
| P2 | Shadow RAM/copies | Shadow storage/service structures | ≥768 KiB encoded scratch plus decoded/snapshot workspaces; large copy under lock | RAM headroom and save latency | ELF BSS/map, copy/lock/save timing | lifetime-overlap map and safe buffer reuse only if needed | Moderate | High |
| P2 | GUI/status | normal GUI and auton status | string snapshots and redraw/SD work | lower-priority contention, possible allocation | GUI iteration/redraw/lock timing | keep change-driven redraw; fixed-size status only if measurements justify | Low-moderate | Medium |
| P2 | Logging modes | logging/status/action modules | no unified compile-time policy | inconsistent observer effect and code reachability | binary size and loop time by mode | `DEBUG`/`DEVELOPMENT`/`COMPETITION` policy | Moderate | Low |
| P2 | Compiler | `common.mk`, Makefile experiment only | `-Os`, no LTO is unbenchmarked against alternatives | potential timing/size gain | matrix in §13/§16 | clean A/B `-Os`/`-O2`/LTO builds | Unknown/moderate | Medium |
| P3 | Minor math/copies | profile square, tiny Pose/Vector helpers | small algebra/tiny-value copies | likely below device/I/O cost | profile/assembly | simplify only after P0-P2 evidence | Low | Low-medium |

No item should be implemented solely because it appears in this table. Each row names the measurement needed to promote a hypothesis into work.

## 18. Final phased roadmap

### Phase 0 — Establish the baseline

- **Measure before:** fresh `-Os` section sizes, task list/stacks, boot time, battery/temperature, 20 autonomous runs, all current loop/endpoint metrics possible without source instrumentation.
- **Inspect:** selected robot/routine, physical start repeatability, LemLib/native route ownership, failure logging, test data capture.
- **Improvement:** a reproducible baseline dataset with no missing/hand-edited trials and documented environmental bounds.
- **Rollback:** not applicable; invalidate and repeat any baseline whose setup or build identity is ambiguous.

### Phase 1 — Profiling instrumentation

- **Measure before:** empty clock-read/aggregate overhead and uninstrumented A/B control runs.
- **Inspect:** clock wrap handling, one-writer stat ownership, output timing, profiler compile modes.
- **Improvement:** execution/period/min/mean/max/deadline-miss data with negligible measured observer effect and no hot-loop allocation/printing.
- **Rollback:** instrumentation changes control-loop maxima, endpoint distribution, or introduces synchronization stalls.

### Phase 2 — Odometry hot-path correctness and analysis

- **Measure before:** native update timing, lock waits/failures, trace output versus LemLib, pose coherence sequence, 20 native-route endpoints.
- **Inspect:** unchecked timed locks, split snapshot, reset/update race, variable shadowing, `changeWeb`, `Vector::SetPosition()` hidden math, sensor validity.
- **Improvement:** checked synchronization, one coherent pose publication, trace-equivalent or intentionally documented math, reduced max time/misses with no endpoint regression.
- **Rollback:** changed heading convention/wrap, new pose jumps, worse X/Y/heading spread, reset failures, or lock-related stalls.

### Phase 3 — Task scheduling and loop determinism

- **Measure before:** start-to-start periods/jitter/misses for odometry, opcontrol, actions, fallback, Shadow, intake.
- **Inspect:** every relative periodic delay, task priority, safety busy loop, overrun policy.
- **Improvement:** narrower period distribution, fewer deadline misses, bounded safety CPU, same or better command response.
- **Rollback:** task starvation, missed safety input, increased control maximum, or changed autonomous endpoint tail.

### Phase 4 — Sensor polling reduction

- **Measure before:** sensor calls per state/second and latency per API call; detection/cancellation latency.
- **Inspect:** big-sort hue states, `sampleDriveSensors()` breadth/vector allocation, controller snapshot duplication, Orbit frame freshness, GPS waits.
- **Improvement:** fewer unused reads and lower maximum loop time with unchanged detection/fault response.
- **Rollback:** missed blocks, delayed reject/accept, false odometry failures, worse fallback behavior, or stale controller capture.

### Phase 5 — Concurrency and locking

- **Measure before:** per-site wait/max/timeouts, motor-owner trace, cancellation latency, race/fault tests.
- **Inspect:** volatile fields, small intake ownership, Orbit writers, LemLib/legacy drive selection, Shadow copy under lock, priority order.
- **Improvement:** explicit single owner per subsystem, defined state synchronization, no unchecked lock failure, shorter bounded waits.
- **Rollback:** deadlock, priority inversion, delayed emergency stop, lost mechanism command, or inability to recover after cancellation.

### Phase 6 — Blocking-operation audit

- **Measure before:** duration and cancellation response for every synchronous command; fault injection for frozen sensors/targets.
- **Inspect:** unbounded native/Orbit loops, small intake pauses, GPS/IMU reset waits, long mechanism helpers and autonomous dwells.
- **Improvement:** every competition-reachable wait has yield, timeout, cancellation, and safe motor-stop semantics; command state machines enable needed concurrency.
- **Rollback:** premature motion termination, unsafe motor continuation, sequencing race, or autonomous repeatability loss.

### Phase 7 — Logging and GUI overhead

- **Measure before:** loop maxima and binary size with current logging; GUI redraw/SD latency.
- **Inspect:** native per-loop prints, action/status boundary logs, debug GUI reachability, Shadow menu polling.
- **Improvement:** competition loops emit no per-iteration formatted I/O; failures/final summaries remain available; GUI stays responsive.
- **Rollback:** loss of actionable failure evidence, hidden safety fault, or no measurable performance/size benefit for added complexity.

### Phase 8 — Memory and allocation

- **Measure before:** `.data/.bss`, stack high-water, free memory, allocation count in hot loops, Shadow save/copy latency.
- **Inspect:** pure-pursuit path copy, motor telemetry vectors, Shadow buffer lifetimes, GUI registries/status strings.
- **Improvement:** zero dynamic allocation in proven critical loops, adequate RAM/stack margin, shorter max time without unsafe ownership.
- **Rollback:** dangling references, buffer aliasing, corrupted Shadow files, stack overflow, or harder-to-reason ownership for negligible gain.

### Phase 9 — Compiler experiments

- **Measure before:** locked baseline configuration from Phases 0-8.
- **Inspect:** exact compile/link flags, prebuilt archive compatibility, warnings, map/sections, generated code for leading hot functions.
- **Improvement:** a matrix row improves worst-case timing or size while all 20-run stability/repeatability gates hold.
- **Rollback:** any new compiler/link failure, code-size/RAM budget violation, timing-tail regression, crash, or endpoint degradation.

### Phase 10 — Regression and autonomous repeatability release gate

- **Measure before:** candidate and immediately preceding accepted baseline, interleaved for at least 20 runs each where practical.
- **Inspect:** raw rows, outliers, failure reasons, battery/temperature drift, deadline misses, task stalls, final errors.
- **Improvement:** predeclared budgets pass; correctness/reliability/repeatability are equal or better; gains persist in worst case and p95, not only mean.
- **Rollback:** any P0/P1 regression or timing gain paired with worse autonomous repeatability. Revert to the last committed checkpoint and retain the failed dataset.

## 19. Requirement coverage and student handoff

| Requested audit area | Report location |
|---|---|
| Performance baseline / high-frequency table | §3 |
| Profiling plan and metrics | §15 |
| Detailed `src/aon/odometry.cpp` audit | §5 |
| Deterministic `delay` versus fixed release | §6 |
| Sensor polling and intake hue states | §7 |
| Blocking operation classification | §8 |
| PROS task/concurrency table | §4 and §14 |
| Memory and allocation classes | §9 |
| Logging and GUI modes | §10 |
| Dead/duplicate runtime work | §11 |
| Compiler `-Os`/`-O2`/LTO analysis and matrix | §13 |
| Math classification | §12 |
| Data copying | §9.3 |
| Code-size/library/section analysis | §13.2 |
| 20-run regression procedure/statistics | §16 |
| P0-P3 ranked backlog | §17 |
| Phases 0-10 with measurement and rollback | §18 |

### First student sprint

The first implementation sprint should be deliberately narrow:

1. Add aggregate-only timing to native odometry, opcontrol, Shadow recorder, and autonomous monitoring.
2. Capture the 20-run baseline before changing behavior.
3. Resolve native pose publication/lock correctness and cross-task `volatile` state ownership with targeted tests.
4. Remove the safety busy loop and gate native per-loop formatted logging.
5. Repeat the same 20-run protocol and compare worst cases/distributions.

Only after that evidence should the team choose between pure-pursuit copy removal, sensor polling changes, command state machines, math representation work, or compiler flags. This order protects the engineering priorities: correctness, reliability, deterministic timing, autonomous repeatability, CPU, memory, then binary size.
