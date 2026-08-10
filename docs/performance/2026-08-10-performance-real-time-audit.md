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

