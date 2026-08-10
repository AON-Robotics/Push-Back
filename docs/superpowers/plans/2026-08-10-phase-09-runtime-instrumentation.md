# Phase 9 Runtime Instrumentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Measure loop periods, jitter, execution time, deadline misses, queue pressure, stale data, and stack risk with bounded overhead before changing control scheduling.

**Architecture:** A platform-independent statistics core consumes explicit timestamps and publishes fixed-size snapshots. Thin PROS adapters instrument named loops; high-frequency paths only update aggregates, while GUI/terminal formatting occurs at low rate. Confirmed synchronization defects from the real-time audit are corrected in isolated, behavior-preserving checkpoints.

**Tech Stack:** C++17, PROS 4.2.2 RTOS, host GCC, fixed-capacity value types, Brain GUI, Git.

## Global Constraints

- Use `docs/performance/2026-08-10-performance-real-time-audit.md` as the evidence baseline.
- Do not invent V5 timing, mutex-wait, stack, or sensor-call measurements.
- Do not change loop periods and logic in the same checkpoint that introduces measurement.
- No terminal, GUI, SD, heap allocation, or unbounded lock in the measurement hot path.
- Preserve all cancellation, authorization, and fallback behavior.
- Keep aggregate storage statically bounded and document its total bytes.
- Run host tests and clean builds for both robot targets at every robot-code checkpoint.

---

## File Structure

- `include/aon/runtime/online-statistics.hpp`: allocation-free count/min/max/mean/variance.
- `include/aon/runtime/loop-monitor.hpp`: pure loop timing and deadline state.
- `include/aon/runtime/buffer-monitor.hpp`: capacity/occupancy/high-water/drop counters.
- `include/aon/runtime/registry.hpp` and `src/aon/runtime/registry.cpp`: fixed named monitor registry and immutable snapshots.
- `src/aon/runtime/pros-clock.cpp`: monotonic millisecond/microsecond adapter.
- `tests/runtime-statistics-test.cpp`: numerical and wrap tests.
- `tests/loop-monitor-test.cpp`: timing/deadline transitions.
- `tests/buffer-monitor-test.cpp`: capacity and saturation tests.
- `docs/runtime/loop-rates.md`: declared target rates and physical measurements.

### Task 1: Allocation-Free Online Statistics

**Files:**
- Create: `include/aon/runtime/online-statistics.hpp`
- Create: `tests/runtime-statistics-test.cpp`

**Interfaces:**
- Produces: `OnlineStatistics::observe(double)`, `reset()`, and `StatisticsSnapshot snapshot() const`.
- Consumes: finite scalar samples; rejects non-finite values with an invalid counter.

- [ ] **Step 1: Write failing numerical tests**

Test empty state, the sequence `{1, 2, 3, 4}`, negative samples, large values, reset, NaN, and infinity. Expected `{count=4,min=1,max=4,mean=2.5,variance=1.666666...}` using sample variance.

- [ ] **Step 2: Compile and confirm failure**

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\runtime-statistics-test.cpp -o bin\host-tests\runtime-statistics-test.exe
```

- [ ] **Step 3: Implement Welford accumulation**

Use fixed scalar members only. `observe` returns false for non-finite values and increments `invalidSamples`. Empty snapshots report count zero and zero-valued aggregates with `valid=false`.

- [ ] **Step 4: Run the test under warnings-as-errors**

Expected: `runtime statistics tests passed`.

- [ ] **Step 5: Commit and push**

```powershell
git add include/aon/runtime/online-statistics.hpp tests/runtime-statistics-test.cpp
git commit -m "feat(runtime): add bounded online statistics"
git push origin Testing
```

### Task 2: Pure Loop and Deadline Monitor

**Files:**
- Create: `include/aon/runtime/loop-monitor.hpp`
- Create: `tests/loop-monitor-test.cpp`

**Interfaces:**
- Produces: `LoopMonitor::begin(releaseUs, startUs)`, `finish(finishUs)`, `LoopTimingSnapshot snapshot()`, and wrap-safe millisecond deadline helpers.
- Consumes: configured `targetPeriodUs` and ordered 64-bit microsecond samples from either real or replay clock.

- [ ] **Step 1:** Write tests for exact period, early/late start, execution duration, one and multiple missed deadlines, invalid timestamp order, reset, and 32-bit millisecond wrap helpers.
- [ ] **Step 2:** Confirm compile failure.
- [ ] **Step 3:** Implement fixed aggregates for actual period, release jitter, execution duration, maximum duration, deadline misses, invalid samples, and iterations.
- [ ] **Step 4:** Run tests; no percentile or dynamic sample storage is added in this checkpoint.
- [ ] **Step 5:** Commit `feat(runtime): measure loop timing and deadlines` and push.

### Task 3: Buffer Pressure and Fixed Registry

**Files:**
- Create: `include/aon/runtime/buffer-monitor.hpp`
- Create: `include/aon/runtime/registry.hpp`
- Create: `src/aon/runtime/registry.cpp`
- Create: `tests/buffer-monitor-test.cpp`
- Create: `tests/runtime-registry-test.cpp`

**Interfaces:**
- Produces: `BufferMonitor::publishOccupancy(size_t)`, `recordDrop(size_t)`, immutable `BufferSnapshot`, `LoopId`, and `RuntimeSnapshot`.
- Registry capacity: compile-time `kMaximumLoopMonitors` and `kMaximumBufferMonitors`; no dynamic names.

- [ ] **Step 1:** Test occupancy zero/capacity, high-water behavior, invalid over-capacity publication, saturating drop counters, and registry duplicate-ID rejection.
- [ ] **Step 2:** Confirm failure, implement fixed arrays/enums, and run tests.
- [ ] **Step 3:** Add `static_assert` checks for snapshot trivial copyability and total registry storage below the documented budget.
- [ ] **Step 4:** Commit `feat(runtime): track bounded runtime resources` and push.

### Task 4: Instrument Without Rescheduling

**Files:**
- Create: `include/aon/runtime/clock.hpp`
- Create: `src/aon/runtime/pros-clock.cpp`
- Modify: `src/aon/core/robot.cpp`
- Modify: `src/aon/auton/actions.cpp`
- Modify: `src/aon/auton/encoder-motion.cpp`
- Modify: `src/aon/shadow/service.cpp`
- Modify: `src/aon/odometry.cpp`
- Modify: mechanism loops selected by active robot.
- Test: existing motion, Shadow, and architecture suites.

**Interfaces:**
- Consumes: current loop release/start/finish points and process runtime registry.
- Produces: named timing snapshots for opcontrol, action monitor, encoder fallback, Shadow sampler, legacy odometry, intake scan/sort, and safety task.

- [ ] **Step 1:** Add the PROS clock adapter using monotonic `pros::micros()`/`pros::millis()` with no formatting.
- [ ] **Step 2:** Wrap one loop at a time with `begin`/`finish`, retaining its existing `pros::delay` and command order.
- [ ] **Step 3:** Run the relevant focused host tests and both clean builds after each loop group.
- [ ] **Step 4:** Inspect generated stack-usage output for touched functions and record static registry bytes in `docs/runtime/loop-rates.md`.
- [ ] **Step 5:** Commit drivetrain, localization, mechanisms, and Shadow instrumentation as separate commits, updating and pushing the handoff each time.

### Task 5: Correct Confirmed Synchronization Hazards

**Files:**
- Modify: `include/aon/odometry/odometry.hpp`
- Modify: `src/aon/odometry.cpp`
- Modify: `include/aon/globals.hpp`
- Modify: `include/aon/intake/intake.hpp`
- Modify: `include/aon/orbit/orbit.hpp`
- Modify: `include/aon/core/hardware.hpp`
- Create: `tests/odometry-snapshot-test.cpp`
- Create: `tests/safety-loop-policy-test.cpp`
- Create: `tests/cross-task-state-test.cpp`

**Interfaces:**
- Produces: coherent `OdometrySnapshot`, checked lock policy, edge-aware/yielding safety policy, and synchronized cross-task flags.
- Consumes: unchanged pose conventions and mechanism state transitions.

- [ ] **Step 1:** Extract and test a pure odometry publisher that copies x/y/heading/timestamp/sequence atomically as one value.
- [ ] **Step 2:** Replace ignored timed-lock results; failed acquisition increments a diagnostic counter and never calls `give()` without ownership.
- [ ] **Step 3:** Make `getPose()` derive from one snapshot and serialize reset/update; preserve formulas in this checkpoint.
- [ ] **Step 4:** Test Controller-X edge/held/release policy; one press cancels immediately, a held button yields at a bounded watchdog period, and release does not rearm cancellation.
- [ ] **Step 5:** Replace cross-task `volatile` booleans/enums with atomics or mutex-owned snapshots according to whether fields form an invariant.
- [ ] **Step 6:** Run race-focused host policy tests, legacy/native motion tests, both builds, and physical baseline regression before commit.
- [ ] **Step 7:** Commit each P0/P1 correction separately and push.

### Task 6: Add Low-Rate Diagnostics View

**Files:**
- Modify: `include/aon/tools/gui/gui-debug.hpp`
- Modify: `src/aon/tools/gui/gui-debug.cpp`
- Modify: `src/aon/tools/gui/ui/gui-displays.cpp`
- Create: `tests/runtime-format-test.cpp`

**Interfaces:**
- Consumes: one immutable `RuntimeSnapshot` at no more than 10 Hz.
- Produces: loop target/actual/max/jitter/deadline values and buffer occupancy/drop display.

- [ ] **Step 1:** Write a pure fixed-buffer formatter test including empty, healthy, missed-deadline, and truncated-name cases.
- [ ] **Step 2:** Implement formatting outside the registry lock and without `std::string` in the snapshot.
- [ ] **Step 3:** Add a debug-only Brain view; normal match screen displays only active timing faults.
- [ ] **Step 4:** Run GUI build, formatter host test, dual builds, and measure GUI refresh cost on hardware.
- [ ] **Step 5:** Commit `feat(gui): display bounded runtime diagnostics` and push.

### Task 7: Physical Timing and Stack Gate

**Files:**
- Modify: `docs/runtime/loop-rates.md`
- Create: `docs/testing/2026-08-10-runtime-instrumentation-checklist.md`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: instrumented small and big builds.
- Produces: measured periods, jitter, maximum execution, misses, stack headroom, and observer-effect notes.

- [ ] **Step 1:** Record idle, driver-control, representative autonomous, Shadow recording, and cancellation runs on each robot.
- [ ] **Step 2:** Compare instrumentation enabled/disabled endpoint behavior and loop maxima; do not approve instrumentation that changes safety or repeatability materially.
- [ ] **Step 3:** Record stack values or state explicitly where PROS cannot provide a trustworthy measurement.
- [ ] **Step 4:** Commit measurements separately; only then plan loop rescheduling using observed misses/jitter.

