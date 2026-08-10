# Phase 10 Telemetry Recording and Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record a bounded, versioned subset of robot state without disturbing control, then replay real traces deterministically through host-testable algorithms.

**Architecture:** Producers emit fixed-size typed frames into a nonblocking bounded queue. A lower-priority recorder owns SD writes and versioned chunks with checksums. Host replay validates the stream and drives selected pure consumers with an injected clock; Shadow retains its separate route format and storage policy.

**Tech Stack:** C++17 fixed buffers, CRC32, PROS SD adapter, existing Shadow storage lessons, host replay executable, runtime/health instrumentation.

## Global Constraints

- No filesystem operation in sensor, estimator, control, observer, mechanism, or driver high-frequency loops.
- Producer enqueue is bounded and never waits behind SD.
- Queue capacity, occupancy, high-water mark, drops, chunk count, and write faults are observable.
- Match defaults record a small low-overhead channel set; Debug/Trace are explicit opt-ins.
- Preserve Shadow file compatibility and do not route Shadow playback through general telemetry.
- Validate size/count/version/checksum before allocating or replaying data.
- Replayed external data never commands a physical robot.

---

## File Structure

- `include/aon/telemetry/frame.hpp`: header, channel/version, fixed payload.
- `include/aon/telemetry/queue.hpp`: single/multi-producer policy and bounded ring.
- `include/aon/telemetry/codec.hpp` and `.cpp`: file/chunk format and CRC.
- `src/aon/telemetry/recorder.cpp`: state machine and low-priority drain.
- `src/aon/telemetry/storage-pros.cpp`: only general telemetry SD owner.
- `tools/telemetry/replay.cpp`: host validator/dispatcher.
- `tests/telemetry-queue-test.cpp`, `telemetry-codec-test.cpp`, `telemetry-replay-test.cpp`.

### Task 1: Typed Frame and Channel Budget

**Files:**
- Create frame header and `tests/telemetry-queue-test.cpp`.

**Interfaces:**
- Produces: `TelemetryLevel`, `ChannelId`, `FrameHeader`, fixed `TelemetryFrame`, and per-channel `ChannelConfig {enabled, minimumLevel, decimation, maximumAgeMs}`.

- [ ] **Step 1:** Test stable channel IDs, payload bounds, timestamp/sequence, unknown version handling, disabled/decimated channel decisions, and trivial copyability.
- [ ] **Step 2:** Implement one envelope and typed encode/decode helpers for initial pose, raw odometry, health fault, runtime timing, action result, and drivetrain observation channels.
- [ ] **Step 3:** Document exact bytes per frame and worst-case configured bandwidth.
- [ ] **Step 4:** Run tests, commit `feat(telemetry): define bounded telemetry frames`, and push.

### Task 2: Nonblocking Bounded Queue

**Files:**
- Create queue header/source as required and complete queue tests.

**Interfaces:**
- Produces: `tryPush`, `tryPop`, immutable queue statistics, and explicit single-producer/single-consumer or protected multi-producer contract.

- [ ] **Step 1:** Test empty/full/wrap, FIFO, high-water mark, producer drop, consumer underrun, counter saturation, reset while stopped, and concurrency policy misuse.
- [ ] **Step 2:** Implement fixed ring with minimal bounded synchronization; a full queue drops according to configured priority and never blocks control.
- [ ] **Step 3:** Integrate Phase 9 buffer monitor and test statistics consistency.
- [ ] **Step 4:** Commit `feat(telemetry): queue frames without blocking control` and push.

### Task 3: Versioned Chunk Codec and Corruption Rejection

**Files:**
- Create codec header/source and codec tests.

**Interfaces:**
- Produces: file header with magic/version/robot/config digest/start time, chunk header with length/count/CRC, and explicit little-endian serialization.

- [ ] **Step 1:** Test round-trip, deterministic bytes, truncated header/chunk, wrong magic/version/robot, impossible counts/length, checksum corruption, unknown optional channel, non-finite payload, and multi-chunk ordering.
- [ ] **Step 2:** Implement bounded reader/writer over caller-provided buffers; never write C++ structs directly.
- [ ] **Step 3:** Add compatibility dispatch for supported versions and fail closed for unsupported versions.
- [ ] **Step 4:** Run tests, commit `feat(telemetry): encode checksummed trace chunks`, and push.

### Task 4: Recorder and PROS Storage Adapter

**Files:**
- Create recorder and storage adapter.
- Modify robot composition/task startup.
- Create: `tests/telemetry-recorder-test.cpp`.

**Interfaces:**
- Produces: `Stopped`, `Recording`, `Stopping`, `Flushing`, `Complete`, and `Fault` states; bounded chunk drain; `RecorderStatus`.

- [ ] **Step 1:** Test start/stop, already active, queue drops, partial chunk, write/flush/close failure, SD removal, cancellation, maximum duration/bytes, and restart.
- [ ] **Step 2:** Reuse Shadow's 4 KiB bounded-write and cleanup lessons through shared low-level policy only where format ownership remains separate.
- [ ] **Step 3:** Run writer in a low-priority task; control producers never take its filesystem lock.
- [ ] **Step 4:** Instrument queue/writer timing, run tests and dual builds, commit recorder locked behind debug configuration, and push.

### Task 5: Deterministic Host Replay

**Files:**
- Create replay tool and replay tests.
- Add small synthetic fixtures under `tests/fixtures/telemetry/`.

**Interfaces:**
- Produces: validated chronological frame iterator, injected `ReplayClock`, channel dispatcher, and consumer adapters for estimator/observers/health/executive policies.

- [ ] **Step 1:** Test normal chronology, equal timestamps, gaps, wrap, out-of-order frame, dropped-frame metadata, unknown channel, consumer rejection, and deterministic repeated result.
- [ ] **Step 2:** Implement no real sleeps; replay advances the injected clock to recorded timestamps.
- [ ] **Step 3:** Add comparison mode that runs two configurations on one trace and emits machine-readable metrics without accepting a winner.
- [ ] **Step 4:** Run all pure algorithm suites against synthetic fixtures, commit `tool(telemetry): replay recorded robot traces`, and push.

### Task 6: Physical SD and Observer-Effect Gate

**Files:**
- Create: `docs/testing/2026-08-10-telemetry-recording-checklist.md`
- Add captured fixtures only after privacy/size review.
- Modify handoff.

- [ ] **Step 1:** Measure control loop maxima, misses, queue drops, CPU proxy, endpoint repeatability, SD latency, and file size with telemetry off/on at match default.
- [ ] **Step 2:** Test full queue, long run, SD absence/removal, interrupted power only under safe disposable data procedure, and recovery.
- [ ] **Step 3:** Verify Shadow recording and telemetry cannot contend unsafely; define explicit mutual exclusion or shared storage coordinator.
- [ ] **Step 4:** Commit measured trace fixtures and acceptance separately; keep high-rate channels disabled if observer effect is material.

