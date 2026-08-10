# Phase 12 Performance Benchmarking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Run repeatable multi-trial robot benchmarks, retain failures, compute defensible statistics, and compare controller configurations without automatic promotion.

**Architecture:** Pure benchmark descriptions, trial results, online/bounded statistics, thresholds, and versioned export are independent of hardware. A robot runner uses the autonomous executive and telemetry; a human establishes each initial condition and authorizes each repetition.

**Tech Stack:** C++17 fixed-capacity results, Phase 9 statistics, Phase 10 telemetry/replay, autonomous executive, PROS Brain/SD adapters, host report tool.

## Global Constraints

- Never accept a configuration from one run.
- Failed, timed-out, cancelled, and invalid trials remain in results and success-rate denominators.
- A benchmark names robot identity, hardware/config digest, firmware commit, initial pose procedure, repetitions, action, channels, and thresholds.
- Robot execution, export, and human summary are bounded; no uncontrolled logging.
- Benchmarks do not unlock competition routes or controller constants automatically.
- Physical setup and environmental differences are recorded, not normalized away silently.

---

## File Structure

- `include/aon/benchmark/description.hpp`: benchmark/action/threshold contract.
- `include/aon/benchmark/result.hpp`: trial and aggregate fixed values.
- `include/aon/benchmark/statistics.hpp` and `.cpp`: mean/stddev/max/success and comparison.
- `src/aon/benchmark/runner.cpp`: executive/telemetry adapter.
- `include/aon/benchmark/codec.hpp` and `.cpp`: machine-readable export.
- `tools/benchmark/report.cpp`: human report and comparison.
- `tests/benchmark-statistics-test.cpp`, `benchmark-runner-test.cpp`, `benchmark-codec-test.cpp`.

### Task 1: Benchmark and Trial Contracts

**Files:**
- Create description/result headers and statistics test.

**Interfaces:**
- Produces: `BenchmarkId`, `BenchmarkDescription`, `AcceptanceThresholds`, `TrialOutcome`, `TrialResult`, and fixed `BenchmarkResult`.

- [ ] **Step 1:** Test valid MoveToPose description, zero/excess repetitions, non-finite pose/threshold, missing configuration digest, unsupported action, fixed trial capacity, and outcome retention.
- [ ] **Step 2:** Implement explicit units and maximum repetitions; descriptions are immutable during a run.
- [ ] **Step 3:** Run tests, commit `feat(benchmark): define repeatable robot trials`, and push.

### Task 2: Statistics and Acceptance

**Files:**
- Create statistics header/source and complete statistics tests.

**Interfaces:**
- Produces: translational/heading/path/time/current/sag/overrun aggregates, success/timeout counts, and `AcceptanceResult` with every failed threshold.

- [ ] **Step 1:** Test known mean/sample standard deviation/max, heading wrap error, all failures, mixed outcomes, one valid sample, NaN rejection, and success percentage denominator.
- [ ] **Step 2:** Reuse Phase 9 online statistics and bounded retained results; do not discard failed trials from success/time reporting.
- [ ] **Step 3:** Implement threshold evaluation only after required repetitions complete and setup validity is confirmed.
- [ ] **Step 4:** Run tests, commit `feat(benchmark): calculate trial acceptance`, and push.

### Task 3: Versioned Result Codec and Reports

**Files:**
- Create codec source/test and report tool.

**Interfaces:**
- Produces: versioned machine result with robot/config/commit/environment/trials/aggregates/checksum and deterministic human summary.

- [ ] **Step 1:** Test round-trip, deterministic export, corrupt checksum, unsupported version, impossible count, missing failure trials, and two-controller comparison compatibility.
- [ ] **Step 2:** Implement bounded explicit serialization using Phase 10 conventions without writing raw structs.
- [ ] **Step 3:** Report mean/stddev/max, heading error, success/timeout, execution time, current, sag, slip, health faults, and deadline misses.
- [ ] **Step 4:** Run tests, commit `feat(benchmark): export benchmark results`, and push.

### Task 4: Host Fake Runner and Resource Policy

**Files:**
- Create runner source and runner tests.

**Interfaces:**
- Consumes: benchmark description, executive action adapter, pose/path metrics, telemetry summaries, cancellation, and explicit human-ready acknowledgement.
- Produces: one trial at a time with safe stop and no automatic next motion.

- [ ] **Step 1:** Test ready/start/execute/collect/complete, failure, timeout, cancel, invalid initial pose, health rejection, storage failure, maximum repetitions, resource conflict, and safe-stop count.
- [ ] **Step 2:** Require explicit acknowledgement between physical repetitions; runner never assumes the robot was reset.
- [ ] **Step 3:** Run executive/health/telemetry/benchmark tests, commit `feat(benchmark): run bounded benchmark trials`, and push.

### Task 5: Initial Benchmark Catalog

**Files:**
- Create target benchmark catalogs and catalog tests.
- Create: `docs/benchmark/benchmark-protocol.md`.

**Interfaces:**
- Produces: MoveToPoseAccuracy, TurnAccuracy, PathRepeatability, CancellationLatency, MechanismTransition, and RuntimeLoad descriptions where supported.

- [ ] **Step 1:** Host-test exact initial pose, action, repetitions, target, thresholds, and required channels for every catalog entry.
- [ ] **Step 2:** Keep unmeasured thresholds labeled proposal and non-authorizing in configuration; runner may collect but not declare pass.
- [ ] **Step 3:** Build both robots with only supported catalog capabilities present.
- [ ] **Step 4:** Commit `feat(benchmark): catalog repeatability tests` and push.

### Task 6: Physical Benchmark Gate and Controller Comparison

**Files:**
- Create: `docs/testing/2026-08-10-performance-benchmark-checklist.md`
- Store versioned results under `docs/benchmark/results/<robot>/<benchmark>/` after review.
- Modify handoff.

- [ ] **Step 1:** Establish marked initial pose, surface, battery window, mechanism configuration, payload, temperature, and operator procedure.
- [ ] **Step 2:** Run at least ten repetitions for basic acceptance and twenty for controller/noise comparisons where practical.
- [ ] **Step 3:** Record every trial including timeout/cancel/fault; rerun only when setup was explicitly invalid and retain that invalid record.
- [ ] **Step 4:** Compare configurations only on compatible setup/digest/protocol and report statistical distribution plus worst case.
- [ ] **Step 5:** Human reviewers decide promotion; commit measured results and any configuration activation separately and push.

