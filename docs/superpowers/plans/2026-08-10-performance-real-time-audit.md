# Performance and Real-Time Audit Implementation Plan

> **Completion record:** Executed in five documentation checkpoints. Checked items were completed against the pinned source; the final build item records the unavailable embedded toolchain explicitly.

**Goal:** Produce a source-grounded performance and real-time systems audit of the `Testing` branch without modifying competition source code.

**Architecture:** The deliverable is one engineering report assembled in reviewable sections. Static searches establish complete inventories; direct source inspection supplies function-level evidence; tables connect every observation to a measurement and student-owned investigation rather than replacement code.

**Tech Stack:** PROS C++ project, GNU Make/ARM toolchain configuration, Markdown documentation, Git.

## Global Constraints

- Do not modify competition source code or generate replacement competition implementations.
- Base every recommendation on the actual `Testing` branch at commit `ac0fedcb1627a7fd0208ca8eaf62a3e2f9117f3c`.
- Prioritize correctness, reliability, deterministic timing, autonomous repeatability, CPU efficiency, memory efficiency, then binary size.
- Preserve comments and documentation; distinguish measured issues from hypotheses that require profiling.
- Commit each coherent documentation checkpoint and verify the report before claiming completion.

---

### Task 1: Establish the Evidence Inventory

**Files:**
- Create: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Inspect: `src/**/*.cpp`, `include/aon/**/*.hpp`, `Makefile`, `common.mk`, `project.pros`

**Interfaces:**
- Consumes: pinned repository commit and user audit brief.
- Produces: source map of loops, tasks, sensors, synchronization, blocking operations, allocations, logging, math, and build settings.

- [x] **Step 1:** Record the audited branch, commit, scope, static-analysis method, and limitations.
- [x] **Step 2:** Search all first-party source and headers for task construction, loops, delay APIs, sensor/motor APIs, mutexes/atomics, dynamic allocation/containers, logging, and expensive math.
- [x] **Step 3:** Read every matching first-party function in context and reject matches in vendored headers or inactive/comment-only code.
- [x] **Step 4:** Create the periodic-loop and PROS-task tables with exact file/function references and explicitly label inferred periods/costs.
- [x] **Step 5:** Verify paths and symbols with `rg`, inspect `git diff --check`, then commit the inventory checkpoint.

### Task 2: Audit Odometry, Timing, Sensors, and Concurrency

**Files:**
- Modify: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Inspect: `src/aon/odometry.cpp`, `include/aon/odometry/odometry.hpp`, and all callers/task owners found in Task 1.

**Interfaces:**
- Consumes: Task 1 inventories.
- Produces: detailed odometry data-flow comparison, deterministic scheduling candidates, sensor-rate recommendations, blocking classification, and shared-resource risk map.

- [x] **Step 1:** Trace odometry sensor acquisition, calculations, lock boundaries, pose getters/setters, duplicate models, and all odometry task startup paths.
- [x] **Step 2:** Compare relative-delay loops with deadline-based scheduling and identify each high-value `delay_until` candidate.
- [x] **Step 3:** Trace sensor reads by state, especially intake scanning/sorting, and recommend intentional polling ranges justified by decision latency.
- [x] **Step 4:** Classify each blocking wait and map every task to shared motors, sensors, mutexes, atomics, and possible command conflicts.
- [x] **Step 5:** Re-run targeted searches, check table completeness, run `git diff --check`, and commit this analysis checkpoint.

### Task 3: Audit Runtime Cost, Memory, Logging, Math, and Builds

**Files:**
- Modify: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Inspect: first-party source plus `Makefile`, `common.mk`, linked libraries, and generated binaries/maps when available.

**Interfaces:**
- Consumes: hot paths and task/resource map from Tasks 1-2.
- Produces: allocation classification, logging/GUI cost assessment, dead/duplicate runtime work, math/copying review, and compiler/code-size experiment matrix.

- [x] **Step 1:** Classify allocation sites as initialization, runtime, or high-frequency and flag only evidence-supported risks.
- [x] **Step 2:** Inventory formatted output and GUI redraw work in timing-critical contexts; define `DEBUG`, `DEVELOPMENT`, and `COMPETITION` behavior.
- [x] **Step 3:** Identify duplicate or debug-only runtime computations while preserving their source availability.
- [x] **Step 4:** Classify hot-path math and copying recommendations by expected value and profiling requirement.
- [x] **Step 5:** Document current compiler/linker flags and define a measurement-driven `-Os`/`-O2`/LTO benchmark matrix without defaulting to `-O3`.
- [x] **Step 6:** Verify every claim against source/build files, run `git diff --check`, and commit this cost-analysis checkpoint.

### Task 4: Design Profiling, Regression Testing, and the Ranked Roadmap

**Files:**
- Modify: `docs/performance/2026-08-10-performance-real-time-audit.md`

**Interfaces:**
- Consumes: all observed risks and candidate optimizations.
- Produces: low-overhead profiler design, 20-run autonomous test protocol, P0-P3 backlog, and phased implementation roadmap with rollback gates.

- [x] **Step 1:** Specify monotonic microsecond measurements, fixed-size aggregate statistics, deadline-miss/jitter definitions, mutex-wait sampling, autonomous errors, and memory/binary measurements without supplying final competition code.
- [x] **Step 2:** Define a controlled protocol for at least 20 autonomous runs and calculations for mean, standard deviation, worst case, and useful 95th percentiles.
- [x] **Step 3:** Rank findings P0-P3 with observed issue, consequence, measurement, investigation, impact, and change risk.
- [x] **Step 4:** Define Phases 0-10 with pre-measurements, inspection scope, success criteria, and rollback triggers.
- [x] **Step 5:** Add an executive summary that separates confirmed findings from profiling hypotheses and names the highest-leverage next actions.
- [x] **Step 6:** Run requirement-by-requirement coverage and placeholder scans, verify all paths/functions, run `git diff --check`, and commit the roadmap checkpoint.

### Task 5: Final Verification and Review

**Files:**
- Review: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Review: all commits made after `ac0fedcb1627a7fd0208ca8eaf62a3e2f9117f3c`

**Interfaces:**
- Consumes: completed report and original audit brief.
- Produces: verified, internally consistent final audit with competition code unchanged.

- [x] **Step 1:** Confirm the audit's own commit set contains documentation only; unrelated concurrent documentation commits were preserved.
- [x] **Step 2:** Check all 17 requested sections and both required tables against the original brief.
- [x] **Step 3:** Validate referenced paths/functions with automated extraction/search plus manual spot checks.
- [x] **Step 4:** Run `git diff --check` and freshly rebuild/run all 13 host tests with warnings as errors. Embedded build and four ARM-object symbol checks were not run because PROS, GNU Make, and the ARM toolchain are not discoverable in this environment.
- [x] **Step 5:** Review the documentation diff independently for standards and spec compliance, resolve findings, and re-verify. This completion record and the resolved findings form the final review checkpoint.
