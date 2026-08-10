# Phase 4 Fault-Tolerant Autonomous Executive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute bounded sequential and parallel autonomous actions with explicit results, resource ownership, cancellation, fallback, recovery, optional/critical policy, and guaranteed safe termination.

**Architecture:** A pure fixed-capacity plan graph and tick-driven executive own lifecycle and propagation. Leaf adapters wrap existing monitored motions and migrated nonblocking mechanisms; native routes remain outside until converted and physically validated one at a time.

**Tech Stack:** C++17 fixed arrays/function-pointer adapters, existing Actions/MotionControl/Shadow/mechanisms, health and constraints, host fake clock/resources.

## Global Constraints

- No infinite wait, retry, recovery, join, or cancellation path.
- Parallel nodes declare resource masks before starting; conflicts never start.
- Controller-X, disable, and critical health cancellation always win.
- Every terminal path invokes the safe-stop coordinator and releases resources.
- Do not hide leaf failures; preserve typed detail when mapping to `ActionOutcome`.
- Keep existing native autonomous routines registered until replacements pass their physical gates.
- Avoid heap allocation and `std::function` construction inside executive ticks.

---

## File Structure

- `include/aon/auton/action-result.hpp`: common result envelope and mappings.
- `include/aon/auton/action.hpp`: leaf lifecycle interface using function pointers/context.
- `include/aon/auton/plan.hpp`: fixed nodes, child indices, criticality, deadlines, resource masks.
- `include/aon/auton/executive.hpp` and `.cpp`: tick/cancel/propagation.
- `src/aon/auton/existing-action-adapters.cpp`: MotionResult/Shadow/mechanism adapters.
- `tests/autonomous-executive-test.cpp`: fake actions, clock, ownership, safe stop.

### Task 1: Common Result and Existing Mappings

**Files:**
- Create result header and `tests/autonomous-executive-test.cpp`.

**Interfaces:**
- Produces: `ActionOutcome`, `ActionResult`, `ActionCriticality`, exhaustive `fromMotionResult`, `fromShadowResult`, and mechanism mappings.

- [ ] **Step 1:** Test every existing enum value maps to stable high-level outcome while retaining subsystem detail; unknown future enum additions must cause compile warning/error through exhaustive switches.
- [ ] **Step 2:** Implement fixed value result with timestamps and recovery flag; no strings.
- [ ] **Step 3:** Run test, commit `feat(auton): unify action results`, and push.

### Task 2: Leaf Lifecycle and Resource Ownership

**Files:**
- Create action/plan headers and extend tests.

**Interfaces:**
- Produces: `ActionVTable {start, update, cancel, result}`, borrowed context pointer, `ResourceMask`, and fixed `ActionNode`.

- [ ] **Step 1:** Test start/update/success/failure/cancel ordering, double start, double cancel, missing callback, deadline, and resource conflict.
- [ ] **Step 2:** Implement explicit lifecycle without virtual allocation; node plans reject invalid indices/cycles/capacity before execution.
- [ ] **Step 3:** Run tests, commit `feat(auton): define bounded action lifecycle`, and push.

### Task 3: Sequential, Parallel, Optional, and Critical Execution

**Files:**
- Create executive header/source and extend tests.

**Interfaces:**
- Produces: `Executive::start(plan, now)`, `tick(now)`, `cancel(now)`, `status`, and one safe-stop callback.

- [ ] **Step 1:** Test sequential success, required failure, optional failure/continue, parallel all-success, parallel critical failure cancelling siblings, parallel timeout, busy resource, empty/invalid plan, and timestamp wrap.
- [ ] **Step 2:** Implement fixed-capacity node runtime state and absolute deadlines.
- [ ] **Step 3:** Ensure safe stop occurs exactly once on every whole-plan exit and every active leaf receives bounded cancellation.
- [ ] **Step 4:** Run tests, commit `feat(auton): execute structured action plans`, and push.

### Task 4: Fallback and Recovery Policy

**Files:**
- Modify plan/executive files and tests.

**Interfaces:**
- Produces: optional fallback/recovery child indices and `RecoveryBudget {maximumAttempts, deadlineMs}`.

- [ ] **Step 1:** Test Blocked->back-away, PoseUnreliable->relocalize, Timeout optional skip, SensorFailure reduced route, recovery failure, exhausted attempts, cancellation during recovery, and resource handoff.
- [ ] **Step 2:** Implement outcome-filtered transitions with global deadline never extended by recovery.
- [ ] **Step 3:** Run tests, commit `feat(auton): bound fallback and recovery actions`, and push.

### Task 5: Existing Motion, Shadow, Mechanism, Health, and Constraint Adapters

**Files:**
- Create existing adapter source and focused tests.
- Modify safety coordinator and routine wrapper files.

**Interfaces:**
- Consumes: `Actions`, Shadow service/player, migrated mechanism commands, health preconditions, and motion constraints.
- Produces: leaves with correct resource masks and result mappings.

- [ ] **Step 1:** Wrap one synchronous monitored motion as a transitional leaf run on the autonomous task; it must retain cancellation/timeout and cannot participate in false parallel drivetrain execution.
- [ ] **Step 2:** Wrap nonblocking mechanism actions only after Phase 6 migration.
- [ ] **Step 3:** Check health/constraints before start and preserve original subsystem result detail.
- [ ] **Step 4:** Run motion-fallback, Shadow, mechanism, health, constraint, and executive suites plus dual builds.
- [ ] **Step 5:** Commit adapters by subsystem and push each.

### Task 6: Incremental Route Migration

**Files:**
- Create target route plan definitions; modify catalogs only after gates.
- Create route-specific executive tests.

- [ ] **Step 1:** Migrate isolated validation routine first, then a short optional-mechanism test, then one competition route segment.
- [ ] **Step 2:** Host-test exact node order, deadlines, resources, optional/critical flags, fallback branch, and final safe stop.
- [ ] **Step 3:** Keep native selector available in a separate slot until three repeated physical successes and cancellation pass.
- [ ] **Step 4:** Commit each route migration and promotion separately; do not bulk-convert native routines.

### Task 7: Physical Executive Failure Matrix

**Files:**
- Create: `docs/testing/2026-08-10-autonomous-executive-checklist.md`
- Modify handoff.

- [ ] **Step 1:** Test success, motion timeout, blockage, poor pose, sensor failure, mechanism failure, optional skip, recovery success/failure, Controller-X, and disable.
- [ ] **Step 2:** Verify exact result/fault, branch taken, total deadline, resource release, and stopped motors/mechanisms.
- [ ] **Step 3:** Repeat promoted route at least three cold boots; retain native fallback until validated replacement meets acceptance.
- [ ] **Step 4:** Commit measurements and authorization separately and push.

