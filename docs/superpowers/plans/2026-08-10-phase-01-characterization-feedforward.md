# Phase 1 Characterization and Feedforward Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Record bounded drivetrain characterization experiments, derive reviewed kS/kV/kA constants on the host, and add validated feedforward alongside existing feedback control for each robot.

**Architecture:** Pure value types, dataset validation, regression, and feedforward calculation are independent of PROS and reusable by mechanisms. A PROS experiment adapter obtains exclusive drivetrain ownership and records central sensor snapshots; constants remain inactive until the exact dataset and physical review are committed.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6 adapter seams, fixed-capacity buffers, CSV/CRC32 artifacts, host GCC, Git.

## Global Constraints

- Do not run characterization until the roadmap physical baseline and device-ownership gates pass.
- Keep feedback control active; feedforward only predicts the nominal command.
- Never automatically activate fitted constants.
- Store separate small- and big-robot constants, units, dataset checksum, fit quality, and approval state.
- Bound experiment duration, voltage, acceleration, samples, SD writes, waits, and cancellation latency.
- Characterization owns the drivetrain exclusively and always exits with zero output and brake mode.
- Do not allocate or write files from the sampling loop.

---

## File Structure

- `include/aon/characterization/types.hpp`: experiment, sample, dataset, constants, and validation enums.
- `include/aon/characterization/dataset.hpp` and `src/aon/characterization/dataset.cpp`: pure validation and bounded append.
- `include/aon/control/feedforward.hpp` and `src/aon/control/feedforward.cpp`: reusable voltage model.
- `tools/characterization/analyze.cpp`: host CSV reader, regression, residual report, and proposal writer.
- `src/aon/characterization/drivetrain-runner.cpp`: PROS experiment adapter.
- `tests/characterization-test.cpp`: pure validation/regression/feedforward tests.
- `docs/characterization/`: experiment procedure, generated proposals, and measured approvals.

### Task 1: Characterization Types and Fail-Closed Validation

**Files:**
- Create: `include/aon/characterization/types.hpp`
- Create: `include/aon/characterization/dataset.hpp`
- Create: `src/aon/characterization/dataset.cpp`
- Create: `tests/characterization-test.cpp`

**Interfaces:**
- Produces: `ExperimentProfile`, `ExperimentDirection`, `CharacterizationSample`, `CharacterizationDataset`, `DatasetIssue`, and `validateDataset`.
- `CharacterizationDataset` uses `std::array<CharacterizationSample, kMaximumCharacterizationSamples>` and a count.

- [ ] **Step 1:** Write failing tests for bounded append, monotonic timestamps, finite values, valid battery/command voltage, both wheel directions, minimum duration/excitation, stale gaps, inconsistent direction, capacity, and empty data.
- [ ] **Step 2:** Compile with `g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\characterization-test.cpp src\aon\characterization\dataset.cpp` and confirm missing symbols.
- [ ] **Step 3:** Implement one typed `DatasetIssue` per rejection and return the first deterministic issue; retain invalid data only as an unapproved artifact.
- [ ] **Step 4:** Run tests and add `static_assert` checks for fixed capacity and trivially copyable samples.
- [ ] **Step 5:** Commit `feat(characterization): validate bounded datasets` and push.

### Task 2: Pure Feedforward and Regression

**Files:**
- Create: `include/aon/control/feedforward.hpp`
- Create: `src/aon/control/feedforward.cpp`
- Modify: `tests/characterization-test.cpp`

**Interfaces:**
- Produces: `FeedforwardConstants {kSVolts, kVVoltSecondsPerInch, kAVoltSecondsSquaredPerInch, approved, datasetCrc}`; `Feedforward::voltage(velocity, acceleration)`; `FitResult fitFeedforward(dataset)`.

- [ ] **Step 1:** Add synthetic noiseless and noisy datasets with known constants, forward/reverse static friction, zero velocity, singular data, outliers, and non-finite values.
- [ ] **Step 2:** Confirm tests fail.
- [ ] **Step 3:** Implement fixed-size normal-equation accumulation with pivot/singularity guards; do not add a matrix dependency or copy all samples.
- [ ] **Step 4:** Report sample count, RMS residual, maximum residual, condition rejection, and fitted constants. `approved` remains false for every fit.
- [ ] **Step 5:** Implement voltage calculation as `sign(velocity)*kS + kV*velocity + kA*acceleration`, define zero-velocity sign policy explicitly, reject invalid constants, and clamp only at the hardware adapter.
- [ ] **Step 6:** Run tests, commit `feat(control): fit and calculate feedforward`, and push.

### Task 3: Host Analysis Tool and Versioned Artifact

**Files:**
- Create: `tools/characterization/analyze.cpp`
- Create: `tools/characterization/README.md`
- Create: `tests/characterization-cli-test.ps1`

**Interfaces:**
- Consumes: versioned CSV with robot identity, experiment metadata, and exact sample columns.
- Produces: human report plus machine-readable `.proposal` containing format version, units, checksum, fit metrics, and inactive constants.

- [ ] **Step 1:** Write CLI tests for four valid experiment files, missing profiles, corrupt headers, duplicate timestamps, wrong robot identity, singular samples, and deterministic output.
- [ ] **Step 2:** Implement bounded line parsing with explicit maximum file/line/sample counts; do not silently skip malformed rows.
- [ ] **Step 3:** Combine all four direction/profile datasets only after each validates and identities/units match.
- [ ] **Step 4:** Print constants and residuals without editing robot configuration.
- [ ] **Step 5:** Run CLI and pure tests, commit `tool(characterization): analyze feedforward datasets`, and push.

### Task 4: Safe Drivetrain Experiment Runner

**Files:**
- Create: `include/aon/characterization/runner.hpp`
- Create: `src/aon/characterization/drivetrain-runner.cpp`
- Modify: target configuration and autonomous test catalog files.
- Create: `tests/characterization-runner-policy-test.cpp`

**Interfaces:**
- Consumes: drivetrain lease, central sensor snapshot, battery voltage, cancellation, experiment configuration, and preallocated dataset.
- Produces: one stopped `ExperimentResult` with typed reason and captured dataset.

- [ ] **Step 1:** Test pure voltage schedules for quasistatic forward/reverse and dynamic forward/reverse, maximum duration, maximum voltage, cancellation, stale sensor, capacity, and invalid feedback.
- [ ] **Step 2:** Implement the PROS adapter with no SD calls in the sampling loop and a scope guard that commands zero on every exit.
- [ ] **Step 3:** Add one visibly locked GUI/test entry. It remains unable to command motors until a robot-specific `characterizationTestingAuthorized` flag is enabled in a dedicated supervised-test commit.
- [ ] **Step 4:** Run host suites and both clean builds; review ownership and stack usage.
- [ ] **Step 5:** Commit runner and locked UI separately, pushing each.

### Task 5: Physical Characterization Gate

**Files:**
- Create: `docs/testing/2026-08-10-drivetrain-characterization-checklist.md`
- Create measured files under `docs/characterization/small/` and `big/`.
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: both physical robots, charged batteries, safe test area, four experiment profiles, and exact firmware commit.
- Produces: committed raw datasets and inactive fit proposals.

- [ ] **Step 1:** Run low-voltage supervised tests first, verify direction and Controller-X, then collect all four experiments per robot.
- [ ] **Step 2:** Record battery state, surface, mass/configuration, wheel setup, start temperature, warnings, and aborted runs.
- [ ] **Step 3:** Analyze on host and reject any invalid dataset; never hand-edit fitted output.
- [ ] **Step 4:** Commit raw data and proposals separately from any constant activation.
- [ ] **Step 5:** Stop for human review of residuals and physical plausibility.

### Task 6: Activate Reviewed Constants and Integrate Feedback Plus Feedforward

**Files:**
- Modify: target configuration files.
- Modify: existing drivetrain controller adapter seams, not vendored LemLib.
- Modify: `tests/characterization-test.cpp`
- Create: `docs/testing/2026-08-10-feedforward-validation-checklist.md`

**Interfaces:**
- Consumes: explicitly approved proposal checksum and robot identity.
- Produces: `command = feedforward(reference) + existing feedback(error)` with existing output limits and cancellation.

- [ ] **Step 1:** Add configuration tests rejecting approved constants with wrong robot, checksum, units, negative/non-finite gains, or unacceptable residual metadata.
- [ ] **Step 2:** Add feedforward at the narrowest existing non-vendored controller seam; preserve feedback gains and record feedforward/feedback components separately.
- [ ] **Step 3:** Run host tests and dual builds, then perform low-speed step and trajectory comparisons with feedforward disabled/enabled.
- [ ] **Step 4:** Accept only repeated improvement without worse overshoot, current, cancellation, or endpoint behavior; otherwise leave constants present but inactive.
- [ ] **Step 5:** Update handoff, commit activation per robot separately, and push.

