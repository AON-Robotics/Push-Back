# Advanced Robot Software Roadmap Design

## Status

Approved in conversation on 2026-08-10. This document defines the dependency
order, shared architecture, validation boundaries, and delivery policy for the
twelve requested reliability systems. It does not authorize crossing an
existing physical validation gate or changing a robot authorization flag
without recorded measurements.

## Purpose

Evolve the repository from several reliable but partly independent systems
into one measurable, fault-tolerant platform with this dependency direction:

```text
hardware samples
    -> centralized sensor snapshots
    -> state estimation and observers
    -> robot health and motion constraints
    -> drivetrain and mechanism control
    -> autonomous executive and driver assistance

telemetry, replay, timing instrumentation, and benchmarks observe every layer
without becoming owners of match-critical hardware.
```

The target is not a rewrite. Existing native autonomous routines, LemLib
integration, motor-encoder fallback, Shadow Auton, GUI diagnostics, and both
robot configurations remain in service until replacements pass their stated
host and physical gates.

## Repository Baseline

The design was produced from the `Testing` branch at source commit
`ac0fedcb1627a7fd0208ca8eaf62a3e2f9117f3c`, plus the documentation-only
real-time audit checkpoints beginning at `94c4c8f`.

The current repository already provides:

- one process-wide legacy `aon::core::Hardware` owner;
- lazy LemLib chassis construction and a monitored `aon::auton::Actions` seam;
- typed motion failure, cancellation, and exclusive drivetrain action state;
- conservative, physically gated motor-encoder fallback;
- fixed-capacity Shadow capture, processing, checksummed storage, and playback;
- a bounded big-robot sorter state machine with acknowledged motor ownership;
- robot-specific hardware maps and controller/fallback parameters;
- native and LemLib autonomous paths with explicit experimental-route gates;
- Brain GUI selection, status, live graphing, field mapping, and debug tools;
- dependency-free host tests for important pure policies and codecs.

These are foundations to deepen, not systems to duplicate.

## Phase 0: Safety-State Reconciliation

Before competition code changes, reconcile the branch with its handoff and
physical evidence.

Current Git evidence conflicts with `docs/CURRENT_HANDOFF.md`: the handoff says
Shadow playback remains unauthorized, while the small-robot
`RobotConfig::shadowPlaybackAuthorized` value is currently `true` following a
later supervised-test checkpoint. The handoff's selected test slot also
predates later autonomous registrations.

Phase 0 must:

1. enumerate every current authorization flag and selected route for both
   robots;
2. map each enabled flag to a committed physical checklist and measured result;
3. fail closed where an enabled behavior has no recorded authorization;
4. update `CURRENT_HANDOFF.md` to match Git and recorded evidence;
5. complete the pending LemLib/native/Shadow physical baseline before changing
   device ownership or enabling new motion behavior;
6. preserve Controller-X cancellation and all native fallbacks throughout.

No host test or successful build substitutes for these measurements.

## Architectural Principles

### One owner, many observers

Each physical motor or sensor has one construction owner and one command owner
at a time. A sampler may publish immutable observations to many consumers.
Health, telemetry, GUI, replay, and benchmarks never command hardware.

The existing dual-robot production architecture plan remains the migration
path from `USING_BIG_ROBOT` and overlapping legacy/LemLib device construction
to two explicit composition roots. Its physical ownership gate remains a hard
stop.

### Pure policy behind narrow adapters

Numerical algorithms and state machines consume value snapshots, timestamps,
and configuration. PROS-facing adapters perform device I/O. This follows the
successful Shadow pattern and allows deterministic host tests.

### Immutable cross-task snapshots

Cross-task state is published by value with one timestamp and sequence number.
Examples include `SensorSnapshot`, `PoseEstimate`, `DrivetrainObservation`,
`MechanismHealthSnapshot`, and `RobotHealthSnapshot`. Readers never assemble a
logical sample through several separately locked getters.

### Bounded match behavior

All buffers, queues, retries, waits, recovery attempts, and autonomous actions
are bounded. High-frequency code does not perform filesystem I/O, unbounded
logging, or repeated heap allocation. Failure exits command a defined safe
state.

### Explicit degradation

Faults have stable codes, source, severity, freshness, first/last timestamps,
and occurrence counts. Systems distinguish ready, degraded, faulted, and
critical states. They never collapse all failure into one Boolean.

### Measurement before optimization

Runtime changes require before/after evidence. The real-time audit identifies
confirmed correctness risks and profiling candidates, but it does not invent
V5 timing values. Instrumentation precedes broad scheduling or performance
changes.

## Shared Foundation

The twelve phases share the following primitives.

### Time and freshness

Provide wrap-safe monotonic timestamp and deadline helpers, plus timestamped
sample metadata:

```cpp
struct SampleMetadata {
  std::uint32_t timestampMs;
  std::uint32_t sequence;
  std::uint32_t expectedPeriodMs;
  DataSource source;
  SampleState state;
};
```

Freshness is derived from timestamps rather than assumed task periods. Replay
uses an injected clock with identical wrap and deadline semantics.

### Fault vocabulary

Use one stable `FaultCode` domain with subsystem ranges and one `FaultRecord`
envelope:

```cpp
enum class FaultSeverity : std::uint8_t { Info, Warning, Fault, Critical };

struct FaultRecord {
  FaultCode code;
  FaultSeverity severity;
  SubsystemId subsystem;
  std::uint32_t firstObservedAt;
  std::uint32_t lastObservedAt;
  std::uint32_t occurrences;
  bool active;
  bool latched;
};
```

Phase-specific enums may describe local state, but telemetry, health, GUI, and
autonomous decisions exchange stable fault records.

### Operation results

Build on `MotionFailureReason`, `MotionResult`, Shadow `ResultCode`, and sorter
fault state without forcing their immediate removal. The autonomous executive
uses a common result envelope with a high-level outcome and subsystem detail:

```cpp
enum class ActionOutcome : std::uint8_t {
  Success,
  Timeout,
  Blocked,
  PoseUnreliable,
  SensorFailure,
  MechanismFailure,
  Cancelled,
  InvalidCommand,
  Busy,
  Unsupported
};

struct ActionResult {
  ActionOutcome outcome;
  FaultCode detail;
  std::uint32_t startedAt;
  std::uint32_t finishedAt;
  bool recoveryAttempted;
};
```

Adapters translate existing results during incremental migration.

### Bounded data structures

Fixed-capacity queues and ring buffers expose capacity, current occupancy,
high-water mark, dropped samples, and overruns. Writers never wait indefinitely
for telemetry consumers. Match configuration drops low-priority telemetry
before delaying control.

### Online statistics

Timing, characterization, and benchmark statistics use allocation-free online
accumulators for count, minimum, maximum, mean, and variance. Percentiles use a
bounded histogram or bounded sample set with an explicit memory budget.

### Ownership

Extend the intent of `MotionState` into explicit subsystem leases. A lease is
non-copyable, bounded to one owner, and releases into a defined safe state.
Cancellation must atomically defeat later commands from the cancelled owner.

## Dependency Graph

```text
Phase 0 safety reconciliation and physical baseline
                         |
                         v
dual-robot composition and single-device ownership
                         |
                         v
common time / fault / result / snapshot / buffer / ownership primitives
       |                 |                         |
       v                 v                         v
Phase 9 instrumentation  sensor snapshot layer     telemetry event envelope
       |                 |                         |
       +--------+--------+-------------------------+
                v
Phase 1 characterization and feedforward
                |
                v
Phase 2 pose estimator and uncertainty
                |
                v
Phase 3 field-based correction
                |
                +------------------+
                v                  v
Phase 5 drivetrain observer   Phase 8 health foundation
                |                  ^
                v                  |
Phase 6 mechanism observers -------+
                |
                v
Phase 11 adaptive constraints
                |
                v
Phase 4 autonomous executive
                |
                +----------> Phase 7 driver assistance
                |
                v
Phase 10 general telemetry and replay
                |
                v
Phase 12 performance benchmarking
```

Health is delivered in two passes: shared health types and aggregation early,
then complete localization, drivetrain, mechanism, storage, and timing inputs
after those producers exist.

## Implementation Sequence

The numbered feature phases are retained for traceability, but implementation
follows their dependencies:

1. Phase 0 safety-state reconciliation and existing physical baseline.
2. Safe prerequisites from the dual-robot production architecture plan.
3. Shared primitives and Phase 9 runtime instrumentation.
4. Central sensor snapshots and Phase 8 health foundation.
5. Phase 1 characterization framework and host analysis.
6. Physical characterization and reviewed constant activation.
7. Phase 2 pose estimation with uncertainty.
8. Phase 3 field-based correction.
9. Phase 5 drivetrain observer.
10. Phase 8 full health aggregation.
11. Phase 6 mechanism observers and bounded recovery.
12. Phase 11 adaptive motion constraints.
13. Phase 4 autonomous executive and incremental route migration.
14. Phase 7 driver assistance.
15. Phase 10 general telemetry and replay.
16. Phase 12 repeated-trial benchmarking.

## Phase Designs

### Phase 9: Real-Time and Memory Instrumentation

Instrument before changing loop scheduling. A pure `LoopStatistics` core
accepts target period, release timestamp, start timestamp, and finish timestamp
and maintains bounded aggregate statistics. A PROS adapter supplies monotonic
timestamps and optional stack information.

Named loop monitors cover drivetrain supervision, encoder fallback, legacy
odometry while retained, Shadow sampling, sensor snapshots, pose estimation,
health, mechanisms, GUI, and telemetry writer. Each monitor reports deadline
misses and maximum execution time without printing from its high-frequency
path.

The first correctness checkpoints address the audit's confirmed risks:

- coherent native odometry snapshot and valid mutex ownership;
- atomic or synchronized cross-task state instead of `volatile` data races;
- a yielding, edge-aware Controller-X safety loop;
- measurement of relative-delay loops before changing release scheduling.

Physical validation verifies loop rates, maximum execution times, stack
headroom, and unchanged autonomous behavior on both robots.

### Phase 1: Automated Characterization and Feedforward

Create a reusable experiment description independent of drivetrain hardware:

```cpp
enum class CharacterizationDirection : std::int8_t { Reverse = -1, Forward = 1 };
enum class CharacterizationProfile : std::uint8_t { Quasistatic, Dynamic };

struct CharacterizationSample {
  std::uint32_t timestampMs;
  float batteryVoltage;
  float commandedVoltage;
  float leftVelocity;
  float rightVelocity;
  float leftPosition;
  float rightPosition;
  float imuHeading;
  float imuAcceleration;
  SampleValidity validity;
};
```

The runner owns no motor directly. It obtains a drivetrain lease, applies a
bounded voltage schedule through the drivetrain adapter, samples the central
snapshot, stops on cancellation/fault/capacity, and always zeros the drive.

A host tool validates monotonic time, direction, voltage range, sufficient
excitation, finite data, sample count, and residual quality before fitting kS,
kV, and kA. Generated constants are proposed as review artifacts; they are
never activated automatically. Each robot configuration stores independent
values, validity metadata, dataset checksum, and approval state.

Feedforward is added to the existing feedback command:

```text
motor command = feedforward(reference velocity, acceleration)
              + existing feedback(error)
```

The framework supports later single-axis mechanism characterization without
coupling the analyzer to LemLib.

### Phase 2: Pose Estimation With Uncertainty

Introduce a centralized estimator separate from controllers. Measurements
carry source, timestamp, observed dimensions, expected variance, validity, and
health state. The first implementation uses a small deterministic covariance
representation for x, y, and heading; it does not require a general dynamic
matrix library.

```cpp
enum class PoseQuality : std::uint8_t { Good, Degraded, Poor, Invalid };

struct PoseUncertainty {
  float xVariance;
  float yVariance;
  float headingVariance;
};

struct PoseEstimate {
  Pose pose;
  PoseUncertainty uncertainty;
  PoseQuality quality;
  std::uint32_t timestampMs;
  std::uint32_t sequence;
};
```

Prediction consumes time-stamped wheel/heading deltas. Updates reject stale,
non-finite, impossible, or statistically inconsistent measurements. Tracking
odometry remains the initial primary source, motor encoders provide a degraded
source, and future absolute observations use the same update interface.

LemLib and native controllers initially continue consuming their existing
pose. An adapter publishes estimator quality to autonomous code before any
controller is switched to the new estimate. Controller migration requires a
separate physical checkpoint.

### Phase 3: Field-Based Pose Correction

Represent an observation by the state dimensions it constrains. A wall range
may update x or y without pretending to observe heading; a landmark bearing
may update heading without overwriting position.

```cpp
enum class ObservedDimension : std::uint8_t { X = 1, Y = 2, Heading = 4 };

struct FieldObservation {
  ObservationType type;
  std::uint8_t observedDimensions;
  std::array<float, 3> value;
  std::array<float, 3> variance;
  std::uint32_t timestampMs;
  ObservationSource source;
};
```

Sensor-specific adapters convert raw measurements using reviewed robot and
field geometry. The estimator applies per-dimension innovation thresholds and
records accepted/rejected observations and reasons. Corrections never directly
overwrite LemLib odometry.

### Phase 5: Slip, Collision, and Stall Detection

The drivetrain observer consumes one synchronized sample containing requested
wheel velocity, effective motor command, measured motor velocity/current/
position, estimated chassis velocity, IMU acceleration, and pose quality.

Detection is separated from response. Pure state machines use persistence and
hysteresis for normal, suspected, and confirmed states. Faults include wheel
slip, blocked robot, external push, motor stall, collision, and sensor
disagreement. Evidence and confidence are included in the immutable output.

Controllers may later inhibit integral accumulation or reduce acceleration;
the executive may abort or recover. The observer itself never commands motors.

### Phase 8: Central Robot Health Manager

The health manager aggregates immutable producer snapshots; it does not poll
all hardware itself. It monitors battery, motor electrical/thermal state,
sensor freshness, localization quality, SD/Shadow state, task timing,
drivetrain observations, and mechanism faults.

```cpp
enum class HealthStatus : std::uint8_t { Ready, Degraded, Fault, Critical };

struct RobotHealthSnapshot {
  HealthStatus overall;
  HealthStatus drivetrain;
  HealthStatus localization;
  HealthStatus sensors;
  HealthStatus mechanisms;
  HealthStatus storage;
  HealthStatus timing;
  std::array<FaultRecord, kMaximumActiveFaults> faults;
  std::size_t faultCount;
  std::uint32_t timestampMs;
  std::uint32_t sequence;
};
```

Aggregation rules are deterministic and host-tested. The GUI shows concise
active faults, while match logging defaults to warnings and worse. Autonomous
actions query health preconditions before risky operations and receive a
specific rejection result.

### Phase 6: Mechanism Observers and Jam Recovery

Split mechanism observation from command policy. A reusable observer compares
requested state, motor velocity/current/position, sensor transitions, elapsed
time, and configured expectations. It emits normal, transitioning, suspected
jam, jammed, stalled, sensor disagreement, and failed-transition states.

Recovery is an explicit bounded command sequence with a strict attempt count
and deadline. Intake recovery stops, reverses briefly, stops, retries forward,
and verifies movement. Every exit stops or holds the mechanism in its defined
safe state. The autonomous executive sees `MechanismFailure` after exhausted
retries.

Existing big-sorter acknowledgement and motor mutex behavior are preserved and
adapted first. Small-robot scan/sort blocking pulses are migrated to timestamped
state machines only after characterization tests preserve their behavior.

### Phase 11: Adaptive Motion Constraints

A pure constraint manager composes independent constraints by taking the most
conservative valid limit for each dimension:

```cpp
struct MotionConstraints {
  float maximumLinearVelocity;
  float maximumLinearAcceleration;
  float maximumAngularVelocity;
  float maximumAngularAcceleration;
};
```

Inputs include path curvature, battery state, mechanism configuration, pose
quality, drivetrain observation, current limits, and robot identity. Invalid or
stale inputs produce conservative configured limits, never aggressive defaults.

Path geometry remains separate. A narrow LemLib adapter applies limits only at
supported seams; vendored LemLib is not forked.

### Phase 4: Fault-Tolerant Autonomous Executive

The executive owns action lifecycle and results, not subsystem implementation.
It supports sequential and parallel composition, absolute deadlines,
cancellation, critical/optional classification, fallback, and recovery.

Parallel actions must declare resource requirements before starting. Conflicts
fail as `Busy` or `InvalidCommand`; they do not race motor ownership. A parent
propagates the first critical failure while still cancelling and safely joining
all active children.

Existing `Actions` motions, mechanism adapters, hybrid sequence, and Shadow
playback receive adapters. Native routes remain registered until migrated one
routine at a time and physically validated. Every executive exit invokes a
single safe-stop coordinator, including Controller-X cancellation.

### Phase 7: Driver Assistance

Create a deterministic pipeline:

```text
ControllerSnapshot -> DriverIntent -> enabled assist modules
                   -> MotionConstraints -> DrivetrainCommand
```

Each assist has explicit engagement, disengagement, freshness, and manual
override rules. Driver intent always wins. Heading hold, snap heading,
straight-drive correction, precision mode, braking, alignment, and anti-tip
limiting are independent modules with robot-specific enable flags.

Assists never operate during autonomous ownership. The controller/Brain shows
engaged status without adding high-rate display work.

### Phase 10: Telemetry Recording and Replay

General telemetry builds on Shadow's bounded and checksummed practices without
changing the Shadow route format. A versioned frame contains timestamp,
sequence, channel identifier, payload version, and bounded payload.

Producers publish without filesystem access. A lower-priority recorder drains
a bounded queue, decimates configured channels, writes bounded chunks, and
reports drops and SD faults. Competition defaults record a small diagnostic
set; debug configurations opt into high-rate channels.

Host replay validates the file, reconstructs timestamp order, and feeds sensor
snapshots into selected pure algorithms under an injected clock. Recorded field
data becomes versioned test fixtures. Compatibility adapters retain useful old
formats when their cost is bounded and tested.

### Phase 12: Automated Performance Benchmarking

A benchmark description names robot configuration, initial pose, action,
repetitions, thresholds, and telemetry channels. Statistics use repeated valid
trials and preserve failures rather than dropping them from averages.

Results include endpoint and path errors, execution time, success/timeout
counts, current and battery sag, slip events, health faults, and deadline
misses. Machine-readable output has a schema version and configuration digest;
human summaries are generated from the same result object.

Controller comparisons require compatible robot configuration, dataset, and
trial protocol. One better run never authorizes a controller change.

## Data Rates and Scheduling

Each producer documents a target rate and maximum age. Initial rates are
starting points for measurement, not vendor claims:

- driver input and drivetrain command: 10 ms;
- drivetrain sensor snapshot and motion observer: 20 ms;
- pose estimator prediction: 10-20 ms based on the single sensor publisher;
- mechanism state machines: 10-50 ms by mechanism dynamics;
- health aggregation: 50-100 ms;
- Brain GUI: 100 ms or slower;
- SD telemetry drain: lower priority, bounded by queue and chunk size.

Deadline scheduling is introduced loop by loop only after baseline timing is
captured. Overruns increment counters and never trigger an unbounded catch-up
loop.

## Memory Policy

Every implementation plan includes a static memory budget. Large arrays live
in process-lifetime workspaces rather than task stacks. Snapshot types remain
small enough to copy predictably. Queue payloads have compile-time maximums.

The Shadow service's existing large capture, route, workspace, playback, and
256 KiB codec buffers must be counted before allocating general telemetry or
replay buffers. Workspaces should be reused where lifetimes do not overlap,
but only after concurrency analysis proves that reuse is safe.

Heap allocation may remain at initialization or noncritical UI boundaries when
bounded and justified. It is removed from high-frequency paths touched by a
phase when a fixed representation is practical.

## Concurrency Policy

- Construct each physical device exactly once after the ownership migration.
- Publish complete snapshots under one synchronization boundary.
- Do not use `volatile` as task synchronization.
- Use atomics for small independent flags/counters and mutexes for invariants.
- Bound mutex waits in periodic tasks and count acquisition failures.
- Never hold a telemetry or GUI lock while issuing motor or filesystem calls.
- Give cancellation higher priority than later commands from the old owner.
- Declare resource needs before starting parallel autonomous actions.
- Make reset/update protocols explicit for odometry and estimators.

## Failure Behavior

Every subsystem documents:

1. invalid input behavior;
2. stale input behavior;
3. cancellation behavior;
4. timeout behavior;
5. safe motor state;
6. latched versus self-clearing faults;
7. diagnostic snapshot fields;
8. whether degraded operation is permitted;
9. the physical validation required before enabling it.

No failure path may wait or retry indefinitely. Telemetry failure never blocks
control. Invalid estimation never silently becomes a high-confidence pose.

## Testing Strategy

### Host tests

Pure tests cover time wrap, freshness, fault aggregation, bounded buffers,
statistics, feedforward regression, estimator math, innovation rejection,
observer persistence, recovery limits, executive composition, assist
arbitration, constraints, codec compatibility, replay timing, and benchmark
statistics.

Tests include invalid/non-finite inputs, stale samples, sensor faults, timeout,
cancellation, state transitions, numerical boundaries, failure propagation,
and resource conflicts.

### Embedded builds

Every robot-code checkpoint receives clean small- and big-robot builds with
warnings reviewed. The committed default remains the expected small-robot
configuration until the dual-target architecture removes source-edited target
selection.

### Physical gates

Physical gates cover authorization reconciliation, existing LemLib/native and
Shadow baselines, ownership migration, characterization, estimator drift and
correction, induced drivetrain and mechanism faults, executive safe exits,
driver-assist feel, runtime/stack measurements, SD load, adaptive constraints,
and repeated benchmarks.

Measured failures are recorded; they are never replaced by assumed success.

## Observability

Every major snapshot answers:

- what state is active;
- what timestamp/sequence produced it;
- what inputs were accepted or rejected;
- what output or decision was produced;
- whether each source is fresh;
- which fault is active;
- why the latest operation failed.

Telemetry levels are `Error`, `Warn`, `Info`, `Debug`, and `Trace`.
Competition defaults to low-overhead warnings and selected measurements.

## Checkpoint and Git Policy

One coherent subphase is one checkpoint. Before each robot-code commit:

1. run relevant host tests;
2. clean-build the small robot;
3. clean-build the big robot;
4. restore the expected committed target;
5. review concurrency, memory, and failure behavior;
6. document remaining physical validation;
7. update `docs/CURRENT_HANDOFF.md`;
8. stage only intended files;
9. commit descriptively;
10. push `Testing` without force.

Documentation-only design and plan checkpoints run path, placeholder,
consistency, and `git diff --check` verification before commit.

## Delivery Documents

This roadmap is followed by separate implementation plans for:

1. `2026-08-10-safety-state-reconciliation-and-baseline.md`;
2. `2026-08-10-dual-robot-roadmap-prerequisites.md`;
3. `2026-08-10-phase-09-runtime-instrumentation.md`;
4. `2026-08-10-phase-01-characterization-feedforward.md`;
5. `2026-08-10-phase-08-robot-health.md`;
6. `2026-08-10-localization-ekf.md` for Phase 2;
7. `2026-08-10-phase-03-field-pose-correction.md`;
8. `2026-08-10-phase-05-drivetrain-observer.md`;
9. `2026-08-10-phase-06-mechanism-observers.md`;
10. `2026-08-10-phase-11-adaptive-constraints.md`;
11. `2026-08-10-phase-04-autonomous-executive.md`;
12. `2026-08-10-phase-07-driver-assistance.md`;
13. `2026-08-10-phase-10-telemetry-replay.md`;
14. `2026-08-10-phase-12-performance-benchmarking.md`.

Each plan must identify exact files, interfaces, test commands, clean-build
commands, physical gates, handoff changes, and frequent commit boundaries.

## Success Criteria

- Sensor values enter one timestamped snapshot path rather than being
  independently assembled by every consumer.
- Autonomous code can distinguish motion, pose, sensor, mechanism, timeout,
  cancellation, and ownership failures.
- Every drivetrain and mechanism command has one explicit owner.
- Pose estimates include calibrated uncertainty and observable quality.
- Health snapshots expose specific faults and freshness.
- Observers detect persistent conditions without single-sample triggers.
- All recoveries, waits, retries, queues, and recordings are bounded.
- Runtime timing and memory behavior are measured on the V5 Brain.
- General telemetry replays real traces through host-testable algorithms.
- Benchmarks compare repeated trials with machine-readable results.
- Existing native routes and safety gates remain until replacements pass
  physical validation.

## Out of Scope

- Rewriting or forking vendored LemLib, PROS, LVGL, fmt, or JSON code.
- Removing native autonomous fallbacks before validated replacements exist.
- Automatically activating fitted constants or controller configurations.
- Claiming physical performance from compilation, simulation, or host replay.
- Unbounded full-rate logging of every available channel during matches.
- A general-purpose dynamic robotics framework where fixed, explicit modules
  satisfy the two real robot configurations.
