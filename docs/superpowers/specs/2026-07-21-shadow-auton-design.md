# Shadow Auton Design

## Status

Approved in conversation on 2026-07-21. This document defines the design only;
it does not authorize closed-loop playback before the physical gates below pass.

## Purpose

Shadow Auton records a driver-operated route, including robot motion and every
driver-controlled mechanism action, into one of three SD-card slots. A later
autonomous run replays the route with closed-loop odometry-based path following
instead of replaying joystick values by time.

The first delivery stage records, validates, saves, loads, and displays routes.
Closed-loop playback remains locked until the existing LemLib odometry baseline
and the recording-only hardware tests pass.

## Goals

- Provide three recordings controlled from the V5 Brain screen.
- Record LemLib pose at 20 ms intervals for up to 60 seconds.
- Record raw driver inputs for diagnostics and route-direction inference.
- Record every driver-controlled mechanism transition on both robot variants.
- Save recordings atomically and detect missing, truncated, corrupt, or
  incompatible files before playback.
- Convert recorded motion into direction-aware LemLib path segments.
- Reproduce movement, stops, and mechanism actions in their recorded order.
- Cancel safely from Controller X, robot disable, unhealthy odometry, or a
  failed motion action.
- Preserve the existing autonomous selector, native routines, and locked motor-
  encoder fallback behavior.

## Non-goals

- Raw time-based joystick replay as the primary autonomous controller.
- Automatically driving from an arbitrary location to the recorded start.
- Silently continuing after odometry becomes unhealthy.
- Enabling automatic motor-encoder fallback or dependent encoder routes.
- Editing a recording or generated path on the Brain in the first version.
- Recording more than one driver controller.

## User Workflow

The Brain exposes a `SHADOW` screen with three slots. Each slot displays empty,
valid, invalid, or recording metadata. The actions are:

- `RECORD`: asks for overwrite confirmation when the slot already contains a
  valid recording, then starts capture during driver control.
- `STOP/SAVE`: stops capture, validates and processes it, then atomically saves
  it to the selected slot.
- `DELETE`: asks for confirmation, then removes the selected slot.
- `PLAY`: loads and validates the slot. It is visibly disabled while playback
  authorization is false.

Before playback, the screen shows the recorded duration and starting pose. The
operator must physically place the robot at that starting location and confirm.
Confirmation initializes LemLib to the recorded start pose; the robot never
drives itself to the start. In testing, confirmation immediately starts the
run. For a competition autonomous run, confirmation arms the selected slot
before the match; autonomous dispatch consumes that one-shot authorization.
Disabling, cancelling, changing slots, or restarting clears the authorization.

The UI uses explicit states: `NO SD`, `EMPTY`, `READY`, `RECORDING`,
`PROCESSING`, `SAVED`, `INVALID`, `PLAY LOCKED`, `PLAYING`, `CANCELLED`, and
`STOPPED`. Storage errors are never reported as a successful save.

## Architecture

### ShadowRecorder

`ShadowRecorder` owns a fixed-capacity in-memory capture buffer. While active,
it samples every 20 ms:

- elapsed milliseconds;
- LemLib X, Y, and heading;
- the four raw controller axes;
- the effective left and right drivetrain commands; and
- forward/reverse/stopped direction classification.

Mechanism transitions are appended to a separate fixed-capacity event buffer.
The operator-control layer reports semantic mechanism commands through one
small recorder interface rather than making the recorder inspect motors and
pistons. The vocabulary covers intake idle/store/corridor/reject/scoring and
sorting modes, scorer height, cart, trapdoor, lever, Brooks, Sem, and Arrow.
Unsupported mechanisms on a robot variant remain absent rather than becoming
synthetic no-op events.

The recorder accepts at most 3,000 samples and 512 mechanism events. Reaching
either limit stops capture and reports a capacity error; it never wraps or
silently drops old data.

### ShadowProcessor

`ShadowProcessor` is platform-independent and host-testable. It:

1. validates timestamps and pose samples;
2. normalizes headings across wraparound;
3. splits motion at direction changes, stationary intervals, and discontinuous
   samples;
4. removes redundant points while preserving corners, heading changes, segment
   endpoints, and mechanism-event anchors;
5. emits forward or reverse path segments in LemLib's text asset format; and
6. maps each mechanism event to a segment and progress anchor, retaining its
   elapsed-time offset for stationary sequences.

Stationary periods of at least 100 ms become explicit dwell segments. A
mechanism event that occurs during a dwell is replayed in timestamp order within
that dwell. Movement events are fired when the route reaches their progress
anchor, so ordinary drivetrain speed variation does not desynchronize them.

### ShadowStorage

`ShadowStorage` is the only module that accesses `/usd`. It manages:

- `/usd/aon-shadow-slot-1-a.bin` and `-1-b.bin`
- `/usd/aon-shadow-slot-2-a.bin` and `-2-b.bin`
- `/usd/aon-shadow-slot-3-a.bin` and `-3-b.bin`

Each file contains a fixed header, raw samples, semantic mechanism events,
processed path segments, and a checksum. Integers are serialized explicitly in
little-endian order and floating-point values have an explicit representation;
C++ structs are not written directly, avoiding padding and ABI dependence.

The header contains magic bytes, format version, monotonically increasing save
generation, robot identity, sample period, duration, start pose, section counts
and sizes, and checksum. Floating-point values use IEEE-754 binary32. The loader
rejects wrong magic, unsupported versions, the wrong robot identity, impossible
counts, size mismatches, non-finite values, or checksum mismatches.

Each slot uses two generations. Saving completely writes, flushes, closes,
reopens, and validates the older generation before it can supersede the current
one. Loading chooses the newest valid generation. A failed or interrupted write
therefore leaves the previous valid generation available without depending on
filesystem rename semantics. Delete removes both generations. SD installation
and every open, read, write, flush, close, and delete result are checked and
surfaced to the GUI.

### ShadowPlayer

`ShadowPlayer` owns playback state but delegates drivetrain commands to the
existing autonomous motion-control boundary. At dispatch it:

1. verifies playback authorization, competition state, cancellation state,
   file validity, robot identity, and odometry health;
2. requires the start-placement confirmation;
3. initializes LemLib to the recording's start pose;
4. creates in-memory LemLib assets from the validated processed segments;
5. follows each segment in its recorded direction; and
6. dispatches semantic mechanism events at progress anchors or within dwell
   segments.

The player does not call mechanisms from an SD parsing layer. A separate
`ShadowMechanisms` adapter maps validated semantic events to the same mechanism
operations used by normal robot code.

Playback is available from the testing UI and as a selectable autonomous
routine. Brain-screen `PLAY` cannot bypass competition-state safety checks.

## Safety Rules

- Recording is allowed only during driver control and ends automatically after
  60 seconds.
- Three consecutive invalid pose samples abort recording and mark the pending
  capture invalid.
- Pose jumps use the existing health thresholds and consecutive-sample policy;
  a discontinuity cannot be simplified into a drivable path.
- Playback initially has a compile-time or robot-configuration authorization
  flag set to false in both robot configurations.
- Playback requires healthy odometry at start and throughout every segment.
- Controller X calls the existing latched autonomous cancellation path, stops
  drivetrain and intake output, and prevents later commands in that dispatch.
- Disabling the robot performs the same stop without auto-resuming when the
  robot is enabled again.
- Any failed or timed-out motion stops the complete route; later mechanism
  events are not fired.
- The motor-encoder fallback remains independently locked. Shadow playback
  never authorizes it and never silently switches to it.
- Only one owner may command the drivetrain during playback.

## Integration Boundaries

- Operator control gains capture hooks after inputs have been interpreted, so
  semantic mechanism intent matches what the driver actually requested.
- Existing drive behavior is unchanged when recording is inactive.
- The normal GUI gains navigation to the Shadow screen without enabling the
  current `TESTING_AUTONOMOUS` mode, whose runtime behavior is separate.
- Valid Shadow slots are registered through the existing autonomous-selection
  mechanism without replacing Kevin or LemLib test routines during validation.
- The existing auton-selection SD file remains separate from Shadow files.

## Error Handling

All public recording, processing, storage, and playback operations return a
typed result with an error category suitable for both logs and short Brain
messages. Expected categories include no SD, read-only SD, open/read/write/
flush/close/rename failure, corrupt file, incompatible version, wrong robot,
capacity reached, invalid pose, discontinuity, no movement, playback locked,
unsafe state, cancelled, odometry failure, and motion failure.

A recording with valid mechanism events but no meaningful motion is permitted
for mechanism-only validation. An entirely empty recording is rejected.

## Verification Strategy

### Host tests

- serialization round trips and deterministic checksum;
- truncated, corrupt, oversized, wrong-version, and wrong-robot files;
- 20 ms sampling bounds and the 60-second limit;
- heading wraparound and non-finite pose rejection;
- redundant-point removal without losing corners or endpoints;
- forward/reverse transitions and stationary dwell extraction;
- mechanism ordering, progress anchoring, and stationary actions;
- capacity failures without buffer overwrite;
- playback state transitions, cancellation latching, and failure propagation;
- atomic-save behavior through an injected storage interface.

### Build verification

- warnings-as-errors host tests for platform-independent modules;
- clean small-robot and big-robot PROS builds;
- restore and rebuild the committed small-robot configuration.

### Physical gates

Physical testing is sequential and each result is recorded before the next:

1. Complete the existing `TEST LemLib 12in` and native Kevin baseline gate.
2. With playback locked, record and reload each SD slot across a full reboot.
3. Verify missing, read-only, and removed-card errors do not crash or claim a
   successful save.
4. Record a stationary mechanism-only sequence and inspect its saved metadata.
5. Record a short straight path, an L-shaped path, a reverse segment, and a
   pause, reviewing processed results without moving the robot autonomously.
6. Authorize playback only for an isolated short-path test with no mechanisms.
7. Test Controller X cancellation and robot disable during that path.
8. Add one mechanism type at a time, then validate a complete 15-second route.
9. Validate a 60-second skills recording only after the 15-second route is
   repeatable.

Failure at any gate relocks playback until the cause is diagnosed and the gate
is repeated. Compilation or host tests alone never authorize robot motion.

## Delivery Sequence

1. Pure data model, processor, serializer, and host tests.
2. SD storage with atomic writes and visible status.
3. Semantic mechanism capture hooks and fixed-capacity recorder.
4. Brain Shadow screen with three slots; playback remains locked.
5. Recording-only physical validation.
6. Closed-loop player and autonomous registration, still locked by default.
7. Short-path physical authorization and progressive mechanism validation.

This sequence deliberately makes useful SD recordings available before any new
autonomous drivetrain behavior is authorized.
