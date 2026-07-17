# Motor-Encoder Odometry Fallback Design

## Goal

Keep LemLib tracking-wheel odometry as the normal autonomous motion source, but
allow the robot to stop, latch into a conservative motor-encoder mode, and retry
one interrupted motion when the tracking sensors genuinely fail. Also provide a
brain-screen control that forces encoder mode before autonomous starts.

The fallback is a safety and continuity feature. It is not expected to preserve
tracking-wheel accuracy after wheel slip or mechanism contact.

## Relationship to the LemLib Migration

This feature extends the existing `aon::auton::Actions` seam created for the
single-chassis migration. Route bodies continue to express named autonomous
actions and do not choose a controller directly. LemLib remains the only normal
owner of drivetrain motion and tracking-wheel odometry. The fallback controller
may command the same drivetrain motors only after the active LemLib motion has
been cancelled and the drivetrain has been stopped.

The pending Task 1 physical gate in the migration plan remains mandatory. This
design may be implemented and bench-tested separately, but it does not authorize
further calibration or route migration without the recorded 12-inch LemLib and
native Kevin fallback results.

## Scope

The first implementation covers the motion primitives used by the current
LemLib validation and staged-route code:

- `moveToPoint`
- `moveToPose`
- `turnToHeading`
- timed `arcadeFor`

`setPose`, cancellation, braking, and logging remain available. An arbitrary
LemLib path asset cannot be reconstructed safely from the current action
interface, so `followPath` cancels and reports an unsupported fallback instead
of guessing a replacement trajectory. Path fallback can be designed later if a
competition route actually needs it.

Native Kevin Loader and Kevin Park are unchanged. They continue to use the lazy
legacy-motion compatibility path and are not wrapped by this fallback.

## Operating Modes

The fallback service has three process-lifetime states:

1. `Tracking`: LemLib tracking-wheel odometry is active and health monitoring is
   enabled.
2. `ForcedEncoder`: the brain GUI selected encoder mode before autonomous.
3. `FaultedEncoder`: a tracking fault was confirmed during autonomous.

`ForcedEncoder` and `FaultedEncoder` are latched. The service never switches
back to tracking mode until the brain restarts. This prevents pose-source
bouncing and makes test results reproducible.

The brain GUI presents a two-state control in the autonomous menu:

- `AUTO FALLBACK` (default): begin in `Tracking` and monitor sensor health.
- `FORCE ENCODERS`: begin in `ForcedEncoder` and bypass tracking-based motion.

The setting is intentionally not written to the SD card. A restart always
returns to the safer, higher-accuracy default and requires an explicit choice
to force encoders again. The selected mode is visible on the main and autonomous
screens.

## Module Boundaries

### Drive Hardware Access

The LemLib integration module continues to construct the left and right motor
groups, rotation sensors, IMU, and chassis exactly once. It exposes narrow
read/command operations needed by monitoring and fallback control rather than
allowing a second module to construct duplicate devices:

- sample left/right motor encoder positions;
- sample left/right/back tracking positions and IMU rotation/status;
- command left and right drivetrain output;
- stop and set drivetrain brake mode.

This keeps physical device ownership in one place and makes it impossible to
run two controller loops intentionally.

### Motion Health Monitor

A motion health monitor evaluates synchronized tracking, IMU, motor-encoder,
and LemLib-pose samples while an action is active. It returns a typed failure
reason but does not command motors or change modes.

Health thresholds live in the active robot configuration so the small and big
robots can be validated separately. Initial values are conservative and must be
bench-tested before field use.

### Encoder Fallback Controller

A small relative-motion controller owns one action at a time after LemLib has
been cancelled. It tares or snapshots motor positions at the transition and
uses deltas, not absolute encoder values. It provides:

- relative forward/reverse distance control from the average left/right motor
  travel;
- heading control from a healthy IMU;
- differential motor-encoder heading estimation when the IMU is unavailable;
- reduced-speed point, pose, and heading operations;
- explicit timeout, cancellation, and brake-on-exit behavior.

The controller does not claim a globally accurate field pose. It maintains a
dead-reckoned diagnostic estimate anchored at the last trusted LemLib pose.

### Resilient Actions

`aon::auton::Actions` remains the only interface used by LemLib route bodies.
For monitored actions it:

1. captures the requested target and last trusted pose;
2. starts the LemLib motion asynchronously;
3. samples health while waiting for completion;
4. returns success normally when LemLib finishes;
5. on a confirmed fault, cancels all LemLib motion and commands zero output;
6. waits for the drivetrain to settle;
7. latches `FaultedEncoder` with a typed reason;
8. computes the remaining relative command from the last trusted pose and the
   original target;
9. retries that action once through the encoder controller with a maximum of
   60 percent of the requested output;
10. returns a structured result and leaves the drivetrain braked.

Actions started in `ForcedEncoder` skip steps 2 through 7 and execute once
through the encoder controller. There is no second retry in forced mode.

Motion methods return a `MotionResult` containing success/failure, active mode,
whether fallback was used, and a failure reason. Autonomous routines propagate
failed required actions instead of always returning success. This prevents the
GUI from displaying `Completed` after an aborted fallback.

## Fault Detection

Automatic switching must detect sensor failure, not ordinary tracking error.
The monitor therefore uses direct device validity and disagreement between
independent sensor families.

A fault is confirmed only by one of these conditions:

- a rotation sensor or IMU repeatedly returns the PROS error sentinel or an
  invalid/non-finite value;
- LemLib pose becomes non-finite;
- a physically impossible pose discontinuity persists across consecutive
  samples;
- a required tracking channel remains effectively unchanged for a configured
  dwell period while the corresponding drivetrain motor encoders show enough
  sustained movement.

Device/pose errors require three consecutive samples. Frozen-sensor detection
uses a 20 ms sample interval and an initial 300 ms dwell, with minimum tracking
and motor deltas specified in robot configuration. Impossible-jump thresholds
are derived above the configured maximum drivetrain speed rather than from a
normal route error tolerance.

The monitor does not switch modes because:

- the final pose misses its target;
- the robot is blocked and both tracking and motor encoders remain still;
- normal wheel slip causes a plausible but inaccurate pose;
- a motion times out without direct sensor-failure evidence.

Those cases return the normal timeout or motion failure. They are valuable
diagnostics, but they do not prove odometry hardware failed.

## Retry Semantics

The interrupted action has exactly one fallback attempt.

For `moveToPoint`, the controller computes the target bearing and remaining
straight-line distance from the last trusted pose. It turns toward the target,
drives the remaining distance, and does not promise final field heading.

For `moveToPose`, it performs the point fallback and then turns to the requested
final heading.

For `turnToHeading`, it retries the remaining angular displacement. It uses the
IMU if the IMU is healthy; otherwise it uses calibrated left/right motor travel
and track width.

For `arcadeFor`, odometry is not required. The action continues using its timed
command and is stopped only by cancellation, timeout, or invalid drivetrain
motor feedback.

Fallback timeouts use only the unused portion of the original timeout plus a
small configurable transition allowance. A failure does not grant an
unbounded second motion window.

If fallback fails or times out, the drivetrain stops with hold or brake mode,
the result is marked failed, and the route must not execute later drivetrain or
mechanism actions that assume the target was reached.

## Concurrency and Safety

- LemLib is cancelled and zero output is confirmed before fallback commands
  are issued.
- Only one resilient action may own drivetrain output at a time.
- Mode and status snapshots are protected against the GUI and autonomous tasks
  reading them concurrently.
- A manual emergency cancel always wins and never triggers an automatic retry.
- Driver control initialization clears pending action ownership but does not
  silently clear a latched diagnostic state.
- Native legacy odometry is never started as part of encoder fallback.

## Diagnostics and Brain Display

Every transition writes a stable terminal record containing the action name,
mode, failure reason, trusted pose, target, tracking samples, motor samples, and
timestamp. Normal action finish logs add the final mode and fallback-used flag.

The brain display shows:

- selected startup mode before autonomous;
- `ODOM: TRACKING`, `ODOM: FORCED ENCODER`, or `ODOM: FAULT ENCODER`;
- a short failure reason after an automatic transition;
- failed routine status if the fallback attempt does not complete.

The display is diagnostic only and never controls the transition after an
autonomous action has started.

## Validation Strategy

Host-testable units use sample sequences and fake drive I/O to verify:

- transient bad samples do not trigger fallback;
- three consecutive invalid samples do trigger fallback;
- motor movement plus frozen tracking triggers only after the dwell time;
- blocked motion with stationary motor encoders does not trigger fallback;
- impossible jumps are rejected conservatively;
- mode latching prevents return to tracking;
- exactly one retry is allowed;
- retry speed is capped and timeout remains bounded;
- failed retry propagates to routine status.

Robot testing proceeds through physical gates:

1. Complete the existing migration Task 1 physical gate with fallback disabled.
2. Bench-test `FORCE ENCODERS` using short forward, reverse, and turn validation
   routines at reduced speed.
3. Simulate a disconnected tracking sensor with wheels off the floor and prove
   the robot stops before the fallback controller starts.
4. In clear floor space, repeat one short motion with a controlled sensor fault
   and verify the single reduced-speed retry.
5. Test a blocked drivetrain and verify it times out without falsely declaring
   odometry failure.
6. Clean-build both robot configurations at every code checkpoint and restore
   `USING_BIG_ROBOT false` before committing.

No competition route uses automatic fallback until its primitive tests pass on
the corresponding physical robot.

## Success Criteria

- Normal LemLib validation behaves identically when all sensors are healthy.
- The brain can force motor-encoder mode before autonomous without SD-persisting
  that risky choice.
- Confirmed tracking failure cancels LemLib, stops the drivetrain, latches
  encoder mode, and retries the interrupted supported action once at reduced
  speed.
- Sensor glitches, wheel slip, a blocked robot, and ordinary motion timeout do
  not cause a false mode switch.
- A failed fallback prevents dependent route actions and marks the routine
  failed.
- Logs and the brain screen identify the active mode and transition reason.
- Native Kevin fallbacks and the single-chassis migration phase gates remain
  intact.

## Out of Scope

- Matching tracking-wheel field accuracy with motor encoders.
- Recovering an arbitrary `followPath` asset.
- Automatically switching back to tracking odometry.
- Detecting ordinary wheel slip as an odometry hardware failure.
- Changing intake, scoring, or other mechanism behavior.
- Removing native Kevin fallbacks or legacy compatibility code.
- Editing vendored LemLib, PROS, LVGL, or other third-party code.
