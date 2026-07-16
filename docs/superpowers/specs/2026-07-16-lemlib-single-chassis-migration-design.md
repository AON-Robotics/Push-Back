# LemLib Single-Chassis Migration Design

## Goal

Make LemLib the only long-term owner of drivetrain motors, odometry sensors,
calibration, and autonomous motion while keeping the existing Kevin Loader and
Kevin Park routines available as temporary rollback paths.

## Current Problem

The program constructs both a legacy drivetrain/odometry stack and a LemLib
chassis over the same physical motors, rotation sensors, and IMU. Normal startup
currently runs both odometry loops even when only LemLib is used. The two motion
systems also use different controllers, coordinate conventions, completion
rules, and task lifecycles. This makes physical results difficult to attribute
to one configuration.

The first LemLib loader experiment was also tested before normal LemLib
calibration and lateral control were enabled. Its failure is not a valid result
for the current chassis configuration.

## Target Architecture

- `aon::lemlib_integration::chassis()` owns drivetrain motion and odometry.
- PROS owns the competition callbacks, tasks, motors, sensors, and mechanisms.
- Autonomous route bodies call `aon::auton::Actions` and named mechanism
  actions; they do not command legacy drivetrain globals directly.
- Driver control continues using LemLib curvature for the small robot.
- GUI selection stores routine identity and dispatches named routines without
  changing chassis configuration.
- Native Kevin routes remain available during migration, but legacy odometry is
  started only when a native route is actually invoked.
- After both Kevin routes have validated LemLib replacements, the legacy
  drivetrain motion profiles, odometry task, duplicate sensor objects, and
  Okapi compatibility dependencies are removed from the active robot path.

## Migration Phases

### Phase 1: Isolate LemLib Validation

Remove unconditional legacy odometry startup. Add an idempotent legacy-motion
preparation function that starts the legacy odometry task before a native route
or native drivetrain test. LemLib validation then runs with only LemLib
odometry, while Kevin fallbacks remain callable.

Physical gate: run the 12-inch LemLib test and one native fallback after a
reboot. Stop if the GUI, driver control, or native route initialization changes.

### Phase 2: Validate LemLib Primitives

Provide separate forward, reverse, clockwise-turn, counterclockwise-turn, and
drive-turn-drive tests. Each test logs expected and actual pose. Expose one test
at a time and require repeatable physical results before advancing.

Calibration categories are changed separately: sensor reversal, tracking-wheel
diameter, tracking offsets, drivetrain geometry, lateral PID, angular PID, and
slew. No commit combines unrelated categories.

### Phase 3: Complete the Autonomous Action API

Add `turnToPoint`, asynchronous motion, `waitUntil`, `waitUntilDone`, and
explicit cancellation. Synchronous behavior remains the default for existing
routes. Async calls are used only when a mechanism must activate during motion.

### Phase 4: Migrate Kevin Park

Kevin Park is the first route migration because it has fewer movements and one
park mechanism action. Keep the native implementation under an explicit
fallback name until the LemLib implementation passes repeated tests.

### Phase 5: Migrate Kevin Loader

Translate the route in independently testable segments: loader approach,
collection, retreat, scoring approach, score, and parking setup. Measure actual
field poses instead of converting legacy distances into guessed coordinates.
Advance only one segment per physical checkpoint.

### Phase 6: Remove Legacy Motion Ownership

After both LemLib replacements pass, remove the legacy odometry task and
duplicate drivetrain motion ownership. Delete unused native motion profiles,
drivetrain PIDs, and autonomous calls. Keep mechanism implementations and any
compatibility code still required outside drivetrain motion.

### Phase 7: Repository Cleanup

Narrow headers, remove obsolete globals, separate validation routines from
competition routes, and update documentation. Structural cleanup must not be
combined with tuning changes.

## Safety and Rollback

- Every phase ends with a separate commit and both robot configurations build.
- `USING_BIG_ROBOT` is restored to `false` before each checkpoint.
- No full route is the first test of a chassis or calibration change.
- The test robot starts at reduced speed with clear space and an emergency stop.
- Native fallbacks are deleted only after their LemLib replacements pass.
- If risk exceeds the benefit at a phase gate, stop and request approval.

## Success Criteria

- Only one odometry task runs during LemLib autonomous and driver testing.
- Forward/reverse distance and both turn directions are repeatable.
- GUI selection and SD persistence continue working.
- Kevin Park and Kevin Loader have physically validated LemLib replacements.
- No active autonomous route calls legacy drivetrain motion APIs.
- Both hardware configurations compile from a clean build.

## Out of Scope

- Mirroring field routes before the original side is validated.
- Rewriting intake scanning or sorting concurrency.
- Changing mechanism behavior during drivetrain migration.
- Removing native fallbacks before physical replacement tests pass.
- Editing vendored PROS, LemLib, LVGL, fmt, or JSON code.
