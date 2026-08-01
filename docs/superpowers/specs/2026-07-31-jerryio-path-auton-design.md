# JerryIO Path Autonomous Design

## Goal

Add a drive-only LemLib autonomous routine that follows
`static/path.jerryio.txt` and is selectable as Blue AUT3. The routine must not
operate the intake, loader, or scorer.

## Approach

Keep the path routine separate from the existing figure-eight validation and
red six-block autonomous. Define a small configuration type for the routine's
name, starting pose, lookahead, timeout, and path limits. The routine will use
the existing `ASSET(path_jerryio_txt)` declaration and the project's
`aon::auton::Actions` wrapper rather than calling the LemLib chassis directly.

This matches LemLib's documented pure-pursuit flow: set the chassis pose to the
start of the generated path, then call `follow` with a lookahead distance,
timeout, and travel direction.

## Runtime Behavior

1. Blue AUT3 starts `TEST JerryIO Path` through the standard `runRoutine`
   wrapper.
2. On the small robot, the routine logs its start and initializes odometry to
   `(-66.557, -35.024, 132 degrees)`. The heading follows the tangent formed by
   the first two generated points so the robot begins in the forward path
   direction.
3. The routine follows `path_jerryio_txt` forward with a 10-inch lookahead and
   a 14,000 ms timeout.
4. Path motion uses fail-closed odometry monitoring, matching the red six-block
   autonomous safety behavior.
5. The drivetrain stops in brake mode after success or failure. The routine
   returns success only when the wrapped path motion succeeds.
6. On the big robot, the routine reports that it is unsupported, stops the
   drivetrain, and returns failure, matching the existing LemLib validation
   routines.

No mechanism action is started or stopped by this routine because its current
scope is path validation only.

## Selector Change

Replace the LemLib figure-eight entry in Blue AUT3 with `TEST JerryIO Path`.
The figure-eight implementation and asset remain available in the codebase;
only its selector slot changes.

## Validation

Add a host-side test that verifies:

- the public routine signature and configuration values;
- the path has a valid LemLib `endData` terminator and finite numeric points;
- its first point matches the configured start position;
- the configured heading matches the initial path tangent within a small
  tolerance;
- the final point has zero speed and all generated speeds are within the
  configured range;
- the routine uses the correct asset, forward direction, timeout, lookahead,
  fail-closed monitoring, and brake stop; and
- Blue AUT3 and its GUI label select the new routine.

Run the focused host test, the project build, and the complete host test suite
before declaring the implementation complete.
