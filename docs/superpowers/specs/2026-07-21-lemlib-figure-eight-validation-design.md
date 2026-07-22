# LemLib Figure-Eight Validation Design

## Purpose

Replace the small robot's AUT3 12-inch straight-line validation with a smooth,
continuous pure-pursuit figure-eight. The test demonstrates LemLib path
following without operating any mechanism or changing either match routine.

The separate Shadow recording failure is caused by unsupported media: the
tested card is a 128 GB SDXC card. Physical Shadow testing must use a FAT32
microSD card no larger than 32 GB. No storage-code workaround is part of this
change.

## Route

- Use one JerryIO/LemLib path asset and one `followPath` action. Do not chain
  point-to-point motions that stop at intermediate targets.
- Keep every path point within X = -22 through 22 inches and Y = 0 through 44
  inches so the robot can be tested within a two-by-two-tile clear area.
- Form a softened figure-eight with two broad lobes and one center crossover.
- Start at the crossover approach with heading zero aligned to positive Y.
- Travel forward for the entire path; do not reverse or pivot in place.
- Finish near the starting region at low speed, without requiring exact pose
  closure on the first physical test.
- Keep generated path output in `static/figure-eight.jerryio.txt`; retain the
  existing JerryIO source metadata in that asset for later tuning.

## Motion Profile

- Use an 8-inch pure-pursuit lookahead.
- Use a maximum generated path speed of 65 and a target speed of 35 at the
  crossover and tightest curvature. Finish at no more than speed 20.
- Allow a conservative 12-second timeout.
- Stop with `pros::E_MOTOR_BRAKE_BRAKE` after success or any failure.
- Preserve controller-X cancellation, motion-health monitoring, drivetrain
  ownership, and fallback locks through the existing `Actions::followPath`
  adapter.

## Integration

- Add a dedicated `RunLemLibFigureEightValidation()` routine.
- Map red and blue AUT3 to `TEST LemLib Figure 8` on the small robot.
- Keep the big robot fail-closed as unsupported.
- Leave Kevin Loader, Kevin Park, Skills routines, Shadow playback policy, and
  all mechanisms unchanged.
- Keep `RunLemLibForwardValidation()` available as a focused diagnostic even
  though AUT3 no longer selects it.

## Verification

Add a host-side asset test before creating the asset. It must verify:

- the asset exists and contains at least 40 path points;
- all coordinates fit inside the 44 by 44 inch envelope;
- neighboring points are no more than 2.5 inches apart;
- the speed column stays within 0 through 65 and slows at the finish;
- the path reaches X at or beyond -12 and 12 inches, and crosses the center
  region at least twice;
- the asset terminates with `endData`.

Then run the existing host suites and clean-build both robot configurations,
restoring `USING_BIG_ROBOT false` before committing. Physical validation starts
at reduced speed in a clear two-by-two-tile area with the robot immediately
stoppable by controller X.

## Physical Acceptance

The checkpoint passes its physical gate only when the robot completes the
entire path smoothly, remains inside the marked area, crosses the center
without oscillation, and stops safely. Record final X, Y, heading, completion
time, and any visible corner-cutting before increasing speed or changing
lookahead.
