# Shadow Playback Physical Gate

Use this checklist only with the small differential-robot configuration and a
clear field. Keep controller X ready as the emergency stop throughout every
run. The automated host and cross-configuration build gates must pass before
uploading.

## Current gate status

- Automated host suites: passed on 2026-07-31.
- Clean ARM builds: small, big, then restored small all passed on 2026-07-31.
- Small playback authorization: `true` for supervised physical testing.
- Physical tests: not run.

The small robot may arm playback whether competition control reports enabled or
disabled. Keep the big configuration locked. If any physical gate fails,
restore the small robot's `shadowPlaybackAuthorized` value to `false` before
any further run.

## Progressive test

1. Clear the field, confirm the robot is using the small configuration, and
   keep controller X ready.
2. Record and save a short, low-speed route without mechanism events.
3. Verify the displayed start X, Y, and heading, then place the robot at that
   pose before arming.
4. Press `PLAY`, then `CONFIRM PLAY` within five seconds in any competition
   state. Verify Skills AUT3 selects `SHADOW PLAYBACK`, then invoke autonomous
   playback within five seconds.
5. Let the drivetrain-only route finish normally. Verify travel direction,
   endpoint error, and stopped drivetrain/intake outputs.
6. Re-arm the same short route, start autonomous, and press controller X
   mid-route. Verify immediate cancellation and stopped outputs separately.
7. Repeat with a short recording containing one intake state and one piston
   transition. Verify event order and final explicit mechanism states.
8. Record each independent result and any displayed fault text below.

Do not replay a long recording until normal completion, emergency cancellation,
and the one-intake/one-piston run all pass.

If movement, event ordering, odometry, endpoint accuracy, or cancellation is
unexpected, stop immediately and set the small configuration's
`shadowPlaybackAuthorized` value back to `false` before any longer test.

## Results

| Gate | Pass/fail | Slot | Duration (ms) | Endpoint error (in) | Fault text / observations |
|---|---|---:|---:|---:|---|
| Drivetrain completion | Not run |  |  |  |  |
| Controller-X cancellation | Not run |  |  | N/A |  |
| Intake + piston | Not run |  |  |  |  |

Tested by:

Date/time:

Supervised-test build commit:
