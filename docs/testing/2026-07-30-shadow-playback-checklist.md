# Shadow Playback Physical Gate

Use this checklist only with the small differential-robot configuration and a
clear field. Keep controller X ready as the emergency stop throughout every
run. The automated host and cross-configuration build gates must pass before
uploading.

## Progressive test

1. Clear the field, confirm the robot is using the small configuration, and
   keep controller X ready.
2. Record and save a short, low-speed route without mechanism events.
3. Verify the displayed start X, Y, and heading; press `PLAY`, then
   `CONFIRM PLAY` within five seconds; verify Skills AUT3 selects
   `SHADOW PLAYBACK`.
4. Place the robot at the displayed start pose and run autonomous.
5. Verify travel direction, endpoint, controller-X cancellation, and stopped
   drivetrain/intake outputs.
6. Repeat with a short recording containing one intake state and one piston
   transition.
7. Verify mechanism-event order and the final explicit mechanism states.
8. Record pass/fail, slot, route duration, endpoint error, and observed fault
   text below.

Do not replay a long recording until both the drivetrain-only run and the
one-intake/one-piston run pass.

If movement, event ordering, odometry, endpoint accuracy, or cancellation is
unexpected, stop immediately and set the small configuration's
`shadowPlaybackAuthorized` value back to `false` before any longer test.

## Results

| Gate | Pass/fail | Slot | Duration (ms) | Endpoint error (in) | Fault text / observations |
|---|---|---:|---:|---:|---|
| Drivetrain only | Not run |  |  |  |  |
| Intake + piston | Not run |  |  |  |  |

Tested by:  
Date/time:  
Small-build commit:  
