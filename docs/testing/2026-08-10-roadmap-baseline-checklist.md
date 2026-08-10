# Roadmap Physical Baseline Gate

This checklist is the boundary between the verified software baseline and the
physical measurements required by the advanced-software roadmap. A successful
host test or ARM build is not physical authorization. Do not begin Phase 1 or
enable any protected behavior until every required row below is completed and
the evidence is committed.

## Firmware under test

- Integration branch: `Testing`
- Fail-closed software checkpoint: `4bc0e36`
- Upload commit: Not recorded
- `USING_BIG_ROBOT` must match the robot being tested.
- Restore and commit `USING_BIG_ROBOT false` after the big-robot upload/build.

The current GUI registrations are:

| Menu | AUT1 | AUT2 | AUT3 |
|---|---|---|---|
| Red | Kevin Loader | Kevin Park | Red Six Block (locked) |
| Blue | Kevin Loader | Kevin Park | JerryIO Path (locked) |
| Skills | Skills AUT1 | TEST LemLib Forward 12in | SHADOW PLAYBACK (locked) |

## Authorization state required before ordinary uploads

Both robot configurations must keep every value below false. Record any
temporary supervised-test build separately; never treat it as the ordinary
firmware baseline.

| Gate | Big | Small | Evidence at `4bc0e36` |
|---|---:|---:|---|
| Automatic encoder fallback | false | false | Host policy test + source review |
| Forced encoder testing | false | false | Host policy test + source review |
| Shadow playback | false | false | Host policy test + source review |
| Red Six Block | false | false | Host policy test + source review |
| JerryIO Path | false | false | Host policy test + source review |

## Safety and test setup

For every moving test:

1. Use a clear field with the robot on the correct starting tile and no people
   inside its possible path.
2. Keep the controller connected and keep controller X ready as the emergency
   stop.
3. Confirm the Brain identifies the intended robot configuration and that the
   displayed autonomous name matches the selected routine.
4. Check motor temperatures, battery charge, wheel security, sensor mounting,
   pneumatic pressure, and mechanism clearance before each run.
5. Stop immediately on unexpected motion, reversed travel, violent correction,
   repeated localization jumps, mechanism collision, or nonzero outputs after
   cancellation.
6. Use a FAT32 SD card of 32 GB or less for SD logging and Shadow tests.

Record tester, date/time, robot identity, battery voltage, field surface, tire
condition, payload, upload commit, and relevant log/recording filenames with
each result. Preserve raw artifacts; do not replace them with a prose summary.

Test metadata:

| Field | Value |
|---|---|
| Tester | Not recorded |
| Date/time and timezone | Not recorded |
| Robot identity | Not recorded |
| Firmware/upload commit | Not recorded |
| SD card capacity | Not recorded |
| SD card format | Not recorded |
| Battery voltage | Not recorded |
| Field surface / tire condition / payload | Not recorded |

## A. Small-robot baseline

Upload a clean small-robot build with every authorization gate false.

| Gate | Required observation | Result | Evidence / measurements |
|---|---|---|---|
| Cold boot | Correct config; no fatal startup fault; GUI responsive | Not run | |
| Disabled safety | Drivetrain and mechanisms remain stopped | Not run | |
| Teleop controls | Expected drive direction; intake/pistons correct; X stops outputs | Not run | |
| Kevin Loader | Red or Blue AUT1 completes or fails safely; final outputs zero | Not run | |
| Kevin Park | Red or Blue AUT2 completes or fails safely; final outputs zero | Not run | |
| LemLib forward | Skills AUT2 travels forward 12 in; record final X/Y/heading and endpoint error | Not run | |
| SD logging | Log created, flushed, readable, and free of repeated write faults | Not run | |
| Reboot recovery | Clean reboot after run; no stale autonomous or actuator state | Not run | |

For the 12-inch validation, measure actual travel with an external reference.
Record commanded distance, measured distance, lateral error, final heading
error, duration, battery voltage, and whether correction oscillated.

## B. LemLib figure-eight regression

The figure-eight implementation still exists, but it is not registered in the
current GUI: Skills AUT2 is the 12-inch forward validation. Do not silently
substitute one route for the other. Record the reviewed test commit and exact
invocation used to expose `RunLemLibFigureEightValidation()` before uploading.
If no reviewed invocation is available, leave these rows incomplete and do not
advance.

| Gate | Required observation | Result | Measurements / evidence |
|---|---|---|---|
| GUI/invocation confirmation | Brain displays the exact figure-eight test name | Not run | |
| Normal completion | Route finishes without unsafe correction; final outputs zero | Not run | |
| Completion time | Record elapsed milliseconds | Not run | |
| Final pose | Record final X, Y, and heading | Not run | |
| Crossover behavior | No pose discontinuity, wrong-branch jump, or oscillatory recapture | Not run | |
| Controller-X cancellation | Motion stops immediately; drivetrain and mechanisms remain zero | Not run | |

## C. Big-robot baseline

Clean-build and upload with `USING_BIG_ROBOT true`, while keeping every
authorization gate false. Restore the repository to `USING_BIG_ROBOT false`
after the test upload.

| Gate | Required observation | Result | Evidence / measurements |
|---|---|---|---|
| Cold boot | Correct config; no fatal startup fault; GUI responsive | Not run | |
| Disabled safety | Drivetrain and mechanisms remain stopped | Not run | |
| Teleop controls | Expected drive direction; mechanisms correct; X stops outputs | Not run | |
| Kevin Loader | Red or Blue AUT1 completes or fails safely; final outputs zero | Not run | |
| Kevin Park | Red or Blue AUT2 completes or fails safely; final outputs zero | Not run | |
| LemLib forward | Skills AUT2 travels forward 12 in; record final X/Y/heading and endpoint error | Not run | |
| SD logging | Log created, flushed, readable, and free of repeated write faults | Not run | |
| Reboot recovery | Clean reboot after run; no stale autonomous or actuator state | Not run | |

## D. Shadow SD and recording gate

Perform these small-robot tests with playback authorization still false. Use
the Shadow menu for storage operations and record all displayed status/fault
text.

| Gate | Required observation | Result | Slot / file / observations |
|---|---|---|---|
| Empty formatted card | Empty slots display cleanly without a read/write fault | Not run | |
| Five-second recording save | Low-speed recording saves and appears in its selected slot | Not run | |
| Reboot and load | Saved metadata and recording remain readable after a cold reboot | Not run | |
| Confirmed overwrite | Existing slot is not replaced before confirmation; confirmed replace succeeds | Not run | |
| Confirmed delete | Existing slot is not erased before confirmation; confirmed erase succeeds | Not run | |
| Card-removal recovery | Removal reports a bounded fault; reinsertion/retry recovers without stale actuation | Not run | |

## E. Shadow playback supervised gate

Follow `docs/testing/2026-07-30-shadow-playback-checklist.md`. This requires a
dedicated supervised-test commit that changes only the small configuration's
`shadowPlaybackAuthorized` value to true. Keep all other gates false. Restore
the flag to false immediately after testing and before any ordinary upload.

| Gate | Result | Recording / log | Observations |
|---|---|---|---|
| Short drivetrain completion | Not run | | |
| Controller-X cancellation | Not run | | |
| Intake + piston event order | Not run | | |
| Authorization restored false | Not run | | |

Do not authorize Red Six Block, JerryIO Path, forced encoder testing, or
automatic encoder fallback as part of this checklist. Each requires its own
progressive physical evidence and explicit review.

## F. Timing and memory evidence

The clean linker reports showed approximately 48.53 MB BSS for big and
48.59 MB BSS for small. Before adding roadmap services, collect a runtime
baseline so later phases have a real budget rather than an assumed one.

| Measurement | Small result | Big result | Evidence |
|---|---|---|---|
| Free heap after cold boot | Not run | Not run | |
| Free heap after autonomous | Not run | Not run | |
| Main/control-loop mean period | Not run | Not run | |
| Main/control-loop maximum period | Not run | Not run | |
| Loop deadline misses | Not run | Not run | |
| SD write worst-case latency | Not run | Not run | |

If the current firmware does not expose one of these measurements reliably,
record `Blocked: instrumentation absent` rather than inventing a value. That
result becomes the input to the Phase 9 instrumentation plan.

## Gate decision

| Decision | Status |
|---|---|
| Small baseline accepted | Not run |
| Big baseline accepted | Not run |
| Shadow playback accepted | Not run |
| Runtime budget recorded | Not run |
| Approved to begin Phase 1 | **No** |

Advancement requires all applicable safety rows to pass, raw evidence to be
preserved, anomalies to be resolved or explicitly bounded, and a reviewer to
change the final decision to `Yes` in a dedicated commit. `Not run`, `Blocked`,
or an unexplained failure means stop.

Tested by:

Reviewed by:

Date/time:

Evidence commit:
