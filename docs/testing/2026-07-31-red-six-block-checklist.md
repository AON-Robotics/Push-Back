# Red Six-Block Physical Test Checklist

## Starting placement

All coordinates use the autonomous routine's local field frame:

- Place the robot tracking center at the repeatable red-side starting mark.
- Align the robot front along local `+Y`; this is heading `0 deg`.
- Local `+X` points to the robot's right and positive heading is clockwise.
- Mark the tracking-center location and front alignment on the field before the
  first run. Record those measurements below before accepting Gate 1.

Measured field landmark for tracking center: **Not measured**

Measured robot edge/fixture used for alignment: **Not measured**

## Safety and advancement rules

- Use the small-robot build and confirm the Brain shows
  `RED 6-BLOCK HYBRID` in Red AUT3.
- Keep controller X ready during every motion test and verify it stops all
  drivetrain and mechanism output before advancing past Gate 1.
- Run one gate at a time. Temporarily change `RunRedSixBlockHybridFull()` to
  the listed inclusive `RedSixPhase`, upload, test, then restore `ScoreSix`.
  Never commit a temporary gate limit.
- Restart the Brain before each cold-boot run and re-establish the exact start
  placement.
- Stop advancement after a collision, missed contact, pursuit oscillation,
  unexpected full stop in the reverse chain, odometry fault, missed score,
  timeout, unsafe mechanism action, or failed controller-X cancellation.
- Tune geometry only through `tools/generate-red-six-block-paths.ps1`; never
  hand-edit a generated JerryIO asset.

## Results

| Gate | Build commit | Start placement | Path max speed | Elapsed ms | Final X | Final Y | Final heading | Mechanism result | X-cancel | Pass/fail | Adjustment |
|---|---|---|---:|---:|---:|---:|---:|---|---|---|---|
| 1. `LoaderPursuit` | Not run | Not measured | 90 | Not run | Not run | Not run | Not run | Cart deployed; no intake motion expected | Not run | Not run | None recorded |
| 2. `LoaderContact` from measured staging pose | Not run | Loader staging pose | 30 | Not run | Not run | Not run | Not run | Repeatable loader contact without collision | Not run | Not run | None recorded |
| 3. `CollectSix` after contact | Not run | Loader contact pose | 0 | Not run | Not run | Not run | Not run | Six blocks collected in bounded 4000 ms dwell | Not run | Not run | None recorded |
| 4. Chained `ReverseClearance` + `ReverseAlignment` | Not run | Loader contact pose | 60 / 40 | Not run | Not run | Not run | Not run | Cart resets before retreat | Not run | Not run | None recorded |
| 5. `GoalPursuit` from measured reverse pose | Not run | Reverse alignment pose | 45 | Not run | Not run | Not run | Not run | Top scorer prepares during open travel | Not run | Not run | None recorded |
| 6. `GoalContact` + `ScoreSix` | Not run | Goal staging pose | 30 | Not run | Not run | Not run | Not run | Six blocks scored; outputs stop afterward | Not run | Not run | None recorded |
| 7. Full route, generated paths capped to 60 | Not run | Measured red start | 60 | Not run | Not run | Not run | Not run | Complete collect-and-score cycle | Not run | Not run | None recorded |
| 8. Full route at approved speed up to 90 | Not run | Measured red start | 90 | Not run | Not run | Not run | Not run | Complete collect-and-score cycle | Not run | Not run | None recorded |
| 9a. Cold-boot full run 1 | Not run | Measured red start | Approved value | Not run | Not run | Not run | Not run | Complete collect-and-score cycle | Not run | Not run | None recorded |
| 9b. Cold-boot full run 2 | Not run | Measured red start | Approved value | Not run | Not run | Not run | Not run | Complete collect-and-score cycle | Not run | Not run | None recorded |
| 9c. Cold-boot full run 3 | Not run | Measured red start | Approved value | Not run | Not run | Not run | Not run | Complete collect-and-score cycle | Not run | Not run | None recorded |

## Promotion decision

Red AUT1 remains `Kevin Loader`. Promotion is **not authorized** until all
gates pass, three consecutive cold-boot runs succeed, and the measured total
duration leaves at least 1000 ms inside the 15-second autonomous period.
