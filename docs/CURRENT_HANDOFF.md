# Current Development Handoff

## 2026-08-10 C++ Safety and Resource Hardening Checkpoint

The hardening plan is implemented through bounded runtime ownership:

- GUI and drivetrain polymorphic bases have virtual nonthrowing destruction;
- millisecond comparisons remain correct across `uint32_t` rollover;
- final-heading alignment stops after stalled progress;
- GPS jump history advances only after filter acceptance;
- every pose component is checked for finiteness;
- all nine physical authorization gates fail closed;
- planner scratch data is fixed-capacity, planner-owned storage;
- odometry mutex waits are bounded at two milliseconds and counted;
- RTOS task creation returns a typed result and autonomous callers stop on
  failure.

Fresh dependency-free host verification passed under C++17 with warnings as
errors. The separate PROS ownership probe uses GNU C++20 because the bundled
PROS headers require C++20; its suppressions cover vendor/header warnings only.
Clean ARM builds linked for both robot configurations; the committed selector
is `USING_BIG_ROBOT false`. Linker-reported BSS remains approximately 48.53 MB
for big and 48.59 MB for small. The only ARM compiler warning is the existing
vendored `json.hpp` `std::is_pod` deprecation.

Static inspection found no first-party manual `new`/`delete` or
`malloc`/`free` ownership and no active object slicing. This does not prove
physical timing, localization accuracy, LiDAR mounting, or memory headroom on
the V5 Brain. Every existing physical checklist result remains `Not run`.

## 2026-08-10 Localization/EKF Software Checkpoint

The corrected three-wheel motion model, fixed three-state EKF, wrapped IMU
update, gated GPS math, coherent `Odometry` snapshot, shared tracking-sensor
ownership, deterministic 10 ms loop, opt-in CSV diagnostics, and inactive
LemLib fused-pose adapter are implemented.

An independent final review found and closed reset/publication races,
wheel-sensor recovery skew, repeated identical GPS fusion, GPS gate poisoning,
non-finite reset input, covariance asymmetry, and IMU-calibration fail-open
behavior. Autonomous routines now refuse to start when the selected
localization path did not complete its boot checks.

Host localization suites pass with warnings as errors. Both small- and
big-robot configurations compiled and linked during implementation, with
`USING_BIG_ROBOT false` restored. The only accepted warning remains the
vendored `json.hpp` `std::is_pod` deprecation.

This is not physical authorization:

- `fusedLemLibAuthorized` is false for both robots;
- GPS is disabled with port zero for both robots;
- existing LemLib wheel/IMU odometry remains the competition default;
- nominal wheel diameter/offsets and all filter noise values still require
  measurement or tuning;
- no EKF accuracy improvement is claimed.

Run and record the existing baseline gate first, then follow
`docs/testing/localization-benchmark.md`. Do not enable fusion unless the raw
geometry/sign tests pass and pausing AON publication proves LemLib has no
independent pose writer. Configure GPS only from a measured installation.

## 2026-08-10 Fail-Closed Authorization Checkpoint

Phase 0 Tasks 1 and 2 are implemented. Both robot configurations now fail the
unvalidated-baseline policy if any of these gates is enabled:

- automatic encoder fallback;
- forced encoder testing;
- Shadow playback;
- Red Six Block;
- JerryIO path autonomous.

The small-robot Shadow playback flag was restored from the supervised-test
value `true` to `false` because the committed playback checklist still records
all physical results as `Not run`. The big configuration and every other gate
remain false.

Verification for this checkpoint:

- authorization policy test passed for the active small configuration;
- the same linked policy test passed with `USING_BIG_ROBOT true`;
- all ten dependency-free host executables passed with warnings as errors;
- clean big-robot build linked successfully;
- the expected small configuration was restored and clean-built successfully;
- the only build warning was the existing vendored `json.hpp` `std::is_pod`
  deprecation;
- linker-reported BSS was approximately 48.53 MB for big and 48.59 MB for
  small, so the roadmap memory-budget gate remains mandatory.

The physical gate is published at
`docs/testing/2026-08-10-roadmap-baseline-checklist.md`. Every physical result
is intentionally `Not run`, and the advancement decision is `No`. Stop here
for measurements. Do not enable a gate or start later roadmap implementation
before that checklist is completed, reviewed, and committed with raw evidence.

## 2026-08-10 Roadmap Planning Note

The approved architecture for the twelve-system reliability roadmap is
recorded in
`docs/superpowers/specs/2026-08-10-advanced-robot-software-roadmap-design.md`.
The complete plan suite is under `docs/superpowers/plans/` with 2026-08-10
filenames. Phase 2 uses the deeper `2026-08-10-localization-ekf.md` plan.

The first executable plan is
`2026-08-10-safety-state-reconciliation-and-baseline.md`. It restores the
unsupported small-robot Shadow playback flag to false, verifies every other
gate remains false, publishes one physical baseline checklist, and then stops
for measured robot results.

Historical reconciliation note (resolved by `4bc0e36`): the small-robot Shadow
flag had temporarily been true for a supervised test even though the physical
results remained `Not run`. The active source now keeps it false. Do not infer
future authorization from builds or host tests.

Read this file at the start of a new development session or after changing computers.
Use Git history as the source of truth when this file and a chat disagree.

## Repository State

- Integration branch: `Testing`
- Remote: `origin` at `AON-Robotics/Push-Back`
- Latest completed Shadow mechanism checkpoint: `d531935`
- Hardware flag committed for uploads: `USING_BIG_ROBOT false`
- Default autonomous test: red/blue AUT3, `TEST LemLib Figure 8`

Confirm these claims before editing:

```powershell
git switch Testing
git fetch origin
git pull --ff-only origin Testing
git status --short --branch
git log -5 --oneline
```

The expected status is a clean `Testing` branch synchronized with
`origin/Testing`. Do not discard, reset, or overwrite unexpected local changes.

## Active Work

The approved architecture and implementation sequence are recorded in:

- `docs/superpowers/specs/2026-07-16-lemlib-single-chassis-migration-design.md`
- `docs/superpowers/plans/2026-07-16-lemlib-single-chassis-migration.md`
- `docs/superpowers/specs/2026-07-17-motor-encoder-odometry-fallback-design.md`
- `docs/superpowers/plans/2026-07-17-motor-encoder-odometry-fallback.md`
- `docs/superpowers/specs/2026-07-21-shadow-auton-design.md`
- `docs/superpowers/plans/2026-07-21-shadow-auton.md`
- `docs/superpowers/specs/2026-07-21-lemlib-figure-eight-validation-design.md`
- `docs/superpowers/plans/2026-07-21-lemlib-figure-eight-validation.md`

Task 1 is implemented. On a fresh boot, LemLib validation no longer starts the
legacy odometry task. Native Kevin and Skills routines start legacy odometry
lazily through `aon::legacy_motion::prepare()`.

Motor-encoder fallback Tasks 1 through 6 are implemented. The code now has a
host-tested health monitor, relative encoder controller, single-owner drive
sampling, structured motion results, failure propagation, and a brain mode
control. Automatic switching is deliberately gated off in both robot
configurations with `automaticFallbackAuthorized = false` until physical
validation reaches that step. Forced mode now has a separate
`forcedEncoderTestingAuthorized = false` gate, so the brain's `AUTO LOCKED`
control cannot start unvalidated encoder motion. Both flags remain false until
the baseline gate below is recorded.

Checkpoint `db084c4` hardens the fallback layer before physical testing:

- controller-X cancellation reaches LemLib and encoder motion;
- cancellation remains latched until a later autonomous dispatch safely rearms;
- concurrent autonomous actions cannot share drivetrain ownership;
- timed arcade motion stops on invalid drive feedback;
- pose-jump fault samples must be consecutive.

Shadow Auton Tasks 1 through 5 are implemented through checkpoint `d531935`:

- fixed-capacity, 20 ms driver sample capture;
- route processing with pose-quality checks and stopped-segment detection;
- versioned, checksummed recording files;
- resilient dual-generation SD storage and recording-session safety;
- normalized, atomic effective-drive capture and semantic mechanism events;
- explicit small- and big-robot mechanism playback adapters;
- bounded, acknowledged big-robot sorter transitions with fail-safe motor ownership;
- host coverage for recorder, processor, codec, storage, and service state;
- clean small- and big-robot builds, with `USING_BIG_ROBOT false` restored.

Shadow Auton Task 6 now adds the normal-GUI `SHADOW` screen, three recording
slots, overwrite/delete confirmation, save status, slot metadata, and an
inert `PLAY LOCKED` control. Playback remains intentionally locked and no
Shadow autonomous routine is registered or armed.

Shadow recording implementation is complete with playback authorization false.
Do not implement or expose playback until the existing LemLib/native baseline
gate passes and all three SD slots survive a full-reboot save/load test.

The photographed Shadow `OPEN FAILED` result on empty cards was traced to a
PROS 4.2.2 errno mismatch: a missing file open is reported as `ENFILE`, while
the adapter previously required `ENOENT` before treating a slot as empty. The
adapter now uses a bounded, fail-closed root-directory check and has host
regression coverage. This explains the same result on the 32 GB card. The
original 128 GB exFAT card remains unsupported; use FAT32 media no larger than
32 GB and cold-boot the Brain after inserting it.

The first physical `STOP SAVE` attempt on the verified 32 GB FAT32 card created
`aon-shadow-slot-1-a.bin` with a length of zero bytes. Windows CHKDSK reported
the FAT32 volume healthy, and deleting only that zero-byte generation restored
Brain startup. The SD adapter now writes encoded recordings in bounded 4 KiB
chunks and removes the target generation after any write, flush, or close
failure. This cleans up reported save failures when deletion succeeds; it
cannot make FAT writes power-loss atomic. Never power off until processing
finishes and the slot reports valid.

A later physical recording remained on `PROCESSING` for more than 10 minutes.
The Shadow simplifier had approximately 35 KB of fixed-capacity scratch arrays
as automatic locals inside the GUI-triggered save call, enough to overflow the
V5 task stack and prevent `finishSave()` from running. `ProcessorWorkspace`
now owns those arrays in persistent service storage. Host compiler stack-usage
output reports 352 bytes for the main processor function and 240 bytes for its
motion-point helper. The full-duration algorithm and physical save latency
still require a new recording test; do not treat a long `PROCESSING` state as
normal V5 performance.

The first physical figure-eight asset failed: it moved clumsily and completed
only an S-shaped half-route. Inspection against LemLib 0.5.6's pursuit
implementation found that the original 174-inch parametric path combined
roughly 3-inch-radius bends with an exactly duplicated start/end point.
LemLib searches globally for the closest point and treats a zero-speed closest
point as completion, so a closed, self-crossing asset is ambiguous.

The replacement is one continuous separated-crossover figure eight. Its lower
and upper center passes are at Y=18 and Y=30, respectively, so pose noise
cannot make LemLib's global nearest-point search jump between them. The two
outside turns are 6.5-inch-radius semicircles, just above the small robot's
6.25-inch half-track; smooth S transitions join them. Curvature-aware speeds
use the fastest centerline command that keeps the ideal outside wheel at or
below 100, and only the unique endpoint has speed zero. This avoids closed-path
ambiguity and all intermediate stop/restart transitions. The path uses a
7-inch lookahead and a 12-second timeout. Host checks cover nonlocal clearance,
non-intersection, the two separated center passes, unique endpoint, turn
radius, path length, and an 18-by-16-inch exterior envelope inside the
requested 48-by-48-inch area. Physical smoothness still requires the physical
gate.

## Pending Physical Gate

Do not begin the next calibration or autonomous-migration checkpoint until the
following tests are recorded:

1. Clear a 48-by-48-inch (two-tile-by-two-tile) area, place the robot at the
   figure-eight start, restart the Brain, and run AUT3
   `TEST LemLib Figure 8`. Keep controller X ready to stop.
2. Record completion time, final LemLib X, Y, and heading, crossover
   oscillation, and any corner-cutting.
3. Restart the brain again and cautiously run one native Kevin fallback.
4. Record whether GUI selection, drivetrain motion, and mechanisms behave as
   they did before Task 1.

Once a native route starts, its legacy odometry task remains active until the
program restarts. Always restart the brain before returning to LemLib testing.

After this existing gate passes, resume Task 7 of the motor-encoder fallback
plan. Add and expose only the 6-inch forced-encoder forward validation first;
do not authorize automatic fallback or expose later primitives yet.

The current encoder controller is suitable only for the isolated one-action
validation sequence. It does not yet maintain a dead-reckoned field pose across
multiple encoder actions. Implement and host-test that pose continuity before
enabling automatic fallback or any route with dependent encoder motions.

## Verified State

At checkpoint `db084c4`:

- All host-side motion health, geometry, output-cap, timeout-budget, travel
  direction, and authorization-policy tests passed with warnings treated as
  errors.
- Clean small-robot build passed.
- Clean big-robot build passed.
- The small-robot configuration was restored and rebuilt.
- Independent safety review found no remaining Critical or Important issues.
- The only observed compiler warning was the existing vendored `json.hpp`
  `std::is_pod` deprecation.

Compilation does not satisfy the pending physical gate.

### Shadow Auton recording gate

Tasks 5 and 6 are implemented and compile-verified. Test recording only:

1. Insert a FAT32 SD card no larger than 32 GB while the Brain is off, then
   cold-boot it. A fresh card should show all three slots as `EMPTY`.
2. Record a five-second drivetrain-only route into slot 1. Press `STOP SAVE`
   and wait until processing finishes and the slot reports valid before
   powering off.
3. Restart the Brain and confirm slot 1 metadata loads. Only then repeat with
   mechanism actions and the remaining slots.
4. Overwrite one slot and delete another; restart and confirm both operations.
5. Remove the SD card during a disposable recording and confirm the prior saved
   generation remains readable after reinserting it.

Do not unlock or run Shadow Auton playback before this recording gate passes.

## Rules for the Next Task

- Preserve native Kevin Loader and Kevin Park until their LemLib replacements
  pass physical testing.
- Make one behavioral or calibration category change per checkpoint.
- Clean-build both robot configurations before committing robot-code changes.
- Restore `USING_BIG_ROBOT false` before committing.
- Commit and push every completed checkpoint to `origin/Testing`.
- Stop at physical gates and record measured results before continuing.
- Do not force-push, hard-reset, or delete fallbacks to resolve divergence.

For the next session, stop at Shadow Physical Gate A. Upload the small-robot
configuration, perform the LemLib/native baseline checks and all recording-only
SD tests above, and record measured results here. Do not begin Task 7 playback
policy until every Gate A item passes.
