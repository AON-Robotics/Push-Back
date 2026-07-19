# Current Development Handoff

Read this file at the start of a new Codex task or after changing computers.
Use Git history as the source of truth when this file and a chat disagree.

## Repository State

- Integration branch: `Testing`
- Remote: `origin` at `AON-Robotics/Push-Back`
- Latest completed implementation checkpoint: `db084c4`
- Hardware flag committed for uploads: `USING_BIG_ROBOT false`
- Default autonomous test: red/blue AUT3, `TEST LemLib 12in`

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

## Pending Physical Gate

Do not begin the next calibration or autonomous-migration checkpoint until the
following tests are recorded:

1. Restart the brain and run AUT3 `TEST LemLib 12in`.
2. Record physical distance and final LemLib X, Y, and heading.
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

## Rules for the Next Task

- Preserve native Kevin Loader and Kevin Park until their LemLib replacements
  pass physical testing.
- Make one behavioral or calibration category change per checkpoint.
- Clean-build both robot configurations before committing robot-code changes.
- Restore `USING_BIG_ROBOT false` before committing.
- Commit and push every completed checkpoint to `origin/Testing`.
- Stop at physical gates and record measured results before continuing.
- Do not force-push, hard-reset, or delete fallbacks to resolve divergence.

Suggested skills for a new Codex task: `diagnosing-bugs` for unexpected robot
behavior, `superpowers:executing-plans` for the migration plan, and
`superpowers:verification-before-completion` before each checkpoint.
