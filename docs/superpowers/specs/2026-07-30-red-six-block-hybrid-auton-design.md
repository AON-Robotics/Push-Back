# Red Six-Block Hybrid Autonomous Design

## Objective

Build a reliable red-side match autonomous for the small differential robot
that collects six blocks from the match loader and scores them in the long
goal. The route should showcase smooth LemLib movement in open space while
using precise closed-loop positioning at the loader and goal.

The first implementation targets only the red side. Mirroring to blue occurs
after the red route passes its physical gates.

## Design Choice

Use a hybrid of long pure-pursuit paths and short LemLib precision actions.

- Pure pursuit handles broad travel where continuous curvature and visible
  smoothness matter.
- `moveToPose` handles loader and goal contact where endpoint position and
  heading matter more than uninterrupted motion.
- Chained reverse point/pose movements leave the loader without unnecessary
  stop-and-turn behavior.

A single path asset is not appropriate for the complete routine because
loader collection requires a dwell and the route changes between forward and
reverse travel. Using only point/pose actions would be easier to tune but would
produce visibly segmented movement.

## Route

### Phase 1: Smooth Loader Approach

Set the known red starting pose, prepare the loader cart, and begin a broad
forward pursuit path from the starting tile to a loader staging pose.

The path:

- is open and non-self-intersecting;
- has a unique zero-speed endpoint;
- stays clear of field obstacles using the complete robot footprint;
- uses curvature-aware speeds;
- ends far enough from the loader for odometry correction and a controlled
  final approach;
- contains no mechanism timing that depends on reaching an ambiguous path
  index.

Immediately after the path succeeds, use a low-speed `moveToPose` to establish
repeatable loader contact. A short bounded forward hold may maintain contact.

### Phase 2: Six-Block Collection

Start the existing loader collection mechanism sequence and dwell for a
measured, bounded duration sufficient for six blocks. The drivetrain holds the
loader contact pose during collection.

Finish loader collection explicitly before retreating. A failed mechanism
transition ends the routine and stops all outputs.

### Phase 3: Chained Reverse Exit

Reset the loader cart and reverse away through two chained LemLib actions:

1. a faster early-exit movement that creates clearance from the loader;
2. a lower-speed pose action that finishes the retreat at the heading required
   by the scoring path.

The first movement uses a nonzero minimum speed and early-exit range so the
second action inherits motion instead of beginning from a full stop. No
turn-in-place action is used unless physical testing proves the chained reverse
geometry cannot meet the scoring-path entry tolerance.

### Phase 4: Smooth Goal Transfer

Follow a second broad pursuit path from the retreat pose toward a long-goal
staging pose. This asset follows the same open-path, unique-endpoint,
curvature, clearance, and footprint rules as the loader path.

Prepare the top scorer during safe open travel, before final goal contact.

### Phase 5: Precision Score

Use a low-speed `moveToPose` for the final long-goal alignment and contact.
Score six blocks using the existing explicit mechanism action, then stop the
drivetrain and every mechanism.

No park or second scoring cycle is included in the first red-side milestone.

## Motion Parameters

Path geometry and speed profiles are generated deterministically by repository
scripts. Initial parameters are conservative starting points, not claims of
physical tuning:

- pursuit lookahead: 7 inches in open travel;
- open-path maximum speed: 90;
- loader staging tail maximum speed: 45;
- loader contact `moveToPose` maximum speed: 30;
- reverse-clearance maximum speed: 60;
- reverse-alignment maximum speed: 40;
- goal staging tail maximum speed: 45;
- goal contact `moveToPose` maximum speed: 30.

The first generated geometry preserves the existing staged experiment's local
coordinate contract:

- start pose: `(0, 0, 0 deg)`;
- loader pursuit staging point: `(0, 24)`;
- loader contact pose: `(4, 31, 86 deg)`;
- reverse clearance point: `(-5, 31)`;
- reverse alignment pose: `(-9, 31, 171 deg)`;
- long-goal contact pose: `(-8, 25, 171 deg)`.

The goal-transfer asset begins at the reverse alignment pose and ends at a
generated staging pose on the final `moveToPose` approach line. The generator
must emit that staging pose into a shared validation header so the asset,
routine, and host test cannot silently disagree.

Every action has an explicit timeout. Total nominal routine duration must
remain within the competition autonomous period with at least a 1-second
software margin. If measured six-block collection and scoring cannot satisfy
that margin, the route remains a test routine and is not promoted to a match
slot.

## Failure Behavior

Every required action returns a structured result. On any failure:

- cancel current LemLib motion;
- stop drivetrain output;
- stop loader, intake, and scoring mechanisms;
- suppress all remaining actions;
- log the phase and action that failed;
- report the autonomous routine as failed.

Controller X remains a latched emergency cancellation. Invalid odometry,
device feedback failure, timeout, or loss of drivetrain ownership follows the
same fail-closed path. The routine does not authorize automatic encoder
fallback.

## Code Boundaries

- Path generator scripts own deterministic geometry and speed generation.
- Generated JerryIO files are embedded assets, not edited by hand.
- `RunRedSixBlockHybridAuton()` owns phase sequencing and failure propagation.
- Existing `Actions` methods own LemLib motion monitoring and cancellation.
- Existing mechanism actions own loader and scoring hardware details.
- Routine selection initially exposes the route only in a testing slot.

The routine moves to Red AUT1 only after every physical gate passes. Existing
Kevin routines remain available until that promotion.

## Automated Verification

Host tests validate:

- both generated assets parse and terminate correctly;
- only each asset's final point has speed zero;
- point spacing and speed changes are bounded;
- paths are open and non-self-intersecting;
- curvature never requires an unintended inner-wheel reversal;
- the modeled complete robot footprint remains in the allowed field region;
- loader and goal staging poses match their precision-action entry poses;
- expected start and endpoint tangents/headings;
- deterministic generator output;
- routine declaration and selector registration;
- failure of any required action suppresses later phases where testable
  through platform-independent sequencing helpers;
- total configured timeout and dwell budget fits the autonomous period plus
  the required 1-second margin.

All existing motion, Shadow, SD, and figure-eight host suites must remain
green. Clean small- and big-robot builds are required, followed by restoration
and rebuild of `USING_BIG_ROBOT false`.

## Physical Gates

Run each gate in a clear field with controller X ready. Record elapsed time,
final pose, heading error, and observed mechanism result.

1. Loader approach pursuit only.
2. Loader precision contact only from the staging pose.
3. Contact plus six-block collection.
4. Reverse-clearance chain only.
5. Goal-transfer pursuit only from the retreat pose.
6. Goal precision contact and six-block scoring.
7. Full routine at reduced path speed.
8. Full routine at the approved competition speed.
9. Three consecutive successful cold-boot runs from measured starting
   placement.

Any collision, missed loader contact, route oscillation, unexpected full stop,
odometry fault, missed score, timeout, or cancellation failure blocks
promotion and returns testing to the failing phase.

## Out of Scope

- blue-side mirroring;
- parking;
- a second loader or scoring cycle;
- Shadow-generated competition paths;
- automatic encoder fallback;
- autonomous tuning without measured physical results.
