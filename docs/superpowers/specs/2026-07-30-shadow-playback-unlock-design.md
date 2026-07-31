# Shadow Playback Unlock Design

## Objective

Implement and unlock one-shot Shadow Auton playback for the small differential
robot. Playback reconstructs recorded motion as closed-loop LemLib paths and
replays every validated semantic mechanism event. It must fail closed on any
motion, odometry, storage, cancellation, timing, or mechanism error.

The first physical target is the valid recording already produced by the
small robot. Big/H-drive playback remains unsupported.

## Existing Foundation

The repository already provides:

- fixed-capacity 20 ms pose, drive-command, and controller capture;
- semantic intake and piston event capture;
- route simplification into forward, reverse, and dwell segments;
- versioned, checksummed, dual-generation SD storage;
- matching-robot and corrupt-file rejection;
- persistent processor scratch memory safe for the V5 task stack;
- LemLib motion monitoring, exclusive drivetrain ownership, bounded timeouts,
  odometry health checks, and latched controller-X cancellation;
- semantic mechanism playback adapters;
- a one-shot playback-arm state machine.

The actual player, runtime path conversion, GUI arming flow, and autonomous
registration do not yet exist. `PLAY LOCKED` is therefore not a single policy
flag waiting to be changed.

## Architecture

### Pure Playback Scheduler

A platform-independent scheduler validates playback policy and walks the
decoded route in segment order. It delegates motion, dwell, progress polling,
mechanism execution, cancellation checks, and final stopping through injected
callbacks.

The scheduler:

1. rejects unauthorized, unarmed, corrupt, wrong-robot, or unsupported-robot
   input before commanding hardware;
2. executes motion and dwell segments sequentially;
3. dispatches each event once, in recorded order;
4. dispatches motion events when monotonic segment progress reaches the
   recorded progress;
5. dispatches dwell events when elapsed dwell time reaches the recorded
   offset;
6. stops immediately after any callback failure or cancellation;
7. suppresses every remaining segment and event after failure.

### V5 LemLib Adapter

The small-robot adapter loads the armed slot into process-lifetime storage and
serializes one motion segment at a time into a bounded in-memory LemLib asset.
Each asset contains the processed X, Y, and inferred speed values plus the
required JerryIO terminator and metadata.

The adapter calls the existing monitored `Actions::followPath` boundary:

- recorded forward/reverse direction is preserved;
- lookahead is 6 inches;
- timeout is `max(recordedDuration * 2 + 1000 ms, 2000 ms)`;
- the temporary asset buffer remains alive until the synchronous follow call
  returns;
- a poll callback projects the current pose onto the active polyline and
  reports monotonic progress to the scheduler.

Dwell segments use condition-based polling in 20 ms increments so controller
X, competition disable, and cancellation remain responsive.

The adapter never authorizes motor-encoder fallback. A LemLib tracking failure
ends playback.

### Mechanism Playback

Recorded semantic events are replayed through the existing small-robot
mechanism adapter. Intake modes and piston states are explicit target states,
not button toggles.

Every kind/value pair is validated before hardware is touched. A wrong-robot
or unsupported event stops playback, stops all mechanisms, and suppresses all
later events.

## GUI and Autonomous Flow

The Shadow screen changes from an inert `PLAY LOCKED` button to a one-shot
arming control only when all of these conditions hold:

- small-robot playback authorization is enabled;
- the selected slot is valid and matches the active robot;
- the robot is disabled;
- recording and saving are idle.

The workflow is:

1. select a valid slot;
2. press `PLAY`;
3. inspect the displayed duration and starting X, Y, and heading;
4. press `PLAY` again within five seconds to confirm;
5. place the robot at the displayed starting pose;
6. select/run the registered Shadow autonomous routine.

Confirmation arms only the selected slot. Autonomous dispatch consumes the arm
before loading or commanding hardware, so the same confirmation cannot start
twice. Recording, deletion, cancellation, robot disable, a different slot
selection, or confirmation expiry clears the arm.

The GUI reports `ARMED`, `PLAYING`, `FINISHED`, `CANCELLED`, or the exact
failure result. It never represents a merely valid file as armed.

## Safety Rules

- Playback authorization defaults to false and is enabled only for the small
  robot after automated verification.
- Big/H-drive playback always returns `UnsupportedRobot`.
- Controller X, competition disable, invalid odometry, path timeout, motion
  failure, mechanism failure, corrupt storage, or wrong robot stops the
  drivetrain and every mechanism.
- Playback owns the drivetrain through the existing motion lease.
- No later event may fire after cancellation or failure.
- A playback arm is one-shot and slot-specific.
- The route start pose is displayed before arming.
- Physical testing occurs only in a clear area with controller X ready.

## Testing

Host tests cover:

- authorization and one-shot slot arming;
- unauthorized, unarmed, wrong-robot, big-robot, and corrupt input making zero
  motion or mechanism calls;
- forward/reverse segment ordering;
- monotonic motion-event progress;
- dwell-event offset timing;
- exactly-once event dispatch;
- failed motion and failed mechanisms suppressing all later work;
- cancellation during motion and dwell;
- timeout calculation and bounded asset serialization;
- polyline projection that never moves progress backward;
- invalid points, counts, directions, event values, and buffer capacity;
- final drivetrain and mechanism stopping on success and failure.

Verification requires:

1. all host suites with warnings treated as errors;
2. clean small-robot build;
3. clean big-robot build with playback still rejected;
4. restoration and clean rebuild of `USING_BIG_ROBOT false`;
5. `git diff --check` and a clean intended worktree.

## Physical Rollout

1. Upload the verified small-robot build.
2. Record and save a short, low-speed route in a clear area.
3. Arm playback and replay drivetrain movement with controller X ready.
4. Repeat with a short recording containing one intake state and one piston
   transition.
5. Verify endpoint accuracy, direction, event order, cancellation, and final
   motor state.
6. Only after those checks, replay longer recordings.

Any unexpected movement, event order, odometry behavior, endpoint error, or
cancellation failure immediately relocks playback until diagnosed.

## Out of Scope

- Big/H-drive or holonomic playback;
- automatic motor-encoder fallback;
- editing recordings on the Brain;
- dynamically retiming mechanism events;
- competition use before the progressive physical rollout passes.
