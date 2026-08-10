# C++ Safety and Resource Hardening Design

## Purpose

Harden the first-party C++ introduced after `9eb4d34` without enabling or
claiming physical validation for localization, LiDAR, fused LemLib, or fused
navigation. Fix demonstrated lifetime and control-flow defects, make resource
failure explicit, reduce avoidable compile coupling, and standardize concise
public-interface comments.

## Scope

This work covers first-party files under `include/aon`, `src/aon`, `tests`,
`tools`, and directly related documentation. Vendored PROS, LemLib, LVGL,
fmt, and JSON sources are excluded.

The implementation must preserve:

- `USING_BIG_ROBOT false` in committed firmware;
- every physical authorization gate as false;
- fixed-capacity storage in periodic and protocol paths;
- the existing GUI, autonomous, Shadow, localization, and navigation behavior
  except where a regression test demonstrates a safety defect;
- the physical baseline decision `Approved to begin Phase 1: No`.

## Findings to Correct

### Lifetime and ownership

`aon::gui` owns `GuiDebug` through `std::unique_ptr<Gui>`, but `Gui` lacks a
virtual destructor. Destruction through the base pointer is undefined behavior.
Polymorphic owning seams must have a non-throwing virtual destructor. Classes
that own hardware or unique resources must explicitly declare their copy and
move policy rather than relying on accidental compiler behavior.

No first-party mismatched `new`/`delete`, `malloc`/`free`, throwing destructor,
or object-slicing defect was found. The design therefore does not introduce
more smart pointers or inheritance without a concrete ownership need.

### Periodic synchronization

Localization snapshot access currently calls `pros::Mutex::take()` with an
unbounded wait and manually pairs it with `give()`. Periodic tasks require a
small bounded timeout, an acquisition-failure counter, and an RAII guard whose
destructor cannot throw. A failed lock must return the most recent coherent
snapshot or skip publication; it must never publish a partially updated value.

### RTOS task creation

Localization task starters treat allocation of a `pros::Task` wrapper as proof
that the underlying task started. Each starter must return an explicit result,
validate the native task handle/status available from PROS, avoid latching a
failed wrapper as success, and allow a later retry. Callers must log or expose
the failure without enabling fallback behavior automatically.

### Navigation and estimator logic

The following behavior must be regression-tested and corrected:

- final-heading alignment applies the same progress watchdog as path travel;
- a GPS sample mutates jump-gate history only after the EKF accepts it;
- timestamp ordering and expiry use unsigned-wrap-safe elapsed-time logic;
- raw odometry publishes only when X, Y, and heading are all finite;
- expired dynamic obstacles are removed correctly across `uint32_t` rollover.

### Authorization coverage

The fail-closed snapshot must include all physical-risk gates:

- automatic encoder fallback;
- forced encoder testing;
- Shadow playback;
- Red Six Block;
- JerryIO path;
- GPS hardware use;
- GPS heading fusion;
- fused LemLib publication;
- fused navigation execution.

Host and source-policy tests must reject any enabled gate while required
physical results remain `Not run`. This policy is a configuration assertion,
not evidence that the robot is physically safe.

### Memory and build resources

Large planner scratch arrays must not be created on a periodic task stack.
Move them into a bounded, reusable workspace owned by the planner module or
supplied by its caller. Record `sizeof` budgets for the planner, protocol,
localization, and obstacle-map state in a host test. Heap allocation is allowed
only during one-time process construction where ownership is explicit and no
periodic allocation results.

Break the `robot-config.hpp`/odometry include cycle by extracting plain
localization configuration value types into a focused configuration header.
Do not adopt C++ modules or a new package manager: the PROS ARM toolchain and
offline competition workflow do not justify that migration.

## Module Design

### Configuration policy module

The configuration policy is a deep module: callers provide a `RobotConfig`,
and its small interface produces a complete authorization snapshot and a
fail-closed verdict. All knowledge of which flags are safety gates remains
local to this module.

### Timed mutex adapter

Use one small RAII adapter around `pros::Mutex` with:

- a bounded acquisition attempt;
- an observable `ownsLock()` result;
- deleted copy and move operations;
- a `noexcept` destructor that releases only an acquired lock.

This is an internal seam used by localization state publication. It must not
grow into a general locking framework.

### GPS validation module

Separate validation from mutation. The gate first returns a candidate decision
without changing history. The caller commits the accepted sample only after
the EKF accepts it. Tests exercise the module through this interface rather
than editing its internal state.

### Planner workspace

The planner owns one bounded workspace containing its node, edge, cost, and
search arrays. Planning clears and reuses it. The public path-planning
interface and fixed maximum sizes remain unchanged.

## Comments and Brief Style

Public safety-significant interfaces receive concise Doxygen comments:

- one-sentence `@brief` describing observable behavior;
- units in parameter or member descriptions;
- ownership and lifetime only where non-obvious;
- bounded timing, rollover, and failure behavior where relevant;
- no comments that merely repeat a name or narrate syntax.

Private implementation comments explain invariants or surprising decisions,
not line-by-line mechanics. Existing verbose comments may be shortened only in
files otherwise modified by this work.

## Testing and Checkpoints

Every behavioral correction follows red-green-refactor:

1. add a focused regression that fails for the demonstrated defect;
2. run it and record the expected failure;
3. implement the smallest correction;
4. run focused and neighboring suites;
5. commit and push an independently reviewable checkpoint.

Required final verification:

- resource-policy compile-time tests;
- authorization policy and source-policy tests;
- fused localization/navigation host suite;
- established legacy host suite;
- static state-size budget test;
- clean big-robot ARM build;
- clean restored-small ARM build;
- `git diff --check` and a two-axis code review.

Physical tests remain `Not run`. Build and host-test success do not authorize
runtime gates.

## Non-Goals

- Replacing value-oriented modules with inheritance-heavy OOP.
- Converting the repository to C++ modules.
- Adding vcpkg, Conan, or another package manager.
- Rewriting vendored dependencies to remove their warnings.
- Changing controller gains, sensor geometry, route geometry, or field data.
- Enabling GPS, fused LemLib, fused navigation, Shadow, or experimental routes.
- Claiming memory headroom without device measurements.
