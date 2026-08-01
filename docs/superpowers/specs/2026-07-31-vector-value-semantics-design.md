# Vector Value Semantics Design

## Purpose

Make `aon::Vector` a deterministic value type without changing its public
method names, mathematical conventions, or numeric results. The refactor
removes heap allocation from vector construction and mutation, eliminates
leaks, and makes copied vectors own independent direction state.

## Current problem

`aon::Vector` stores its direction as an `Angle*` initialized with `new`.
`SetDirection`, `SetDegrees`, and `SetRadians` allocate additional `Angle`
objects without releasing prior allocations. The class has no destructor or
copy/move policy, so compiler-generated copies share the same pointer.

This is especially risky because vectors are copied by odometry, drivetrain,
and geometry code. A direction change through one copy can affect another
copy, and routine motion calculations repeatedly allocate memory.

## Approved behavior

`Vector` has value semantics. Copy construction and copy assignment copy the
direction value. Mutating a copy does not affect the original.

This deliberately corrects the existing pointer-aliasing behavior. That
aliasing is not part of the supported interface.

All other observable behavior remains unchanged:

- `SetX`, `SetY`, and `SetPosition` derive magnitude with `std::hypot` and
  direction with `std::atan2`.
- `SetMagnitude` keeps the existing absolute stored magnitude while using the
  supplied signed value to calculate Cartesian components.
- Degree/radian conversion continues to be performed by `Angle`; values are
  not normalized or wrapped.
- Arithmetic operators retain their existing Cartesian or polar meanings.
- Existing public method names, parameter types, return types, implicit
  conversions, and stream output remain source-compatible.
- No odometry formulas, drivetrain commands, timing, units, hardware ports,
  controller gains, or task synchronization change in this stage.

## Design

Replace the private `Angle* direction` member with an inline `Angle direction`
member. Update internal member access from pointer syntax to value syntax.

`SetDirection(Angle*)` remains available for source compatibility. It copies
the supplied angle into the inline member; it does not retain or take ownership
of the pointer. Existing callers therefore keep ownership and lifetime
responsibility for the argument. The method retains its current requirement
that the pointer be non-null; null handling is not added because that would be
a separate behavioral and error-handling change.

`SetDegrees` and `SetRadians` update the inline member directly. No new
abstraction, interface class, dependency, or error mechanism is introduced.

## Test seam

Add a dependency-free Windows host test that includes only
`aon/tools/vector.hpp` and the standard library. The test exercises the public
`Angle` and `Vector` interfaces rather than private representation.

The required red test copies a vector, changes the copy's direction, and
asserts that the original direction and components remain unchanged. It must
fail against the pointer-owning implementation because both instances share
direction state.

The green suite also covers:

- default construction;
- Cartesian-to-polar conversion on axes and quadrants;
- polar-to-Cartesian conversion in degrees and radians;
- zero and negative magnitude behavior;
- copy construction and copy assignment independence;
- addition, subtraction, vector/scalar multiplication, division, dot product,
  and normalization;
- degree/radian conversion for values outside one revolution, preserving
  `Angle`'s current lack of automatic wrapping;
- representative chained setters used by drivetrain and odometry callers.

Tests compile as C++17 with GCC 13.1 and `-Wall -Wextra -Werror`, matching the
existing host-test convention. The complete existing host suite and the PROS
embedded build run before the implementation checkpoint is committed.

## Documentation

Update the public `Vector` class documentation to state that it is a value type
and that `SetDirection` copies from a required, non-owning pointer. Comments
will describe ownership and compatibility intent rather than restating code.

## Checkpoint boundary

The implementation checkpoint contains only:

- the new Vector host test;
- the `Vector` direction ownership refactor;
- focused public ownership documentation;
- the implementation plan for this stage.

Unrelated formatting, naming cleanup, legacy drivetrain edits, and hardware
configuration changes are excluded.
