# Hardware Map Consistency Design

## Purpose

Remove duplicated drivetrain and tracking-device port literals from the legacy
hardware owner and LemLib configuration without changing any hardware command,
port, reversal, calibration, authorization flag, or public robot-control API.
Add host-side validation that makes disagreement between the two odometry paths
explicit before a robot is uploaded.

This phase does not cross the physical gate in `docs/CURRENT_HANDOFF.md`.
It does not authorize encoder fallback, alter autonomous selection, unlock
Shadow playback, or correct any observed configuration discrepancy.

## Evidence and Current Risk

`src/aon/core/hardware.cpp` and `src/aon/config/robot-config.cpp` independently
repeat the same eight drive-motor ports and four tracking-device ports for each
robot. A future edit can therefore update the legacy drivetrain while leaving
LemLib on stale hardware values.

The current small-robot tracking configuration is semantically consistent:
legacy port `-18` and LemLib port `18` with `rightReversed=true` both select
physical port 18 in the reversed direction.

The current big-robot configuration is not semantically consistent. Legacy
odometry receives right tracker `-6` and calls `set_reversed(true)`. LemLib
constructs its sensor with `-6`, then unconditionally calls
`set_reversed(false)`. The PROS API documents negative rotation ports as
reversed, while `set_reversed(false)` explicitly clears that direction. The
new validator must report `RightTrackingReversalMismatch` for the big map.
Changing either value is a hardware-facing behavioral change and is excluded
from this phase.

## Considered Approaches

### Shared dependency-free map and pure validator — selected

Create one small configuration module containing the raw values consumed by
both legacy hardware construction and LemLib configuration. A pure validator
compares the two tracking representations by physical port and final reversal
state. Host tests exercise both robot maps without constructing PROS devices.

This removes the demonstrated duplication, preserves current construction and
ownership, and creates a seam for later hardware validation.

### Source-text consistency test — rejected

A test could parse the two `.cpp` files and compare numeric literals. Such a
test would be coupled to formatting and implementation text rather than the
configuration contract. It would also become obsolete as soon as either file
is reorganized.

### Full hardware factory with injected PROS devices — deferred

Changing drivetrain constructors and device ownership would produce a cleaner
long-term boundary, but it would touch real-time initialization order, PROS
device lifetime, and public constructor APIs. That scope is unjustified before
the pending physical baseline gate.

## Architecture

Create `include/aon/config/hardware-map.hpp` as a dependency-free data and
validation interface in namespace `aon::config`. It contains:

- the existing `DrivePorts` and `TrackingPorts` public types, moved without
  renaming from `robot-config.hpp`;
- `LegacyTrackingPorts`, which records the signed ports passed to legacy
  `Odometry`;
- `RobotHardwareMap`, containing one shared drive map plus the legacy and
  LemLib tracking representations;
- `HardwareMapIssue`, an `enum class` whose values distinguish invalid ports,
  per-sensor physical-port mismatches, and per-sensor reversal mismatches;
- `smallRobotHardwareMap` and `bigRobotHardwareMap` important configuration
  values; and
- `[[nodiscard]] HardwareMapIssue validateHardwareMap(const
  RobotHardwareMap&) noexcept`.

Implement validation in `src/aon/config/hardware-map.cpp`. Validation is pure:
it does not initialize devices, write logs, mutate configuration, or command
hardware. Signed legacy tracking ports derive reversal from `port < 0`.
LemLib tracking ports derive physical identity from the absolute port number
and final reversal from the explicit `*Reversed` member because
`chassis.cpp` calls `set_reversed` after construction.

`Hardware::Hardware()` will expand the shared drive arrays into its existing
initializer-list constructor and pass the shared legacy tracking values to
`Odometry`. `activeRobotConfig()` will copy the same drive map and the LemLib
tracking representation into the existing `RobotConfig`. No constructor
signature, object lifetime, member order, or selected-robot macro changes.

## Behavior and Compatibility

The following must remain byte-for-byte equivalent as configuration values:

- small drive ports: left `{11, -12, 13, -14}`, right `{1, -2, 3, -4}`;
- big drive ports: left `{12, -13, -18, 19}`, right `{-1, 2, 3, -4}`;
- small legacy tracking: `{19, -18, 5, 16}`;
- big legacy tracking: `{5, -6, 7, 14}`;
- small LemLib tracking: `{19, 18, 5, 16, false, true, false}`;
- big LemLib tracking: `{5, -6, 7, 14, false, false, false}`.

The current big-robot mismatch remains observable and documented. The small
map must validate as `None`; the big map must validate as
`RightTrackingReversalMismatch`. The validator reports but never repairs.

The project remains C++17. No dependency or language-standard change is
introduced.

## Testing

Add `tests/hardware-map-test.cpp` before creating the production interface.
The initial compile must fail because `aon/config/hardware-map.hpp` does not
exist.

The test uses literal, independently reviewed expectations to characterize all
current drive and tracking values. It verifies:

- the small map is consistent;
- the big map reports exactly the known right-reversal mismatch;
- a copied small map with a wrong right physical port reports the right-port
  mismatch;
- a copied small map with a wrong explicit reversal reports the right-reversal
  mismatch; and
- port zero and absolute port values above 21 report `InvalidPort`.

After the focused red-green cycle, rebuild and run all existing host tests plus
the new test with GCC 13.1, `-std=c++17 -Wall -Wextra -Werror`. Clean-build the
small and big PROS configurations, restore `USING_BIG_ROBOT false`, and verify
the final small build before committing.

## Documentation and Failure Handling

All new public types, enum values, configuration objects, and the validator
receive Doxygen comments describing signed-port semantics, ownership, side
effects, and the fact that validation does not authorize hardware use.

Validation returns an enum rather than throwing, logging, or aborting. This
preserves the project’s current error-handling strategy and lets later callers
decide whether a mismatch is informational or fail-closed. This phase adds no
runtime caller; the host test is the only consumer of the diagnostic result.
