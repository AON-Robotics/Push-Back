# Pre-upload reliability audit — 2026-08-16

This is a software-only audit. It does not establish that wiring, mechanisms,
sensor directions, calibration values, or autonomous routes are correct on a
physical robot.

## Repository and subsystem map

- Framework: PROS 4.2.2 C/C++ firmware with PROS competition callbacks.
- Compiler/build: Arm GNU Toolchain 14.3.Rel1, `arm-none-eabi-g++` 14.3.1,
  GNU Make 4.4.1, C++ `gnu++26`, C `gnu23`.
- Third party: LemLib 0.5.6, LVGL 9.2.0, fmt, and a vendored nlohmann JSON
  header.
- Lifecycle: `src/main.cpp` delegates `initialize`, `disabled`,
  `competition_initialize`, `autonomous`, and `opcontrol` to `core::Robot`.
- Hardware owner: `core::Hardware`; compatibility references are exposed by
  `globals.hpp`.
- Drive/localization: DifferentialDrive on the small build; HDrive on the big
  build; legacy three-wheel odometry plus an optional, authorization-gated
  fused EKF/LemLib path.
- Autonomous: GUI-selected native “Kevin” routes are the default; JerryIO,
  Red Six Block, fused localization, automatic fallback, and Shadow playback
  remain explicitly authorization-gated.
- Mechanisms: variant-specific intake/sorter, cart, scorer/trapdoor or
  proximity routing, parking pistons, and an ORBIT subsystem whose ports are
  placeholders.
- Background tasks: GUI, autonomous emergency stop, intake scan, intake sort,
  Shadow recorder, and localization/display tasks when requested. All
  long-running task loops yield to the PROS scheduler.
- Tests: 30 C++ host executables plus source-policy, symbol-boundary, and
  mechanism-caller PowerShell checks.

## A. Executive result

Baseline small-robot firmware built successfully. The first `make all` was
stopped by the audit runner's 120-second command limit while compiling, with no
compiler failure; resuming with `make quick` linked successfully. Baseline
tests all passed.

Final status:

- Small-robot build: PASS (`make all`, clean build).
- Big-robot compile variant: PASS (`make clean`; `make quick
  EXTRA_CXXFLAGS=-DUSING_BIG_ROBOT=1`).
- Host tests: PASS, 30/30 C++ executables.
- Policy/symbol checks: PASS for authorization, isolated LemLib validation,
  mechanism dependency boundaries, and both robot-specific symbol sets.
- Compiler warnings: one unchanged warning in vendored
  `include/aon/tools/json.hpp:12492` for deprecated `std::is_pod`; no team-code
  warning was emitted by either firmware build. It was not suppressed.
- Final small artifact: `bin/monolith.bin`, 1,161,144 bytes. Pre-change
  artifact size was 1,161,032 bytes.
- Severity ledger: **2 CRITICAL, 5 HIGH, 9 MEDIUM, 4 LOW**. Of these, 3 HIGH
  and 3 MEDIUM findings were corrected. The two CRITICAL items are invalid
  placeholder device mappings which cannot be corrected without wiring data.

Remaining upload blockers/risks:

1. ORBIT constructs a rotation sensor, vision sensor, and motor on invalid,
   conflicting smart port 0, and `Configure()` calls `orbit.configure()` at
   every normal boot.
2. The potentiometer is constructed on invalid ADI port `Z`.
3. Big-robot legacy and LemLib right tracking-wheel reversal declarations do
   not agree.
4. Native legacy motion timeouts stop the current drive command but do not
   report failure to the `void` route, so a route can continue to later steps.
5. Pneumatic disabled-state policy and all physical directions/geometry remain
   unverified.

The software is improved, but it is **not ready for an unattended full-field
autonomous run** until the critical port placeholders and hardware-dependent
items are resolved on the actual selected robot.

## Finding ledger

| Severity | Finding | Result |
|---|---|---|
| CRITICAL | ORBIT uses smart port 0 for three device types | Unchanged; wiring required |
| CRITICAL | Potentiometer uses invalid ADI port `Z` | Unchanged; wiring/removal decision required |
| HIGH | GUI/intake/ORBIT cross-task flags used `volatile`, causing C++ data races | Fixed with atomics |
| HIGH | Legacy blocking drive loops could issue commands after field disable | Fixed; disable is a hard loop exit |
| HIGH | `drivePID(0)` divided by zero and `drivePID` had no effective timeout | Fixed |
| HIGH | Big right tracking reversal differs between legacy and LemLib maps | Unchanged; physical direction test required |
| HIGH | Native route motion timeout/failure is not propagated to abort later steps | Unchanged; interface/route validation required |
| MEDIUM | Legacy `follow({})` dereferenced an empty path | Fixed |
| MEDIUM | PID result and default MotionProfile limits were uninitialized | Fixed |
| MEDIUM | HDrive/XDrive/Mecanum experimental `goToPose` has no hard timeout | Disable escape added; timeout still unresolved |
| MEDIUM | ORBIT blocking rotations have no timeout and width-to-distance accepts zero width | Unchanged; subsystem is not safely mapped or enabled |
| MEDIUM | Disabled stops motors but deliberately retains pneumatic command states | Unchanged; mechanical safety intent required |
| MEDIUM | Normal build does not enable broad warning flags | Unchanged; host tests use `-Wall -Wextra -Werror` |
| MEDIUM | `run-host-tests.ps1` covers only a subset of the repository's tests | New safety test added; remaining tests were run manually |
| MEDIUM | PROS CLI/toolchain discovery depends on the VS Code extension environment | Unchanged; direct bundled Make/toolchain was used |
| MEDIUM | Vendored JSON uses deprecated `std::is_pod` under `gnu++26` | Unchanged; dependency upgrade should replace vendored file as a unit |
| LOW | `Pose::theta` documentation conflicts with the established +Y/clockwise convention | Unchanged |
| LOW | GUI guide contains stale paths/API examples | Unchanged |
| LOW | Old, commented controller experiments and unused legacy members remain | Listed, not deleted |
| LOW | Small intake deadline comparison is not wrap-safe after about 49 days uptime | Unchanged; not match-realistic |

## B. Changes made

All production changes below are AI-generated outside code and require student
review, understanding, physical testing, and competition-policy attribution as
applicable.

| Severity | File / region | Original problem | Correction and behavioral effect | Verification |
|---|---|---|---|---|
| HIGH | `include/aon/drivetrain/legacy-motion-safety.hpp`; legacy loops in `drivetrain.hpp` and four drivetrain `.cpp` files | Blocking legacy motion did not treat field disable as an exit; empty paths accessed `back()` | Added a tested, pure continuation rule; disable stops further motion and final heading commands; empty paths stop and return | New truth-table test, small and big ARM builds |
| HIGH | `Drivetrain::drivePID` / `turnPID` | Zero targets divided by zero; `drivePID` computed but ignored its time limit | Zero/invalid-speed calls stop and return; drive PID now uses its existing 3× estimated deadline | ARM builds; native symbol checks |
| HIGH | `Hardware::alliance`, Intake task state, ORBIT task state, GUI selection | `volatile` did not synchronize concurrent task reads/writes | Replaced shared state with `std::atomic` and explicit acquire/release access where values cross tasks; intended values and timing are unchanged | Both compile variants; host suite |
| MEDIUM | `PID::result` | `GetResult()` before the first output/reset read indeterminate state | Initialized result to zero | Both compile variants |
| MEDIUM | `MotionProfile` default constructor | Default limits were indeterminate and could enter invalid arithmetic | Default limits now use the existing selected-robot constants | Both compile variants |
| MEDIUM | `tools/run-host-tests.ps1`, `tests/legacy-motion-safety-test.cpp` | New disable rule needed a hardware-independent regression test | Added the test to the standard host runner | Test observed failing before helper existed, then passed |

No hardware mappings, motor reversals, gains, path points, autonomous distances,
mechanism delays, button mappings, or pneumatic defaults were changed.

## C. Issues found but not changed

- **Wiring:** replace or intentionally remove all ORBIT port-0 devices and the
  ADI-`Z` potentiometer. Do not upload assuming these represent real ports.
- **Big tracking direction:** compare the right tracking wheel's positive
  direction against both `legacyTracking.right = -6` and LemLib port 6 with
  reversal false; update only after a hand-motion test.
- **Geometry:** measure drive wheel diameter, track width, tracking wheel
  diameter, and all three offsets on each actual chassis.
- **Motor direction/cartridge:** verify every signed motor port and the blue
  drive/intake versus green small scorer cartridges.
- **Pneumatics:** decide whether disabled should retain or retract cart,
  scorer, trapdoor, arrow/sem, and brooks. The code currently retains the last
  command; small opcontrol commands arrow extended whenever DOWN is not held.
- **Native autonomous failure flow:** convert legacy moves/routes to return and
  propagate a status before relying on them under fault conditions. This
  changes route control flow and needs route-by-route physical validation.
- **Experimental holonomic and ORBIT controllers:** add physically justified
  deadlines and sensor-failure behavior before enabling; ORBIT also needs a
  zero/invalid vision-width policy.
- **Tuning:** PID gains, motion-profile acceleration/deceleration, EKF noise,
  confidence gates, lookahead, stall thresholds, and autonomous timing were
  not retuned from source inspection.
- **Calibration/API assumptions:** validate LemLib calibration completion and
  sensor status on-brain. The custom fused IMU calibration has a 3000 ms
  timeout; the default LemLib path depends on the installed binary library.
- **Routes:** Red/Blue AUT3, fused localization, automatic encoder fallback,
  and Shadow playback remain locked pending physical validation. Do not bypass
  the authorization flags to test them casually.
- **SD card:** Shadow recording/playback requires real SD presence, capacity,
  write, reboot persistence, corruption, and power-loss testing.
- **Driver preference:** confirm joystick signs, curvature behavior, turbo
  buttons, double-tap lever behavior, sorting mode, and every mechanism button
  with the actual drivers.

## D. Hardware configuration table

Signed smart ports indicate the code's reversal setting. “Uncertain” means the
mapping must be compared with the physical robot.

### Small robot (`USING_BIG_ROBOT=false`, default build)

| Device | Port(s) | Reversal / gear / configuration |
|---|---:|---|
| Left drive motors | `11, -12, 13, -14` | Signed reversals; blue cartridge; hold in auton, brake in driver |
| Right drive motors | `1, -2, 3, -4` | Signed reversals; blue cartridge; hold in auton, brake in driver |
| Left tracking rotation | `19` | Legacy positive; LemLib not reversed |
| Right tracking rotation | `-18` / physical 18 | Legacy negative; LemLib reversed=true |
| Back tracking rotation | `5` | Not reversed |
| IMU | `16` | LemLib and legacy |
| Corridor motor | `-9` | Reversed; blue; coast |
| Elevator motor | `-6` | Reversed; blue; coast |
| Judge motor | `7` | Not reversed; blue; coast |
| Scorer motor | `-8` | Reversed; green; hold |
| Intake distance | `20` | Detection threshold 45 mm (PROS distance units) |
| Intake optical | `17` | Hue ranges: red 356–359 or 1–25; blue 170–230 |
| Scorer pneumatic | ADI `H` | Starts retracted |
| Cart pneumatic | ADI `B` | Starts retracted |
| Trapdoor pneumatic | ADI `A` | Starts retracted |
| Arrow pneumatic | ADI `C` | Starts retracted; opcontrol extends unless DOWN held |
| Brooks/park pneumatic | ADI `G` | Starts retracted |
| Controller | Master | LemLib curvature drive; Kevin mappings |
| ORBIT rotation / vision / motor | `0 / 0 / 0` | **CRITICAL: invalid and conflicting placeholders** |
| Potentiometer | ADI `Z` | **CRITICAL: invalid placeholder** |

Key geometry in code: 2.75 in drive wheel, 12.5 in track width, effective 450
RPM drivetrain, 2.0 in tracking wheels, left/right offsets -1.125/+1.125 in,
back offset -1.572 in. All require physical measurement.

### Big robot (`USING_BIG_ROBOT=true`)

| Device | Port(s) | Reversal / gear / configuration |
|---|---:|---|
| Left drive motors | `12, -13, -18, 19` | Signed reversals; blue cartridge |
| Right drive motors | `-1, 2, 3, -4` | Signed reversals; blue cartridge |
| H-drive middle motor | `-15` | Reversed; blue cartridge |
| Left tracking rotation | `5` | Legacy/LemLib not reversed |
| Right tracking rotation | `-6` / physical 6 | **Uncertain conflict:** legacy reversed, LemLib reversed=false |
| Back tracking rotation | `7` | Not reversed |
| IMU | `14` | LemLib and legacy |
| Elevator motors | `20, -11, -10` | Signed reversals; blue; brake |
| Judge motor | `17` | Not reversed; blue; brake |
| Intake distance | `9` | Detection threshold 45 mm |
| Intake optical | `16` | Same code hue ranges as small robot |
| Accept proximity | ADI `F` | Digital proximity input |
| Reject proximity | ADI `E` | Digital proximity input |
| Cart pneumatic | ADI `H` | Starts retracted |
| Sem pneumatic | ADI `G` | Starts retracted |
| Brooks/park pneumatic | ADI `D` | Starts retracted |
| Controller | Master | Fabian holonomic mappings |
| ORBIT rotation / vision / motor | `0 / 0 / 0` | **CRITICAL: invalid and conflicting placeholders** |
| Potentiometer | ADI `Z` | **CRITICAL: invalid placeholder** |

Key geometry in code: 3.25 in legacy drive wheel constant, 15.5 in drive
width, 2.0 in tracking wheels, offsets ±1.572/1.572 in; LemLib configuration
must be compared separately with the chassis. No duplicate **valid** smart or
ADI ports were found within either selected variant.

## E. Physical robot pre-upload checklist

- [ ] Put the correct robot on blocks with all drive wheels safely clear.
- [ ] Confirm the intended small/big compile configuration and program slot.
- [ ] Resolve ORBIT smart port 0 and potentiometer ADI `Z` before upload.
- [ ] Compare every smart/ADI port in section D with actual wiring.
- [ ] Confirm every motor reversal and physical forward direction.
- [ ] Confirm blue/green motor cartridges and configured gearsets.
- [ ] Confirm Brain firmware is compatible with PROS 4.2.2 and LemLib 0.5.6.
- [ ] Confirm charged battery, controller pairing, and radio link.
- [ ] Check every sensor connection and Brain device listing.
- [ ] Leave the robot stationary during IMU calibration; verify completion.
- [ ] Hand-check tracking sensor directions, especially big right port 6.
- [ ] Measure/confirm tracking wheel diameter, offsets, drive diameter, and
      track width.
- [ ] Confirm all pneumatic startup states and safe mechanical clearances.
- [ ] Keep immediate access to Brain/battery power and controller X stop.
- [ ] Confirm autonomous alliance/slot selection and that locked routes remain
      locked unless the team is deliberately validating them.
- [ ] Place the robot in the exact route starting pose and heading.
- [ ] Check mechanism freedom, air pressure, motor temperature, and stalled
      loads before every autonomous test.

## F. First powered test sequence

Stop immediately on an unexpected direction, device error, sustained stall,
unexpected pneumatic motion, or loss of control.

1. Power the robot with drive wheels safely off the ground and mechanisms
   clear.
2. Verify Brain startup completes, the intended program is selected, and no
   unexpected task continuously commands hardware.
3. Inspect the Brain device list and verify every configured sensor/device;
   resolve port-0/ADI-`Z` errors first.
4. Command every motor individually at low output for a brief pulse.
5. Verify each motor's positive direction and signed reversal.
6. Verify forward, reverse, clockwise turn, counter-clockwise turn, and big
   robot strafe at low output.
7. Pulse every pneumatic output individually while watching clearances; record
   the real meaning of extended/retracted.
8. Verify every controller button, held action, edge-triggered toggle,
   double-tap, turbo, sorter mode, and emergency stop.
9. With motors disabled, move the robot by hand and inspect left/right/back
   tracking changes and odometry X/Y.
10. Rotate clockwise and counter-clockwise by hand; verify IMU and odometry
    heading sign and wrap behavior.
11. Run very short closed-loop forward/reverse and turn movements with a clear
    stop zone; confirm timeout and field-disable stop behavior.
12. Exercise autonomous mechanisms without full autonomous driving.
13. Run the selected native autonomous on blocks or in a constrained clear
    area with reduced-risk starting conditions and a spotter at power.
14. Run full autonomous only after every preceding check passes; validate one
    route and one phase at a time.
15. Run driver control, including transitions from autonomous and disabled.
16. Power-cycle the Brain and repeat startup, calibration, device, pneumatic,
    selector, and safe-stop checks.

Only the student team performing these steps on the real robot can establish
physical verification. This audit does not guarantee a successful first
upload or autonomous run.
