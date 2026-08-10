# Localization and EKF Design

## Goal

Provide one reliable localization estimate for autonomous motion on both AON
robots. Correct tracking-wheel geometry, angle handling, sensor validity,
timing, resets, and pose publication before adding a lightweight three-state
extended Kalman filter. Existing autonomous route bodies continue using
`aon::auton::Actions`; they do not learn about the filter.

## Current Architecture

Normal competition startup calibrates and uses the LemLib 0.5.6 chassis.
LemLib supplies the pose consumed by driver control, autonomous actions, path
following, Shadow recording, and motion-health monitoring. The older
`aon::Odometry` implementation starts only when a native route calls
`aon::legacy_motion::prepare()`.

The legacy implementation is not a sound basis for sensor fusion:

- it does not sample the configured back tracking wheel during updates;
- it uses a coordinate transform incompatible with the active LemLib frame;
- its IMU delta is not unwrapped across the heading boundary;
- its fixed gyro-confidence blend is one, making encoder heading dead work;
- it publishes position and heading behind separate, incorrectly used locks;
- it retains a duplicate experimental `changeWeb` position;
- runtime reset tares the IMU and blocks for three seconds;
- GPS blocks, returns meters into an inches interface, and is constructed on
  invalid port zero;
- its loop uses relative delays and therefore drifts with execution time.

Improving only the legacy class would not improve the normal LemLib path
follower. The finished design therefore has one AON estimator feeding the pose
used by LemLib motion while keeping native routines as a temporary compatibility
path until their existing deletion gate is satisfied.

## Coordinate and Unit Contract

The estimator adopts the already documented LemLib competition frame:

- field `+X` points right;
- field `+Y` points forward from a zero-heading start;
- zero heading faces `+Y`;
- positive heading is clockwise;
- public positions are inches;
- public headings are degrees;
- internal angles and angular covariance are radians.

Robot-local translation is represented as rightward displacement `r` and
forward displacement `f`. Every type and configuration field includes its unit
in either its name or documentation. GPS meters are converted once in the
sensor adapter before entering estimator math.

## Ownership and Module Design

The localization cluster becomes one deep module with a small public interface:

```cpp
Pose getPose() const;
Pose rawOdometryPose() const;
void resetPose(double xInches, double yInches, double headingDegrees);
void update();
LocalizationDiagnostics getDiagnostics() const;
```

Internal responsibilities are separated as follows:

- `sensor-measurements.hpp` defines fixed-size wheel, IMU, GPS, validity, and
  timestamp measurements without depending on PROS.
- `pose-estimator.hpp/.cpp` implements platform-independent three-wheel motion
  and the corrected raw pose.
- `ekf.hpp/.cpp` implements allocation-free three-state prediction and
  measurement correction.
- `odometry.hpp/.cpp` adapts PROS sensors, owns update/reset sequencing,
  maintains one coherent published snapshot, and runs the deterministic loop.
- robot configuration owns geometry, noise, validity gates, periods, and GPS
  enablement.
- `chassis.cpp` continues owning the physical drive and tracking devices and
  adapts the fused pose to the LemLib motion stack.

Physical devices must be constructed exactly once. No high-frequency path uses
dynamic allocation, `std::vector`, streams, or a general matrix library.

## Three-Wheel Motion Model

Let the left and right longitudinal wheel offsets be `xL < 0` and `xR > 0`.
Let the lateral/back wheel have forward offset `yB < 0` when mounted behind the
tracking center. With signed measured distances `dL`, `dR`, and `dB`, clockwise
wheel-derived rotation is:

```text
dTheta = (dL - dR) / (xR - xL)
```

Center forward and rightward motion are:

```text
dForward = ((dL + xL * dTheta) + (dR + xR * dTheta)) / 2
dRight   = dB - yB * dTheta
```

For `alpha = theta + dTheta / 2` and
`scale = sinc(dTheta / 2)`, pose propagation is:

```text
x'     = x + scale * (dRight * cos(alpha) + dForward * sin(alpha))
y'     = y + scale * (-dRight * sin(alpha) + dForward * cos(alpha))
theta' = wrap(theta + dTheta)
```

`sinc(z)` uses a small-angle series near zero, so the same equation handles
stationary, straight, arc, rotation, lateral, and combined motion without a
motion-type branch.

Corrected raw odometry uses the same model and measurement layer. It remains
visible for diagnostics and benchmarking but is not a second pose publisher.

## Sensor Preprocessing

One update reads each required device once and converts it into a local
measurement snapshot. The adapter:

- converts rotation centidegrees to inches using the configured effective
  tracking-wheel diameter;
- calculates increments against reset-safe baselines;
- rejects PROS error sentinels and non-finite values;
- reports independent validity for every wheel, IMU, and GPS field;
- converts the IMU reading into the field frame using a maintained offset;
- unwraps angular deltas with the shortest signed difference;
- converts GPS meters to inches before validation;
- applies a minimum GPS sampling interval so one sensor sample is never fused
  repeatedly at the faster localization rate;
- rejects GPS samples outside configured bounds, with excessive reported
  error, impossible displacement, implausible heading change, or excessive
  normalized innovation.

GPS rejection is observable in diagnostics. It never overwrites pose and never
blocks the localization task.

If one longitudinal wheel is invalid, prediction is skipped rather than
silently substituting a different model. An invalid back wheel suppresses
lateral prediction for differential-drive compatibility but is a reported
degraded condition. An invalid IMU skips only the IMU update. GPS loss skips
only GPS correction.

## EKF State and Prediction

The minimum state is:

```text
state = [x, y, theta]^T
```

Velocity and gyro bias are not included until recorded measurements show that
their benefit justifies their tuning and maintenance cost.

Prediction uses wheel-derived `dRight`, `dForward`, and `dTheta`. The nonlinear
motion model above is `g(state, input)`. Its state Jacobian is:

```text
F = [1  0  scale * (-dRight*sin(alpha) + dForward*cos(alpha))]
    [0  1  scale * (-dRight*cos(alpha) - dForward*sin(alpha))]
    [0  0  1]
```

Covariance prediction is:

```text
Pminus = F * P * F^T + Q
```

`Q` contains a configurable stationary floor plus configurable distance- and
rotation-dependent contributions. This prevents false certainty while allowing
uncertainty to grow with the motion most likely to create slip.

## Measurement Updates

### IMU heading

The IMU is an absolute heading measurement with:

```text
H = [0 0 1]
innovation = wrap(imuHeading - predictedTheta)
S = PthetaTheta + Rimu
K = P[:, theta] / S
```

Prediction deliberately uses wheel-derived rotation. Using IMU delta in the
prediction and then applying the same IMU as a measurement would double-count
one sensor and produce unjustified confidence.

### GPS position

GPS position uses:

```text
H = [1 0 0]
    [0 1 0]
```

The explicit two-by-two innovation covariance is inverted only when its
determinant exceeds a configurable numerical tolerance. A normalized
innovation gate complements the physical validity checks.

### GPS heading

GPS heading is implemented as a separate wrapped scalar update with its own
noise value and enable flag. It remains disabled until repeated robot tests
show that it improves heading accuracy or repeatability.

### Covariance stability

Every accepted measurement uses Joseph form:

```text
P = (I - K*H) * Pminus * (I - K*H)^T + K * R * K^T
```

After prediction and correction, the implementation normalizes heading,
symmetrizes covariance, checks every value for finiteness, and permits clamping
only tiny negative diagonal values attributable to floating-point rounding. A
failed numerical guard preserves the last valid state and increments a
diagnostic counter.

## Reset and Timing Semantics

Boot calibration remains distinct from runtime pose reset. Runtime reset:

1. serializes against the update publisher;
2. snapshots current wheel and IMU readings;
3. establishes new wheel baselines;
4. sets the IMU-to-field heading offset without taring or recalibrating;
5. resets raw and fused states to the requested pose;
6. resets covariance to configured initial uncertainty;
7. publishes one coherent snapshot.

`update()` performs exactly one nonblocking cycle. A separate loop uses a
monotonic 10 ms release grid with `pros::delay_until`, records actual `dt`,
maximum execution time, and deadline misses, and never performs terminal or SD
I/O in the critical section.

One mutex protects the complete published snapshot. Calculations occur in local
fixed-size values; the writer holds the mutex only while copying the completed
pose, covariance diagonal, and diagnostics. `getX()`, `getY()`, and heading
getters derive from one `getPose()` snapshot rather than acquiring independent
locks.

## LemLib Integration Gate

LemLib is supplied as an opaque static library and has no documented external
pose-provider interface. Before enabling the custom estimator in competition,
an isolated integration checkpoint must demonstrate all of the following:

- AON remains the only active wheel/IMU pose propagator.
- LemLib motion reads the fused pose through its existing pose seam.
- LemLib does not overwrite fused pose between AON publications.
- Reset is atomic from both AON and LemLib callers.
- Exactly one task owns localization sensor sampling in normal operation.
- Existing cancellation, fallback monitoring, Shadow recording, and route
  interfaces continue to observe a coherent pose.

The preferred adapter disables LemLib wheel odometry and publishes the AON pose
through `setPose()`. If LemLib 0.5.6 cannot support that deterministically, work
stops at this gate. The project will not run two competing pose writers and
will not replace autonomous motion control without a separately approved
design.

## Diagnostics and Logging

`LocalizationDiagnostics` contains fixed-size values for:

- raw pose and fused pose;
- wheel distances and validity;
- wheel-derived angular increment;
- IMU field heading and validity;
- latest GPS pose, reported error, freshness, and validity;
- GPS accepted/rejected status and typed rejection reason;
- covariance diagonal;
- loop `dt`, execution time, and deadline misses;
- sensor, numerical, and reset counters.

Normal competition operation prints nothing. An explicit development logger
may format snapshots as CSV outside the localization loop. Direct SD logging is
added only if it can share the existing SD architecture without blocking the
estimator or conflicting with Shadow recording.

## Configuration Requiring Measurement or Tuning

The following are configuration, not source-file magic numbers:

- effective tracking-wheel diameters;
- signed left, right, and back offsets;
- sensor reversal states;
- localization and GPS sampling periods;
- initial covariance;
- stationary and motion-dependent process noise;
- IMU heading noise;
- GPS position and heading noise;
- GPS port, mounting offsets, and mounting-heading offset;
- GPS reported-error, field-bound, jump, heading-jump, and innovation gates;
- numerical singularity tolerance.

Current tracking geometry is treated as unverified. The big robot's known
right-tracker reversal disagreement is preserved and reported until physical
calibration resolves it. GPS remains disabled because the repository currently
configures invalid port zero and does not provide verified mounting geometry.

## Verification

Platform-independent host tests cover:

1. zero motion;
2. straight forward and backward motion;
3. pure clockwise and counterclockwise rotation;
4. constant-radius arcs;
5. lateral motion;
6. combined translation and rotation;
7. `359` to `0` and `-180` to `+180` angle crossings;
8. temporary IMU and GPS loss;
9. stale GPS samples and large GPS outliers;
10. correction after accumulated dead-reckoning drift;
11. repeated resets;
12. stationary covariance behavior;
13. finite, symmetric covariance with non-negative diagonal;
14. singular innovation rejection;
15. exact coordinate and unit conventions.

Robot validation compares the existing LemLib baseline, corrected raw
odometry, and fused pose over repeated 72-inch drives, 90-degree turns, square
paths, figure eights, lateral H-drive motion, and return-to-start tests. Each
configuration records final X/Y/heading error and repeatability across at least
five runs. Twenty runs are preferred for tuning noise values. No documentation
claims that the EKF improves localization until measured results support it.

## Safety and Scope

- Do not modify autonomous route geometry or mechanism behavior.
- Do not authorize currently gated fallback, Shadow playback, or competition
  routes as part of localization work.
- Do not edit vendored PROS, LemLib, LVGL, fmt, or firmware files.
- Preserve the native compatibility path until its existing physical deletion
  gate is satisfied.
- Make one behavior, calibration category, or filter stage per commit.
- Run host tests and both robot builds at every applicable checkpoint.
- Restore the small-robot configuration before every commit.
- Stop at physical measurement and LemLib integration gates rather than
  inventing geometry or claiming unmeasured performance.

