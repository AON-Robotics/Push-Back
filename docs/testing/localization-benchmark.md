# Localization Validation and Benchmark

## Current status

The platform-independent three-wheel model, three-state EKF, GPS gates,
diagnostics, coherent runtime pose, and authorization-gated LemLib adapter are
implemented and host/build verified. They have not been validated on either
physical robot.

The final review also verified reset/update serialization, recovery-cycle
rebaselining after a wheel sensor fault, changed-observation GPS freshness,
transactional GPS gating, covariance symmetrization, and fail-closed autonomous
startup after an IMU calibration failure.

- `fusedLemLibAuthorized` remains `false` for both robots.
- GPS remains disabled with port `0` for both robots.
- The competition path follower therefore continues to use the preserved
  LemLib wheel/IMU odometry path.
- No claim that the EKF improves accuracy is authorized yet.

## Coordinate and unit contract

- Public position: inches.
- Public heading: degrees.
- Estimator heading: radians.
- Field origin: center of the field.
- `+X`: right/east when the robot heading is zero.
- `+Y`: forward/north when the robot heading is zero.
- Heading zero: faces `+Y`.
- Positive heading: clockwise.
- Robot-local displacement is `(right, forward)`.

One `Odometry::getPose()` call returns a mutually consistent fused X, Y, and
heading snapshot. `rawOdometryPose()` returns the corresponding uncorrected
wheel pose. Runtime `resetPose()` changes sensor baselines and the IMU field
offset without taring, calibrating, or sleeping.

## Implemented motion and filter equations

For left, right, and back incremental tracking distances `dL`, `dR`, and `dB`,
with signed tracking offsets `xL`, `xR`, and `yB`:

```text
dTheta   = (dL - dR) / (xR - xL)
dForward = ((dL + xL*dTheta) + (dR + xR*dTheta)) / 2
dRight   = dB - yB*dTheta
```

With `alpha = theta + dTheta/2` and
`scale = sinc(dTheta/2)`:

```text
x'     = x + scale*(dRight*cos(alpha) + dForward*sin(alpha))
y'     = y + scale*(-dRight*sin(alpha) + dForward*cos(alpha))
theta' = wrap(theta + dTheta)
```

The EKF state is deliberately limited to:

```text
state = [x, y, theta]^T
```

Prediction uses the same nonlinear propagation. If `dx` and `dy` are its
field increments, the state Jacobian is:

```text
F = [1  0   dy]
    [0  1  -dx]
    [0  0    1]

P' = F*P*F^T + Q
```

The IMU and optional GPS-heading corrections use wrapped scalar innovations:

```text
innovation = shortestAngleDelta(theta, measurement)
S          = Ptheta,theta + R
K          = P[:,theta] / S
```

GPS position uses an explicit inverse of the 2-by-2 X/Y innovation matrix and
rejects excessive normalized innovation squared (NIS). Every measurement
correction uses the Joseph covariance form:

```text
P' = (I - K*H)*P*(I - K*H)^T + K*R*K^T
```

This costs only fixed 3-by-3 storage and bounded loops; there is no dynamic
allocation, general matrix package, terminal output, or SD I/O in `update()`.

## Values requiring measurement or tuning

### Geometry and hardware

| Value | Small robot | Big robot | Status |
|---|---:|---:|---|
| Tracking diameter | 2.000 in | 2.000 in | Existing nominal value; measure loaded effective diameter |
| Left offset `xL` | -1.125 in | -1.572 in | Existing nominal value; full-turn measurement required |
| Right offset `xR` | +1.125 in | +1.572 in | Existing nominal value; full-turn measurement required |
| Back offset `yB` | -1.572 in | -1.572 in | Existing nominal value; full-turn measurement required |
| Right tracker reversal | Legacy/LemLib agree | Known legacy/LemLib mismatch | Do not resolve without a signed physical test |
| GPS port and mounting | Unknown | Unknown | Disabled; do not invent |

For a pure rotation using this model, calculate candidate offsets from recorded
signed deltas:

```text
xL = -dL/dTheta
xR = -dR/dTheta
yB =  dB/dTheta
```

Average clockwise and counter-clockwise measurements only when their signs and
magnitudes repeat. Do not tune geometry from one run.

### Initial filter values

These are conservative starting values, not measured claims:

| Configuration value | Current value | Required evidence |
|---|---:|---|
| Initial position variance | 4.0 in^2 | Reset repeatability |
| Initial heading variance | `(5 deg)^2` | Reset/IMU alignment repeatability |
| Stationary position variance | `1e-6 in^2/update` | 30-second stationary wheel noise |
| Stationary heading variance | `1e-8 rad^2/update` | 30-second stationary IMU noise |
| Position variance per inch | `0.01 in^2/in` | Repeated-path residuals |
| Heading variance per radian | `0.01 rad^2/rad` | Repeated turn residuals |
| IMU heading variance | `(2 deg)^2` | Stationary and turn residual variance |
| GPS position variance | `4.0 in^2` | At least 20 fixed-location samples |
| GPS heading variance | `(8 deg)^2` | Separate enabled/disabled comparison |
| Loop period | 10 ms | Deadline diagnostics under competition load |

GPS bounds, maximum reported error, jump thresholds, sample period, and NIS
limits are also configuration values. They must be tuned one category at a
time from logged rejected and accepted samples.

PROS GPS does not expose a hardware sample timestamp. The adapter therefore
marks a reading fresh only when its raw position, heading, error, or validity
signature changes, then applies the configured minimum sample period. This is
conservative while stationary and prevents identical polls from repeatedly
shrinking covariance.

## Required physical integration gate

### 1. Preserve and record the existing baseline

On a fresh small-robot boot, first run the pending LemLib figure-eight and one
native Kevin fallback described in `docs/CURRENT_HANDOFF.md`. Record final pose,
completion, crossover behavior, GUI behavior, mechanisms, and cancellation.
Stop if this baseline fails.

### 2. Measure signs and geometry with fusion disabled

1. Mark the tracking-center starting point and record raw sensor values.
2. Push the robot forward 24 inches. Left and right deltas must have the same
   configured forward sign; the back delta should be near zero.
3. Push backward 24 inches and require opposite, repeatable signs.
4. Move the H-drive right and left without rotation; the back tracker must
   report the expected sign and longitudinal trackers should remain near zero.
5. Perform one slow full clockwise and counter-clockwise turn. Record all three
   signed wheel deltas and continuous IMU rotation.
6. Calculate offsets with the equations above and repeat until the result is
   stable. Commit each proven geometry/sign category separately.

### 3. Validate corrected raw pose

With the AON estimator running in an isolated diagnostic build, run:

1. stationary for 30 seconds;
2. five repeated runtime resets;
3. forward and reverse 12 inches;
4. clockwise and counter-clockwise 90-degree turns;
5. one constant-radius arc;
6. right and left lateral motion on the H-drive;
7. combined translation and rotation.

Require finite covariance, correct signs, no multi-second reset pause, and zero
unexplained deadline misses.

### 4. Prove the LemLib integration seam

Only after steps 1-3 pass, set `fusedLemLibAuthorized=true` on an isolated
small-robot validation build. Confirm:

- LemLib pose equals each published fused snapshot;
- pausing AON publication causes no independent LemLib pose drift;
- only one AON localization task exists;
- stationary pose remains within the measured noise band;
- controller-X cancellation and action monitoring remain functional.

Any independent drift proves that LemLib still has a competing writer. Restore
authorization to false and redesign; do not compensate by publishing more
often or inflating noise.

Repeat the full gate independently for the big robot. Never copy the small
robot's measured geometry or authorization.

## GPS gate

If no GPS is installed, record that fact and leave it disabled. If installed,
record its smart port, X/Y mounting offset in meters, heading offset, field
alignment, and reported error at multiple known positions.

Enable GPS position first and keep GPS heading disabled. Test stationary data,
temporary obstruction/disconnection, impossible jumps, field bounds, recovery,
and correction after dead-reckoning drift. Tune position variance from at least
20 residual samples. Evaluate GPS heading in a separate five-run comparison and
enable it only if measured heading error and repeatability improve.

## Repeatable A/B/C benchmark

Run at least five trials per robot and mode:

- straight 72-inch forward and reverse;
- clockwise and counter-clockwise 90-degree turns;
- square;
- constant-radius arc;
- figure eight;
- return to start;
- lateral and combined motion on the H-drive.

Modes:

- A: preserved LemLib/current baseline;
- B: corrected raw three-wheel pose;
- C: EKF fused pose.

Record final X error, Y error, heading error, duration, deadline misses, GPS
accepted/rejected counts, and covariance diagonal. Calculate mean, standard
deviation, maximum absolute error, and repeatability range.

| Robot | Mode | Path | Runs | Mean X error | Mean Y error | Mean heading error | Repeatability | Status |
|---|---|---|---:|---:|---:|---:|---:|---|
| Small | A | Pending suite | 0 | N/A | N/A | N/A | N/A | Not run |
| Small | B | Pending suite | 0 | N/A | N/A | N/A | N/A | Not run |
| Small | C | Pending suite | 0 | N/A | N/A | N/A | N/A | Not run |
| Big | A | Pending suite | 0 | N/A | N/A | N/A | N/A | Not run |
| Big | B | Pending suite | 0 | N/A | N/A | N/A | N/A | Not run |
| Big | C | Pending suite | 0 | N/A | N/A | N/A | N/A | Not run |

Do not state that mode C improves localization unless its recorded error and
repeatability outperform both A and B.

## Assumptions that still require physical proof

- Configured tracking-wheel reversals produce the signs defined above.
- The nominal offsets describe the tracking center accurately.
- PROS IMU rotation is clockwise-positive on both installations.
- Wheel slip is acceptably represented by the starting process noise.
- Null LemLib odometry sensors prevent a competing pose writer in authorized
  mode; this must be observed, not inferred from compilation.
- A future GPS installation can be aligned to the same field frame.
- A three-state filter is sufficient; velocity and bias states are not justified
  without residual data showing a need.
