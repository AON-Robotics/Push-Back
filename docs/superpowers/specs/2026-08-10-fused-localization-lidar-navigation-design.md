# Fused Localization and LiDAR Navigation Design

**Date:** 2026-08-10

**Status:** Approved for implementation planning

## Purpose

Build a known-field autonomous navigation stack for the two AON VEX U robots.
The system combines tracking-wheel odometry and IMU data on the V5 Brain with
compact LiDAR observations and route proposals produced by a Raspberry Pi. It
does not implement unknown-environment SLAM. The field is known; LiDAR is used
to correct localization and detect temporary obstacles.

The V5 remains the drivetrain safety authority. Loss of the Pi or LiDAR must
not create unrestricted motor commands or invalidate emergency-stop behavior.

## Current Repository Findings

The active LemLib chassis uses three rotation sensors and one IMU. LemLib owns
the pose used by current LemLib autonomous actions. Native autonomous routines
start a separate legacy `aon::Odometry` task lazily, so the repository has two
independent pose systems.

The legacy odometry implementation reads only the left and right tracking
wheels even though it owns a back wheel. It uses a constant IMU confidence,
does not expose covariance, contains blocking GPS/reset behavior, and has
fragile heading and mutex handling. It must not become the foundation of the
new estimator.

GPS, vision, distance, and optical APIs are present. GPS and orbit vision are
constructed on port zero in the current hardware configuration and therefore
are not available localization sensors. The configured intake distance sensor
serves mechanism detection and does not have validated wall geometry.

The repository already has useful motion ownership, cancellation, health
monitoring, encoder fallback, structured results, route authorization gates,
logging, fixed-capacity Shadow recording, native host tests, and LemLib path
following. Those facilities must be reused or preserved. Existing physical
validation gates remain in force.

## Architectural Decision

Use a split V5/Pi architecture:

- The V5 samples tracking wheels and IMU, owns the authoritative fused state,
  validates Pi messages, follows accepted routes, and owns all motor safety.
- The Raspberry Pi drives the 2D LiDAR, filters scans, matches known field
  walls, extracts dynamic obstacles, and performs global route planning.
- The Pi sends compact observations, obstacle tracks, and routes. It does not
  stream raw point clouds into the V5 control loop.
- The V5 continues wheel/IMU localization when the Pi link is stale. The
  active navigation policy decides whether degraded confidence permits reduced
  speed or requires a stop.

This avoids putting point-cloud processing on the V5 and avoids making Linux
or a serial link part of the direct drivetrain command loop.

## Coordinate and Unit Contract

All new modules use inches, seconds, and radians internally. Field coordinates
follow the existing LemLib convention:

- positive Y is forward at heading zero;
- positive X is robot-right at heading zero;
- positive heading is clockwise;
- public display and legacy adapters may convert heading to degrees only at
  their boundaries.

Every transform names its source and destination frame. LiDAR mounting
calibration supplies a rigid robot-to-LiDAR transform containing X offset, Y
offset, and yaw. Field and robot dimensions are configuration values rather
than estimator constants.

## Localization State and Interfaces

The initial estimator state is `[x, y, theta, vx, vy, omega]`. Acceleration,
gyro bias, and wheel bias are excluded initially because the available sensors
do not yet demonstrate that these states are sufficiently observable. Recorded
data may justify adding them later.

The public estimate contains pose, world-frame velocity, a fixed 6-by-6
covariance matrix, capture time, and status flags. Confidence is reported as
standard deviations derived from covariance: position in inches, heading in
degrees at presentation boundaries, velocity uncertainty, and age of the most
recent accepted absolute correction. No arbitrary confidence percentage is
used.

Hardware-independent localization components consume:

- timestamped tracking-wheel positions and health flags;
- timestamped IMU heading/angular-rate data and health flags;
- standardized corrective observations with value, covariance, capture time,
  source, sequence number, and observation kind.

Corrective observation kinds initially cover field position, heading, full
pose, and wall-normal axis position. This interface permits later GPS, visual
landmark, or dedicated distance-sensor adapters without coupling them to EKF
internals.

## Odometry and Estimation Mathematics

Tracking-wheel deltas are converted into a body-frame SE(2) motion increment
using calibrated wheel diameters and offsets. Straight-line and small-angle
limits use numerically stable expressions. Heading is wrapped consistently,
and encoder baselines are reset atomically with pose resets.

An extended Kalman filter predicts pose and velocity using measured elapsed
time and the odometry increment. IMU observations correct angular state with
independent uncertainty. Process covariance grows with translation, rotation,
elapsed time, invalid samples, and detected slip.

Corrective measurements use normalized innovation squared gating with
dimension-appropriate chi-square thresholds. Non-finite, malformed, stale,
duplicate, physically impossible, or statistically inconsistent observations
are rejected with an explicit reason and telemetry record.

A fixed-capacity state history permits delayed Pi observations to be applied
at their capture time and propagated to the current estimate. History length,
message age, and replay work are bounded. Measurements older than retained
history are rejected rather than applied to the present as if they were new.

## Authoritative Pose Migration

The new `LocalizationManager` is the sole pose source for new navigation. Its
hardware adapter owns one fixed-rate task and publishes coherent estimate
snapshots without exposing mutable estimator state.

Existing LemLib and native routines remain unchanged while the new stack is
host-tested and physically gated. New obstacle-aware navigation uses an
injected fused-pose source. The project must not claim that LemLib's opaque
internal follower consumes fused localization when it does not.

Migration proceeds route by route. Existing pose consumers are redirected to
the localization facade only after equivalent behavior is tested. The legacy
odometry task is retired only after all remaining native consumers have moved.
No two tasks may independently reset shared sensor baselines.

## Static Field Model

The static model is separate from localization and contains fixed-capacity:

- field boundaries and wall segments;
- permanent restricted or collision regions;
- goals and scoring poses;
- named landmarks and autonomous waypoints.

Geometry is immutable during a match and never expires. Field data is selected
for the active game and checked for finite values, consistent winding, valid
bounds, and unique identifiers in host tests.

## Raspberry Pi and LiDAR Processing

The Pi-side program owns the model-specific LiDAR driver. Downstream processing
uses a model-independent scan interface. A scan pipeline:

1. validates range and angle samples;
2. transforms points through the calibrated LiDAR mounting transform;
3. compensates for pose and capture time where data permits;
4. matches line features against eligible known field walls;
5. produces a pose or wall-normal observation with covariance and residual;
6. removes matched static geometry;
7. clusters remaining points into bounded dynamic obstacle candidates.

Transparent or weakly reflective field material can make wall returns
unreliable. Wall observations therefore require minimum support, geometric
conditioning, bounded residual, and covariance derived from fit quality.
Failure to find a trustworthy wall produces no correction.

## Dynamic World Model

Dynamic obstacles are circles or axis-aligned/oriented rectangles with pose,
size, optional velocity, confidence, source, first/last observation time, and
expiry time. The tracker associates bounded observations, smooths motion, and
expires stale entries. Static geometry is never inserted into this expiring
collection.

The V5 stores only the bounded obstacle subset needed to validate and follow
the active route. The Pi may retain a larger working set within explicit
limits.

## Planning and Replanning

The Pi uses a hybrid visibility graph. Nodes consist of start, goal, selected
static vertices, and relevant inflated obstacle corners or tangent points.
Collision-free visible edges are searched with A*. Obstacles are inflated by
the robot footprint plus a configurable safety margin. Planning returns a
bounded waypoint route, cost, source-map revision, and validity horizon.

An occupancy grid is not used initially because the field geometry is known,
the workspace is small, and deterministic geometric planning needs less memory
and tuning.

Replanning occurs only when:

- the objective changes;
- a meaningful world-model revision intersects the remaining route corridor;
- localization correction invalidates the route start relationship;
- the follower reports blocked or unrecoverable progress.

Rate limits and change thresholds prevent continuous replanning. Failure to
find a route returns a structured result; it never falls through to a direct
unchecked drive command.

## Path Following

New planned routes use a V5-side follower with an injected fused-pose source.
The controller is a nonlinear differential-drive pose controller with bounded
linear/angular velocity, acceleration limiting, cross-track correction,
waypoint progression, final-pose tolerances, timeout, cancellation, and motion
ownership integration.

The existing LemLib follower remains for existing routes until measurements
show a reason to migrate them. The new follower is introduced because dynamic
routes require explicit fused-pose access and route replacement, not because a
different controller name is intrinsically better.

## V5/Pi Protocol

Communication uses a supported wired serial connection and framed binary
messages containing magic, protocol version, message type, payload length,
sequence number, capture timestamp, payload, and CRC. The parser has a fixed
maximum frame size, bounded buffers, resynchronization, and explicit errors.

Messages cover heartbeat/time synchronization, V5 pose snapshots, LiDAR
localization observations, dynamic obstacle batches, route requests, route
responses, route invalidation, and diagnostic counters. Pi messages cannot
issue raw motor or mechanism commands.

The V5 rejects unsupported versions, invalid CRCs, oversized payloads,
non-finite geometry, stale or repeated sequences, out-of-bounds routes, and
routes exceeding fixed capacities. Link health is based on heartbeat age and
accepted-message progress, not merely an open port.

## Recovery and Failure Policy

- Wheel slip increases process uncertainty and may reduce allowed speed.
- One conflicting absolute observation is rejected.
- Repeated independent, geometrically consistent LiDAR observations may
  recover a physically displaced robot.
- A large accepted correction pauses or cancels the follower before replacing
  its reference and requesting a route revalidation.
- Pi or LiDAR dropout degrades to wheel/IMU estimation.
- Loss of essential tracking or IMU data stops fused navigation.
- Poor confidence uses explicit policy thresholds for normal, reduced-speed,
  replan-required, and stop-required states.
- Emergency cancellation remains latched through existing motion ownership
  rules until explicitly rearmed by a safe autonomous dispatch.

## Telemetry

Compile-time and runtime controls govern logging. A fixed-size record includes
time, raw odometry increment/pose, fused state, covariance diagonal, sensor
health, accepted observation, residual, innovation score, rejection reason,
Pi link state, active route revision, follower error, and replan reason.

Competition logging is rate-limited and non-blocking. Detailed replay data can
be enabled during testing and exported for host simulation without placing
large automatic arrays on a V5 task stack.

## Test Strategy

Estimator and geometry code compile as standard C++17 without PROS hardware.
Tests use deterministic simulated or recorded inputs and cover:

- straight, lateral, curved, and in-place motion;
- tracking-wheel offsets, small-angle integration, and heading wraparound;
- variable update periods and timestamp anomalies;
- encoder and IMU noise, disconnects, and dropout;
- absolute correction, delayed correction, and covariance contraction;
- outlier rejection, wheel slip, physical displacement, and recovery;
- covariance growth and confidence-policy transitions;
- field geometry and wall-observation conditioning;
- dynamic obstacle association, motion, confidence, and expiry;
- planner clearance, unreachable goals, capacity limits, and determinism;
- meaningful route invalidation and replan rate limiting;
- follower convergence, cancellation, timeout, and route replacement;
- serial framing, corruption, truncation, resynchronization, version mismatch,
  replayed messages, heartbeat loss, and recovery.

Every behavior change follows a red-green-refactor cycle. Hardware-facing
phases clean-build both robot configurations. Physical validation is recorded
separately and remains mandatory before enabling competition execution.

## Delivery Boundaries

The program is implemented as four independently reviewable projects:

1. localization core and sensor-independent estimator;
2. known field model, observation gating, confidence, and recovery;
3. dynamic obstacle model, planner, replanning, and fused-pose follower;
4. Raspberry Pi protocol and model-independent LiDAR integration.

Each project leaves existing competition routes operational and keeps new
hardware behavior behind configuration and authorization gates. The LiDAR
model, exact serial port, mounting transform, and measured noise parameters are
deployment configuration. Their absence must not prevent host implementation
or safe V5-only operation.

## Explicit Non-Goals

- Unknown-environment occupancy-grid SLAM.
- Raw point-cloud processing on the V5.
- Pi-issued motor or mechanism commands.
- Enabling unvalidated competition routes.
- Rewriting working LemLib routines before fused navigation is proven.
- Adding estimator states or sensor fusion merely because a device API exists.
