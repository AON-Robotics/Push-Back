# LiDAR Mounting and Calibration

The LiDAR path is intentionally disabled for competition use until this
checklist has measured values. Host tests prove software behavior; they do not
prove the sensor, mounting, field materials, or robot dynamics.

## Hardware record

Complete this table before writing a model-specific Raspberry Pi driver.

| Item | Measured value |
| --- | --- |
| LiDAR manufacturer and exact model | Not selected |
| Electrical interface and logic levels | Not measured |
| Supply voltage, peak current, and regulator | Not measured |
| Scan rate and samples per revolution | Not measured |
| Reliable minimum and maximum range | Not measured |
| Raspberry Pi model and operating system | Not selected |
| V5-to-Pi physical link | Not selected |
| Driver/library and pinned version | Not selected |
| Current VEX competition manual revision reviewed | Not recorded |

Do not power the LiDAR from an unverified V5 or Pi rail. Use the sensor data
sheet to size a regulated supply, share ground only where the chosen interfaces
require it, and verify voltage levels before connecting a raw UART. Prefer a
supported isolated or USB link when practical.

## Rigid mounting

Mount the scanner above mechanisms and game objects that otherwise occlude the
horizontal scan plane. The mount must not flex under acceleration or contact.
Protect the optics without placing transparent material in the scan plane;
clear covers can refract or suppress measurements.

The software convention is:

- inches and radians;
- robot positive X points right;
- robot positive Y points forward;
- positive yaw is clockwise;
- `xRightInches` and `yForwardInches` locate the LiDAR origin from the robot
  tracking center;
- `yawRadians` rotates the LiDAR forward ray from robot forward.

Record at least three independent measurements of X, Y, and yaw, then enter the
mean. Re-measure after every mount change or collision. Never tune the transform
until bad scans appear to fit; measure it physically first.

## Calibration sequence

1. Run `powershell -ExecutionPolicy Bypass -File tools/run-host-tests.ps1`.
2. Confirm tracking-wheel offsets, wheel diameter, and IMU direction using the
   existing localization calibration workflow.
3. Place the stationary robot at five surveyed poses: field center and near
   each wall. Capture raw polar scans and fused pose snapshots.
4. Check range scale against at least three tape-measured distances spanning
   the intended operating range.
5. Fit the robot-to-LiDAR X/Y/yaw transform from the stationary scans. Keep a
   separate validation set; do not validate on the scans used for fitting.
6. Test every relevant field-wall material and incidence angle. Record dropout,
   bias, and standard deviation. Shallow-angle or low-support wall observations
   must remain rejected.
7. Measure Pi-to-V5 latency, jitter, dropped frames, reconnect behavior, and
   timestamp offset while the robot is moving. Use capture timestamps with the
   fixed pose history; never label receipt time as capture time.
8. Tune covariance from repeated residuals. A correction variance must describe
   observed error, not the desired trust level.
9. Run push, wheel-slip, and displacement tests. A single large correction must
   remain blocked; recovery requires repeated consistent observations.
10. Run obstacle insertion, expiry, route blocking, replanning, cancellation,
    stale-link, and emergency-stop tests at reduced speed.

## Activation gates

Keep fused LiDAR navigation unauthorized until all of these pass on both robot
configurations:

- exact hardware record and current competition legality review;
- rigid mount and measured transform;
- stationary and moving scan datasets retained with results;
- serial framing, CRC, sequence, stale-link, and reconnect tests;
- covariance and confidence thresholds supported by measured distributions;
- route-by-route validation with mechanisms installed;
- driver emergency stop always overrides Pi output;
- V5 remains the only motor and mechanism authority;
- team captain signs the physical test record.

The Pi may propose observations, obstacles, and routes. It must never directly
command drive or mechanism motors.
