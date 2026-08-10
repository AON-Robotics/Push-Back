# V5–Raspberry Pi Protocol, Version 1

The wire format is defined by `aon/communication/pi-protocol.hpp`. All integer
and IEEE-754 float fields are little-endian. Payloads are fixed-capacity and no
valid frame exceeds 146 bytes.

## Frame

| Offset | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | magic `0xA05A` (bytes `5A A0`) |
| 2 | 1 | protocol version `1` |
| 3 | 1 | message type |
| 4 | 2 | payload length, 0–128 |
| 6 | 4 | monotonically increasing sequence |
| 10 | 4 | capture timestamp in milliseconds |
| 14 | N | payload |
| 14+N | 4 | CRC-32 |

CRC-32 uses polynomial `0xEDB88320`, initial value `0xFFFFFFFF`, and final
bitwise inversion. It covers the version byte through the final payload byte;
the two magic bytes and CRC field are excluded.

Receivers reject unsupported versions, unknown types, oversized payloads, CRC
errors, duplicates, and out-of-order sequences. The V5 must stop accepting Pi
navigation proposals whenever link health becomes stale. Parser recovery scans
for the next magic pair; a malformed frame never changes pose or drive output.

## Message types

| Value | Name | Direction | Status |
| ---: | --- | --- | --- |
| 1 | Heartbeat | both | framed |
| 2 | PoseSnapshot | V5 → Pi | typed codec |
| 3 | LocalizationObservation | Pi → V5 | typed wall-axis codec |
| 4 | ObstacleBatch | Pi → V5 | typed codec |
| 5 | RouteRequest | V5 → Pi | reserved |
| 6 | RouteResponse | Pi → V5 | typed chunk codec |
| 7 | RouteInvalidation | both | reserved |
| 8 | Diagnostics | both | reserved |

Reserved messages may be framed but must not affect runtime behavior until a
typed payload contract and tests exist.

## Typed payloads

`PoseSnapshot` is 24 bytes: X, Y, heading, position variance, and heading
variance as five 32-bit floats, followed by the 32-bit estimate timestamp.

`LocalizationObservation` is 15 bytes: axis (`0` X, `1` Y), axis position and
variance as floats, 16-bit fit support, and 32-bit capture timestamp. Variance
must be positive and support nonzero. The V5 also applies age, duplicate, and
normalized-innovation gates before changing the EKF.

`ObstacleBatch` starts with an 8-bit count (0–4). Each 29-byte obstacle contains
shape, X, Y, radius, half width, half height, heading, and confidence. Geometry
uses inches/radians in field coordinates; confidence is in `[0,1]`. Circles
require positive radius. Rectangles require positive half extents.

`RouteResponse` begins with route ID (32-bit), chunk index, chunk count, and
point count. Up to 14 X/Y float pairs follow. Route IDs are nonzero, chunks are
zero-indexed, and `chunkIndex < chunkCount`. The V5 must assemble a complete,
ordered route and independently collision-check it before following it.

## Timing and ownership

- Capture timestamps describe when sensor data was acquired, not when a frame
  was transmitted or received.
- Synchronize clocks at connection and monitor offset and drift. If the offset
  is uncertain, discard delayed corrections rather than applying them now.
- Use the V5 pose history to interpolate the capture-time pose for scan
  processing and observation validation.
- Heartbeats refresh link health only when their sequence advances.
- Stale link, unsafe localization confidence, cancellation, timeout, or invalid
  route always produces a V5-side stop.
- The Pi performs scan processing and may plan; the V5 owns fused pose, final
  collision checks, speed limits, drive output, and emergency stop.

Run `powershell -ExecutionPolicy Bypass -File tools/run-host-tests.ps1` after
every protocol change. Any wire-format change requires a new protocol version,
new golden-byte tests on both ends, and a documented migration.
