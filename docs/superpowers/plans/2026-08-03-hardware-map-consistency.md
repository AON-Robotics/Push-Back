# Hardware Map Consistency Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give legacy hardware construction and LemLib configuration one dependency-free source of drivetrain port values, and report existing tracking-direction disagreement without changing it.

**Architecture:** A small `aon::config` value module owns both robot hardware maps and a pure validation function. The legacy `Hardware` owner and `activeRobotConfig()` consume those values while retaining their existing construction, ownership, and selected-robot behavior.

**Tech Stack:** C++17, GCC 13.1 host tests, PROS CLI 3.5.6, Arm GNU Toolchain 14.3.1, GNU Make, Git.

## Global Constraints

- Preserve every current motor port, tracking port, reversal, calibration value, authorization flag, constructor signature, device lifetime, and initialization order.
- Report the big-robot right-tracking reversal mismatch as `HardwareMapIssue::RightTrackingReversalMismatch`; do not correct or normalize the stored values.
- Do not command or initialize hardware from the new map validator.
- Keep `USING_BIG_ROBOT false` in the committed tree.
- Do not change C++17, add dependencies, edit vendored sources, or cross the physical gates in `docs/CURRENT_HANDOFF.md`.
- Stage and commit only the files named in this plan.

---

### Task 1: Characterize and centralize the hardware map

**Files:**
- Create: `tests/hardware-map-test.cpp`
- Create: `include/aon/config/hardware-map.hpp`
- Create: `src/aon/config/hardware-map.cpp`
- Modify: `include/aon/config/robot-config.hpp`
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `src/aon/core/hardware.cpp`

**Interfaces:**
- Produces: `DrivePorts`, `TrackingPorts`, `LegacyTrackingPorts`, `RobotHardwareMap`, `HardwareMapIssue`, `smallRobotHardwareMap`, `bigRobotHardwareMap`, and `validateHardwareMap(const RobotHardwareMap&) noexcept` in namespace `aon::config`.
- Consumes: existing signed PROS port semantics and the unchanged `Hardware`, `Odometry`, and `RobotConfig` constructors.

- [ ] **Step 1: Write the failing public-behavior test**

Create `tests/hardware-map-test.cpp`:

```cpp
#include "aon/config/hardware-map.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

using aon::config::HardwareMapIssue;
using aon::config::RobotHardwareMap;

void smallRobotValuesRemainUnchanged() {
  constexpr std::array<std::int8_t, 4> expectedLeft{11, -12, 13, -14};
  constexpr std::array<std::int8_t, 4> expectedRight{1, -2, 3, -4};
  const RobotHardwareMap& map = aon::config::smallRobotHardwareMap;

  CHECK(map.drive.left == expectedLeft);
  CHECK(map.drive.right == expectedRight);
  CHECK(map.legacyTracking.left == 19);
  CHECK(map.legacyTracking.right == -18);
  CHECK(map.legacyTracking.back == 5);
  CHECK(map.legacyTracking.imu == 16);
  CHECK(map.lemlibTracking.left == 19);
  CHECK(map.lemlibTracking.right == 18);
  CHECK(map.lemlibTracking.back == 5);
  CHECK(map.lemlibTracking.imu == 16);
  CHECK(!map.lemlibTracking.leftReversed);
  CHECK(map.lemlibTracking.rightReversed);
  CHECK(!map.lemlibTracking.backReversed);
  CHECK(aon::config::validateHardwareMap(map) == HardwareMapIssue::None);
}

void bigRobotValuesAndKnownMismatchRemainVisible() {
  constexpr std::array<std::int8_t, 4> expectedLeft{12, -13, -18, 19};
  constexpr std::array<std::int8_t, 4> expectedRight{-1, 2, 3, -4};
  const RobotHardwareMap& map = aon::config::bigRobotHardwareMap;

  CHECK(map.drive.left == expectedLeft);
  CHECK(map.drive.right == expectedRight);
  CHECK(map.legacyTracking.left == 5);
  CHECK(map.legacyTracking.right == -6);
  CHECK(map.legacyTracking.back == 7);
  CHECK(map.legacyTracking.imu == 14);
  CHECK(map.lemlibTracking.left == 5);
  CHECK(map.lemlibTracking.right == -6);
  CHECK(map.lemlibTracking.back == 7);
  CHECK(map.lemlibTracking.imu == 14);
  CHECK(!map.lemlibTracking.leftReversed);
  CHECK(!map.lemlibTracking.rightReversed);
  CHECK(!map.lemlibTracking.backReversed);
  CHECK(aon::config::validateHardwareMap(map) ==
        HardwareMapIssue::RightTrackingReversalMismatch);
}

void validatorDistinguishesPortAndReversalFailures() {
  RobotHardwareMap wrongPort = aon::config::smallRobotHardwareMap;
  wrongPort.lemlibTracking.right = 17;
  CHECK(aon::config::validateHardwareMap(wrongPort) ==
        HardwareMapIssue::RightTrackingPortMismatch);

  RobotHardwareMap wrongReversal = aon::config::smallRobotHardwareMap;
  wrongReversal.lemlibTracking.rightReversed = false;
  CHECK(aon::config::validateHardwareMap(wrongReversal) ==
        HardwareMapIssue::RightTrackingReversalMismatch);
}

void validatorRejectsPortsOutsideTheV5Range() {
  RobotHardwareMap zeroPort = aon::config::smallRobotHardwareMap;
  zeroPort.lemlibTracking.left = 0;
  CHECK(aon::config::validateHardwareMap(zeroPort) ==
        HardwareMapIssue::InvalidPort);

  RobotHardwareMap highPort = aon::config::smallRobotHardwareMap;
  highPort.legacyTracking.back = -22;
  CHECK(aon::config::validateHardwareMap(highPort) ==
        HardwareMapIssue::InvalidPort);
}

}  // namespace

int main() {
  smallRobotValuesRemainUnchanged();
  bigRobotValuesAndKnownMismatchRemainVisible();
  validatorDistinguishesPortAndReversalFailures();
  validatorRejectsPortsOutsideTheV5Range();
  std::cout << "hardware map tests passed\n";
  return 0;
}
```

The production changes caught are wrong hardware literals, loss of the known
big-robot diagnostic, incorrect signed-port normalization, and failure to
reject invalid V5 ports.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```powershell
$compilerBin = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin'
$compiler = "$compilerBin\g++.exe"
$env:Path = "$compilerBin;$env:Path"
New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
& $compiler -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\hardware-map-test.cpp src\aon\config\hardware-map.cpp `
  -o bin\host-tests\hardware-map-test.exe
```

Expected: compilation fails because `aon/config/hardware-map.hpp` and
`src/aon/config/hardware-map.cpp` do not exist. This is the intended missing
production boundary, not a test syntax failure.

- [ ] **Step 3: Add the dependency-free map interface**

Create `include/aon/config/hardware-map.hpp` with Doxygen comments on every
public type, enum, configuration value, and function:

```cpp
#pragma once

#include <array>
#include <cstdint>

namespace aon::config {

struct DrivePorts {
  std::array<std::int8_t, 4> left;
  std::array<std::int8_t, 4> right;
};

struct TrackingPorts {
  std::int8_t left;
  std::int8_t right;
  std::int8_t back;
  std::int8_t imu;
  bool leftReversed;
  bool rightReversed;
  bool backReversed;
};

struct LegacyTrackingPorts {
  std::int8_t left;
  std::int8_t right;
  std::int8_t back;
  std::int8_t imu;
};

struct RobotHardwareMap {
  DrivePorts drive;
  LegacyTrackingPorts legacyTracking;
  TrackingPorts lemlibTracking;
};

enum class HardwareMapIssue : std::uint8_t {
  None,
  InvalidPort,
  LeftTrackingPortMismatch,
  RightTrackingPortMismatch,
  BackTrackingPortMismatch,
  ImuPortMismatch,
  LeftTrackingReversalMismatch,
  RightTrackingReversalMismatch,
  BackTrackingReversalMismatch,
};

inline constexpr RobotHardwareMap smallRobotHardwareMap{
    {{{11, -12, 13, -14}}, {{1, -2, 3, -4}}},
    {19, -18, 5, 16},
    {19, 18, 5, 16, false, true, false},
};

inline constexpr RobotHardwareMap bigRobotHardwareMap{
    {{{12, -13, -18, 19}}, {{-1, 2, 3, -4}}},
    {5, -6, 7, 14},
    {5, -6, 7, 14, false, false, false},
};

[[nodiscard]] HardwareMapIssue validateHardwareMap(
    const RobotHardwareMap& map) noexcept;

}  // namespace aon::config
```

Comments must state that signed ports select a physical V5 port by absolute
value, legacy tracking reversal comes from the sign, LemLib final reversal is
the explicit boolean applied after construction, the values are non-owning
data, and validation has no hardware side effects.

- [ ] **Step 4: Implement validation without hardware dependencies**

Create `src/aon/config/hardware-map.cpp`:

```cpp
#include "aon/config/hardware-map.hpp"

#include <array>

namespace aon::config {
namespace {

int physicalPort(std::int8_t port) noexcept {
  const int value = port;
  return value < 0 ? -value : value;
}

bool isValidPort(std::int8_t port) noexcept {
  const int value = physicalPort(port);
  return value >= 1 && value <= 21;
}

bool allPortsAreValid(const RobotHardwareMap& map) noexcept {
  for (const std::int8_t port : map.drive.left) {
    if (!isValidPort(port)) return false;
  }
  for (const std::int8_t port : map.drive.right) {
    if (!isValidPort(port)) return false;
  }
  const std::array<std::int8_t, 8> trackingPorts{
      map.legacyTracking.left, map.legacyTracking.right,
      map.legacyTracking.back, map.legacyTracking.imu,
      map.lemlibTracking.left, map.lemlibTracking.right,
      map.lemlibTracking.back, map.lemlibTracking.imu,
  };
  for (const std::int8_t port : trackingPorts) {
    if (!isValidPort(port)) return false;
  }
  return true;
}

}  // namespace

HardwareMapIssue validateHardwareMap(const RobotHardwareMap& map) noexcept {
  if (!allPortsAreValid(map)) return HardwareMapIssue::InvalidPort;

  if (physicalPort(map.legacyTracking.left) !=
      physicalPort(map.lemlibTracking.left)) {
    return HardwareMapIssue::LeftTrackingPortMismatch;
  }
  if (physicalPort(map.legacyTracking.right) !=
      physicalPort(map.lemlibTracking.right)) {
    return HardwareMapIssue::RightTrackingPortMismatch;
  }
  if (physicalPort(map.legacyTracking.back) !=
      physicalPort(map.lemlibTracking.back)) {
    return HardwareMapIssue::BackTrackingPortMismatch;
  }
  if (physicalPort(map.legacyTracking.imu) !=
      physicalPort(map.lemlibTracking.imu)) {
    return HardwareMapIssue::ImuPortMismatch;
  }
  if ((map.legacyTracking.left < 0) !=
      map.lemlibTracking.leftReversed) {
    return HardwareMapIssue::LeftTrackingReversalMismatch;
  }
  if ((map.legacyTracking.right < 0) !=
      map.lemlibTracking.rightReversed) {
    return HardwareMapIssue::RightTrackingReversalMismatch;
  }
  if ((map.legacyTracking.back < 0) !=
      map.lemlibTracking.backReversed) {
    return HardwareMapIssue::BackTrackingReversalMismatch;
  }
  return HardwareMapIssue::None;
}

}  // namespace aon::config
```

The explicit LemLib boolean is the final reversal state because `chassis.cpp`
invokes `set_reversed` after sensor construction. The function returns the
first issue and never mutates `map`.

- [ ] **Step 5: Move the existing public port types without renaming**

Modify `include/aon/config/robot-config.hpp` to include
`aon/config/hardware-map.hpp` and remove only the duplicate `DrivePorts` and
`TrackingPorts` definitions. Keep `RobotIdentity`, `FallbackConfig`,
`LemLibDriveConfig`, `RobotConfig`, and `activeRobotConfig()` unchanged.

- [ ] **Step 6: Make both consumers use the shared values**

In `src/aon/config/robot-config.cpp`, select `bigRobotHardwareMap` or
`smallRobotHardwareMap` in the existing preprocessor branches. Replace only
the drive and tracking aggregates with `hardwareMap.drive` and
`hardwareMap.lemlibTracking`.

In `src/aon/core/hardware.cpp`, include the new header and define an internal
`constexpr const RobotHardwareMap& selectedHardwareMap` using the existing
`USING_BIG_ROBOT` branch. Replace the legacy odometry tracking literals with
`selectedHardwareMap.legacyTracking` members. Expand each shared drive array
element into the existing drivetrain initializer lists. Do not alter any
other initializer, order, or numeric value.

- [ ] **Step 7: Compile and run the focused test to verify GREEN**

Run the Step 2 command, then:

```powershell
& .\bin\host-tests\hardware-map-test.exe
```

Expected: `hardware map tests passed`, exit code 0, and no compiler warning.

- [ ] **Step 8: Review the behavior-preservation diff**

Run:

```powershell
git diff --check
git diff -- include/aon/config/hardware-map.hpp `
  src/aon/config/hardware-map.cpp include/aon/config/robot-config.hpp `
  src/aon/config/robot-config.cpp src/aon/core/hardware.cpp `
  tests/hardware-map-test.cpp
```

Confirm every removed port literal reappears unchanged in the appropriate
shared map, no hardware initializer moved, and the known mismatch remains.

---

### Task 2: Verify and publish the implementation checkpoint

**Files:**
- Test: `tests/hardware-map-test.cpp` and all existing host tests
- Verify: both embedded robot configurations
- Modify temporarily and restore: `include/aon/constants.hpp`

**Interfaces:**
- Consumes: Task 1 shared map and validator.
- Produces: a clean `Testing` checkpoint synchronized with `origin/Testing`.

- [ ] **Step 1: Rebuild and run all nine host tests**

Run:

```powershell
$compilerBin = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin'
$compiler = "$compilerBin\g++.exe"
$flags = @('-std=c++17', '-Wall', '-Wextra', '-Werror', '-Iinclude')
$env:Path = "$compilerBin;$env:Path"
New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
$builds = @(
  @{ Name='hardware-map-test'; Sources=@('tests\hardware-map-test.cpp','src\aon\config\hardware-map.cpp') },
  @{ Name='vector-test'; Sources=@('tests\vector-test.cpp') },
  @{ Name='figure-eight-path-test'; Sources=@('tests\figure-eight-path-test.cpp') },
  @{ Name='motion-fallback-test'; Sources=@('tests\motion-fallback-test.cpp','src\aon\auton\motion-health.cpp','src\aon\auton\fallback-geometry.cpp') },
  @{ Name='red-six-block-path-test'; Sources=@('tests\red-six-block-path-test.cpp','src\aon\auton\hybrid-sequence.cpp') },
  @{ Name='jerryio-path-auton-test'; Sources=@('tests\jerryio-path-auton-test.cpp') },
  @{ Name='shadow-sd-directory-test'; Sources=@('tests\shadow-sd-directory-test.cpp') },
  @{ Name='shadow-sd-write-test'; Sources=@('tests\shadow-sd-write-test.cpp') },
  @{ Name='shadow-auton-test'; Sources=@('tests\shadow-auton-test.cpp','src\aon\shadow\recorder.cpp','src\aon\shadow\processor.cpp','src\aon\shadow\codec.cpp','src\aon\shadow\storage.cpp','src\aon\shadow\service-state.cpp','src\aon\shadow\player.cpp','src\aon\shadow\player-pros.cpp') }
)
foreach ($build in $builds) {
  & $compiler @flags @($build.Sources) -o "bin\host-tests\$($build.Name).exe"
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
foreach ($build in $builds) {
  & ".\bin\host-tests\$($build.Name).exe"
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
```

Expected: all nine pass messages and exit code 0.

- [ ] **Step 2: Clean-build the big-robot configuration**

Use `apply_patch` to change only:

```cpp
#define USING_BIG_ROBOT false
```

to `true`. Run:

```powershell
$pros = 'C:\Users\jojur\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-cli-windows\pros.exe'
$toolchain = 'C:\Users\jojur\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr'
$env:PROS_TOOLCHAIN = $toolchain
$env:Path = "$toolchain\bin;$env:Path"
& $pros build clean all
```

Expected: all sources compile, firmware links, and the only accepted warning is
the existing vendored `json.hpp` `std::is_pod` deprecation.

- [ ] **Step 3: Restore and clean-build the small-robot configuration**

Use `apply_patch` to restore `#define USING_BIG_ROBOT false`, then run the same
clean build. Follow it with `& $pros build` and require exit code 0 with
`Nothing to be done for 'quick'`.

- [ ] **Step 4: Verify final scope and configuration**

Run:

```powershell
rg -n '^#define USING_BIG_ROBOT' include\aon\constants.hpp
git diff --check
git status --short --branch
git diff --stat
```

Expected: `USING_BIG_ROBOT false`; only the six production/test files from
Task 1 plus this plan are modified or untracked; no generated binary is shown.

- [ ] **Step 5: Commit the implementation checkpoint**

```powershell
git add -- include/aon/config/hardware-map.hpp `
  src/aon/config/hardware-map.cpp include/aon/config/robot-config.hpp `
  src/aon/config/robot-config.cpp src/aon/core/hardware.cpp `
  tests/hardware-map-test.cpp `
  docs/superpowers/plans/2026-08-03-hardware-map-consistency.md
git diff --cached --check
git commit -m "Centralize robot hardware port maps"
```

- [ ] **Step 6: Push and verify synchronization**

```powershell
git push origin Testing
git status --short --branch
git rev-parse HEAD
git rev-parse origin/Testing
```

Expected: clean `Testing`, equal commit IDs, and no pull request because the
approved workflow publishes checkpoints directly to the integration branch.
