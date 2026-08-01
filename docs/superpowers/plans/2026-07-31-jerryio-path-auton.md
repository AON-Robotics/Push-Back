# JerryIO Path Autonomous Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a drive-only LemLib autonomous that follows `static/path.jerryio.txt` from its generated starting pose and replaces the figure-eight entry in Blue AUT3.

**Architecture:** A dependency-free `JerryIoPathAuton` configuration owns the route constants shared by robot code and a host test. The existing `Actions::followPath` adapter executes the embedded JerryIO asset with fail-closed odometry monitoring, while the existing selector wrapper supplies cancellation and status handling.

**Tech Stack:** C++17, PROS 4.2.2, LemLib 0.5.6, JerryIO/LemLib v0.5 path asset, MinGW host tests, GNU Make/ARM toolchain.

## Global Constraints

- Register the routine as Blue AUT3 with the exact name `TEST JerryIO Path`.
- Operate no intake, loader, scorer, or other mechanism.
- Preserve `static/path.jerryio.txt` exactly as supplied by the user.
- Start at `(-66.557, -35.024, 132.06 degrees)` and follow forward.
- Use a 10-inch lookahead, 14,000 ms timeout, and fail-closed odometry monitoring.
- Stop with `pros::E_MOTOR_BRAKE_BRAKE` on success, failure, or unsupported big-robot execution.
- Do not change Red AUT3, Skills autons, fallback behavior, or Shadow playback.

---

## File Structure

- Create `include/aon/auton/jerryio-path-auton.hpp`: dependency-free routine name, pose, and motion constants.
- Create `tests/jerryio-path-auton-test.cpp`: validate the asset contract, runtime wiring, and Blue AUT3 registration.
- Modify `include/aon/auton/routines.hpp`: replace the generic named path-test declaration with the dedicated routine declaration.
- Modify `src/aon/auton/lemlib-routines.cpp`: execute the embedded asset with the approved pose and safety behavior.
- Modify `src/aon/auton/routine-selectors.cpp`: select the dedicated routine from Blue AUT3.
- Modify `include/aon/tools/gui/gui.hpp`: show the dedicated routine name in Blue AUT3.

---

### Task 1: Lock Down the JerryIO Asset Contract

**Files:**
- Create: `tests/jerryio-path-auton-test.cpp`
- Create: `include/aon/auton/jerryio-path-auton.hpp`
- Verify without modifying: `static/path.jerryio.txt`

**Interfaces:**
- Consumes: LemLib v0.5 path rows in `static/path.jerryio.txt`.
- Produces: `aon::auton::JerryIoPathAuton` with `name`, `startX`, `startY`, `startHeading`, `lookahead`, `timeoutMs`, and `maximumPathSpeed` constants.

- [x] **Step 1: Read the test-quality rules**

Read `superpowers:test-driven-development/writing-good-tests.md` before writing the test. The production mutation that must fail is changing any configured start coordinate, heading, lookahead, timeout, or maximum speed independently of the path asset.

- [x] **Step 2: Write the failing host test**

Create `tests/jerryio-path-auton-test.cpp` with a parser that reads numeric rows until `endData`. Assert:

```cpp
using aon::auton::JerryIoPathAuton;
CHECK(std::string(JerryIoPathAuton::name) == "TEST JerryIO Path");
CHECK(JerryIoPathAuton::startX == -66.557);
CHECK(JerryIoPathAuton::startY == -35.024);
CHECK(JerryIoPathAuton::startHeading == 132.06);
CHECK(JerryIoPathAuton::lookahead == 10.0F);
CHECK(JerryIoPathAuton::timeoutMs == 14000);
CHECK(JerryIoPathAuton::maximumPathSpeed == 127);
```

After parsing, require at least 80 points, finite coordinates and speeds, a valid terminator, a first point equal to the configured X/Y, all speeds in `[0, maximumPathSpeed]`, and final speed equal to zero. Compute the initial LemLib heading with `atan2(second.x - first.x, second.y - first.y)` in degrees and require its error from `startHeading` to be at most `0.1` degree.

- [x] **Step 3: Compile and verify RED**

Run:

```powershell
$compiler = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe'
$env:Path = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin;' + $env:Path
New-Item -ItemType Directory -Force 'bin\host-tests' | Out-Null
& $compiler -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\jerryio-path-auton-test.cpp `
  -o bin\host-tests\jerryio-path-auton-test.exe
```

Expected: compilation fails because `aon/auton/jerryio-path-auton.hpp` does not exist.

- [x] **Step 4: Add the minimal route configuration**

Create `include/aon/auton/jerryio-path-auton.hpp`:

```cpp
#pragma once

namespace aon::auton {

struct JerryIoPathAuton {
  static constexpr const char* name = "TEST JerryIO Path";
  static constexpr double startX = -66.557;
  static constexpr double startY = -35.024;
  static constexpr double startHeading = 132.06;
  static constexpr float lookahead = 10.0F;
  static constexpr int timeoutMs = 14000;
  static constexpr int maximumPathSpeed = 127;
};

}  // namespace aon::auton
```

- [x] **Step 5: Compile and run the focused test to verify GREEN**

Repeat the Step 3 compile command, then run:

```powershell
& '.\bin\host-tests\jerryio-path-auton-test.exe'
```

Expected: `JerryIO path auton tests passed`.

- [x] **Step 6: Commit the validated path boundary**

```powershell
git add -- 'include/aon/auton/jerryio-path-auton.hpp' `
  'tests/jerryio-path-auton-test.cpp'
git commit -m "Validate JerryIO autonomous path"
```

Do not stage `static/path.jerryio.txt`; it remains the user's pre-existing modification until the final feature checkpoint.

---

### Task 2: Integrate the Drive-Only Routine into Blue AUT3

**Files:**
- Modify: `tests/jerryio-path-auton-test.cpp`
- Modify: `include/aon/auton/routines.hpp`
- Modify: `src/aon/auton/lemlib-routines.cpp`
- Modify: `src/aon/auton/routine-selectors.cpp`
- Modify: `include/aon/tools/gui/gui.hpp`

**Interfaces:**
- Consumes: `JerryIoPathAuton`, `path_jerryio_txt`, and `Actions::followPath(const char*, const asset&, float, int, bool, const std::function<void()>&, OdometryMonitoring)`.
- Produces: `int RunJerryIoPathAuton()` and Blue AUT3 GUI/selector registration.

- [x] **Step 1: Extend the host test before production wiring**

Add `#include "aon/auton/routines.hpp"` and a compile-time signature assertion:

```cpp
static_assert(std::is_same_v<
    decltype(&aon::routines::RunJerryIoPathAuton), int (*)()>);
```

Do not add source-text assertions. The host test exercises the real path
contract; the embedded build and final diff review verify selector and runtime
wiring without coupling a test to implementation spelling.

- [x] **Step 2: Compile and verify RED**

Repeat Task 1 Step 3.

Expected: compilation fails because `RunJerryIoPathAuton` is not declared.

- [x] **Step 3: Declare and implement the dedicated routine**

Replace `int RunJerryIoPathTest(const char* name);` in `include/aon/auton/routines.hpp` with:

```cpp
int RunJerryIoPathAuton();
```

In `src/aon/auton/lemlib-routines.cpp`, include the configuration and replace the generic `RunJerryIoPathTest` function with:

```cpp
int RunJerryIoPathAuton() {
  using aon::auton::JerryIoPathAuton;
  auto& routine = aon::auton::actions();

#if USING_BIG_ROBOT
  aon::auton::logStep(JerryIoPathAuton::name, "unsupported big robot");
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  return 0;
#else
  aon::auton::logStep(JerryIoPathAuton::name, "start");
  routine.setPose(JerryIoPathAuton::startX, JerryIoPathAuton::startY,
                  JerryIoPathAuton::startHeading);
  const auto result = routine.followPath(
      JerryIoPathAuton::name, path_jerryio_txt,
      JerryIoPathAuton::lookahead, JerryIoPathAuton::timeoutMs, true, {},
      aon::auton::OdometryMonitoring::FailClosed);
  routine.stop(pros::E_MOTOR_BRAKE_BRAKE);
  aon::auton::logStep(JerryIoPathAuton::name,
                      result.succeeded ? "finish" : "failed");
  return result.succeeded ? 1 : 0;
#endif
}
```

- [x] **Step 4: Register Blue AUT3 and its GUI label**

Include `aon/auton/jerryio-path-auton.hpp` in the selector and GUI header. Change only Blue AUT3:

```cpp
int BlueRoutine3() {
  return runRoutine(aon::auton::JerryIoPathAuton::name,
                    RunJerryIoPathAuton);
}
```

```cpp
{aon::auton::JerryIoPathAuton::name, aon::routines::BlueRoutine3},
```

- [x] **Step 5: Run the focused test to verify GREEN**

Repeat Task 1 Step 3 and Step 5.

Expected: `JerryIO path auton tests passed` with no warnings.

- [x] **Step 6: Run the complete host suite**

Rebuild and run `vector-test`, `figure-eight-path-test`, `motion-fallback-test`, `red-six-block-path-test`, `shadow-sd-directory-test`, `shadow-sd-write-test`, and `shadow-auton-test` using the source lists in `docs/superpowers/plans/2026-07-31-vector-value-semantics.md`, plus the new focused test.

Expected: all eight executables print their pass messages and return exit code 0.

- [x] **Step 7: Run a clean embedded build**

```powershell
$toolchain = 'C:\Users\jojur\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr'
$env:PROS_TOOLCHAIN = $toolchain
$env:Path = "$toolchain\bin;$env:Path"
make clean
make
```

Expected: a fresh `bin/monolith.bin` with no new compiler errors or warnings.

- [x] **Step 8: Review and commit the complete implementation**

Run `git diff --check`, review the exact scoped diff, and use the code-review skill. Fix all Critical or Important findings and rerun affected verification. Then commit the feature, including the user-supplied path now that its contract and use are complete:

```powershell
git add -- 'static/path.jerryio.txt' `
  'include/aon/auton/jerryio-path-auton.hpp' `
  'include/aon/auton/routines.hpp' `
  'include/aon/tools/gui/gui.hpp' `
  'src/aon/auton/lemlib-routines.cpp' `
  'src/aon/auton/routine-selectors.cpp' `
  'tests/jerryio-path-auton-test.cpp' `
  'docs/superpowers/plans/2026-07-31-jerryio-path-auton.md'
git diff --cached --check
git commit -m "Add JerryIO path autonomous"
```

Expected final working tree: clean, with Red AUT3 unchanged and Blue AUT3 labeled `TEST JerryIO Path`.
