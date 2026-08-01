# Vector Value Semantics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `aon::Vector` an allocation-free value type whose copies have independent direction state while preserving its public interface and numerical behavior.

**Architecture:** Keep `Angle` and `Vector` as header-only math value types. Replace only `Vector`'s owning raw pointer with an inline `Angle`, and verify the change through the public math interface with a dependency-free host test.

**Tech Stack:** C++17 host tests with GCC 13.1; embedded PROS 4.2.2 build with Arm GNU 14.3.1 and `gnu++26`.

## Global Constraints

- Preserve all existing public `Angle` and `Vector` method names, parameter types, return types, conversions, and operators.
- Preserve degree/radian conversions, signed input behavior, Cartesian components, and arithmetic results.
- `SetDirection(Angle*)` requires a non-null, non-owning pointer and copies its value.
- Do not change odometry formulas, drivetrain commands, timing, units, hardware configuration, or task synchronization.
- Do not add dependencies or perform unrelated formatting and naming cleanup.
- Treat each completed task as a reviewable checkpoint; commit and push `Testing` after full verification.

---

### Task 1: Prove independent copy semantics and remove raw ownership

**Files:**
- Create: `tests/vector-test.cpp`
- Modify: `include/aon/tools/vector.hpp:350`

**Interfaces:**
- Consumes: Existing `aon::Angle` and `aon::Vector` public methods from `include/aon/tools/vector.hpp`.
- Produces: The same public `aon::Vector` interface with inline `Angle` ownership and independent compiler-generated copy/move behavior.

- [ ] **Step 1: Write the public-behavior host test**

Create `tests/vector-test.cpp` with literal, hand-derived expectations. The
production mutation caught by `copyMutationDoesNotChangeOriginalDirection` is
restoring pointer aliasing between copies.

```cpp
#include "aon/tools/vector.hpp"

#include <cmath>
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

constexpr double kTolerance = 1e-9;

void checkNear(double actual, double expected,
               double tolerance = kTolerance) {
  CHECK(std::abs(actual - expected) <= tolerance);
}

void defaultVectorIsZero() {
  aon::Vector vector;
  checkNear(vector.GetX(), 0.0);
  checkNear(vector.GetY(), 0.0);
  checkNear(vector.GetMagnitude(), 0.0);
  checkNear(vector.GetDegrees(), 0.0);
  checkNear(vector.GetRadians(), 0.0);
}

void cartesianSettersPreserveCurrentConversions() {
  aon::Vector vector = aon::Vector().SetPosition(3.0, 4.0);
  checkNear(vector.GetX(), 3.0);
  checkNear(vector.GetY(), 4.0);
  checkNear(vector.GetMagnitude(), 5.0);
  checkNear(vector.GetDegrees(), 53.13010235415598);

  vector.SetX(0.0);
  checkNear(vector.GetMagnitude(), 4.0);
  checkNear(vector.GetDegrees(), 90.0);

  vector.SetY(-4.0);
  checkNear(vector.GetMagnitude(), 4.0);
  checkNear(vector.GetDegrees(), -90.0);
}

void polarSettersPreserveCurrentConversions() {
  aon::Vector vector = aon::Vector().SetMagnitude(5.0).SetDegrees(90.0);
  checkNear(vector.GetX(), 0.0);
  checkNear(vector.GetY(), 5.0);
  checkNear(vector.GetMagnitude(), 5.0);
  checkNear(vector.GetRadians(), M_PI / 2.0);

  vector.SetRadians(M_PI);
  checkNear(vector.GetX(), -5.0);
  checkNear(vector.GetY(), 0.0);
  checkNear(vector.GetDegrees(), 180.0);

  aon::Vector negative = aon::Vector().SetMagnitude(-2.0);
  checkNear(negative.GetMagnitude(), 2.0);
  checkNear(negative.GetX(), -2.0);
  checkNear(negative.GetY(), 0.0);
}

void extendedAnglesRemainUnwrapped() {
  aon::Vector vector = aon::Vector().SetMagnitude(1.0).SetDegrees(450.0);
  checkNear(vector.GetDegrees(), 450.0);
  checkNear(vector.GetRadians(), 5.0 * M_PI / 2.0);
  checkNear(vector.GetX(), 0.0);
  checkNear(vector.GetY(), 1.0);

  vector.SetRadians(-3.0 * M_PI);
  checkNear(vector.GetRadians(), -3.0 * M_PI);
  checkNear(vector.GetDegrees(), -540.0);
  checkNear(vector.GetX(), -1.0);
  checkNear(vector.GetY(), 0.0);
}

void setDirectionCopiesRequiredNonOwningInput() {
  aon::Angle angle = aon::Angle().SetDegrees(30.0);
  aon::Vector vector = aon::Vector().SetMagnitude(2.0).SetDirection(&angle);
  angle.SetDegrees(60.0);

  checkNear(vector.GetDegrees(), 30.0);
  checkNear(vector.GetX(), std::sqrt(3.0));
  checkNear(vector.GetY(), 1.0);
}

void copyMutationDoesNotChangeOriginalDirection() {
  aon::Vector original = aon::Vector().SetPosition(3.0, 4.0);
  aon::Vector copy = original;
  copy.SetPosition(0.0, 5.0);

  checkNear(original.GetX(), 3.0);
  checkNear(original.GetY(), 4.0);
  checkNear(original.GetMagnitude(), 5.0);
  checkNear(original.GetDegrees(), 53.13010235415598);
  checkNear(copy.GetDegrees(), 90.0);

  aon::Vector assigned;
  assigned = original;
  assigned.SetPosition(-5.0, 0.0);
  checkNear(original.GetDegrees(), 53.13010235415598);
  checkNear(assigned.GetDegrees(), 180.0);
}

void arithmeticRetainsExistingMeanings() {
  aon::Vector first = aon::Vector().SetPosition(3.0, 4.0);
  aon::Vector second = aon::Vector().SetPosition(5.0, 12.0);

  aon::Vector sum = first + second;
  checkNear(sum.GetX(), 8.0);
  checkNear(sum.GetY(), 16.0);

  aon::Vector difference = second - first;
  checkNear(difference.GetX(), 2.0);
  checkNear(difference.GetY(), 8.0);

  checkNear(first.Dot(second), 63.0);

  aon::Vector normalized = first.Normalize();
  checkNear(normalized.GetMagnitude(), 1.0);
  checkNear(normalized.GetDegrees(), 53.13010235415598);

  aon::Vector scaled = first * 2.0;
  checkNear(scaled.GetMagnitude(), 10.0);
  checkNear(scaled.GetDegrees(), 53.13010235415598);

  aon::Vector divided = second / first;
  checkNear(divided.GetMagnitude(), 2.6);
  checkNear(divided.GetDegrees(),
            67.38013505195957 - 53.13010235415598);
}

}  // namespace

int main() {
  defaultVectorIsZero();
  cartesianSettersPreserveCurrentConversions();
  polarSettersPreserveCurrentConversions();
  extendedAnglesRemainUnwrapped();
  setDirectionCopiesRequiredNonOwningInput();
  copyMutationDoesNotChangeOriginalDirection();
  arithmeticRetainsExistingMeanings();
  std::cout << "vector tests passed\n";
  return 0;
}
```

- [ ] **Step 2: Compile and run the test to verify the RED state**

Run:

```powershell
$compiler = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe'
& $compiler -std=c++17 -Wall -Wextra -Werror -Iinclude `
  tests\vector-test.cpp -o bin\host-tests\vector-test.exe
$env:Path = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin;' + $env:Path
& .\bin\host-tests\vector-test.exe
```

Expected: compilation succeeds, then the executable fails in
`copyMutationDoesNotChangeOriginalDirection` because changing the copied
vector through `SetPosition` also changes the original's shared direction.

- [ ] **Step 3: Replace pointer ownership with an inline value**

In `Vector`, document value semantics and replace the private member:

```cpp
/**
 * @brief Two-dimensional vector with Cartesian and polar representations.
 * @details Vector is a value type. Copies own independent direction state and
 * mutation performs no dynamic allocation.
 */
class Vector {
 private:
  double x = 0;
  double y = 0;
  double magnitude = 0;
  Angle direction{};
```

Update `SetX`, `SetY`, and `SetPosition` to call
`direction.SetRadians(...)`. Update `SetMagnitude`, getters, `Normalize`, and
string conversion to use `direction.` or `direction` rather than pointer
syntax.

Keep the `SetDirection` signature and copy behavior:

```cpp
/**
 * @brief Copies a required direction and updates Cartesian components.
 * @param direction Required non-owning pointer; the pointed-to value is copied.
 * @return This vector after the direction change.
 * @pre `direction` is not null.
 */
Vector SetDirection(Angle* direction) {
  this->direction.SetDegrees(direction->GetDegrees());
  x = magnitude * std::cos(this->direction.GetRadians());
  y = magnitude * std::sin(this->direction.GetRadians());
  return *this;
}
```

Preserve the existing degree/radian conversion path without heap allocation:

```cpp
Vector SetDegrees(double degrees) {
  Angle angle;
  angle.SetDegrees(degrees);
  return SetDirection(&angle);
}

Vector SetRadians(double radians) {
  Angle angle;
  angle.SetRadians(radians);
  return SetDirection(&angle);
}
```

- [ ] **Step 4: Run the Vector test to verify the GREEN state**

Repeat the Step 2 compile and run commands.

Expected: `vector tests passed`, exit code 0, and no compiler warnings.

- [ ] **Step 5: Perform the focused mutation review**

The test suite must catch each conceptual regression during review:

- replacing `Angle direction{}` with shared pointer storage;
- making `SetDirection` retain the caller's pointer;
- omitting the Cartesian recomputation in `SetDirection`;
- changing `SetMagnitude(-2.0)` to store `-2.0` rather than `2.0`.

No source mutation needs to remain after this review.

Run the ownership scan after restoring the implementation:

```powershell
rg -n 'direction->|new Angle|std::string\(\*direction\)' `
  include\aon\tools\vector.hpp
```

Expected: no matches in `Vector`.

---

### Task 2: Verify and publish the implementation checkpoint

**Files:**
- Test: `tests/vector-test.cpp`
- Verify: all files compiled by the existing host and PROS build commands

**Interfaces:**
- Consumes: The allocation-free `aon::Vector` implementation from Task 1.
- Produces: A verified `Testing` branch checkpoint synchronized with `origin/Testing`.

- [ ] **Step 1: Rebuild and run every host test with warnings as errors**

Run:

```powershell
$compiler = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin\g++.exe'
$flags = @('-std=c++17', '-Wall', '-Wextra', '-Werror', '-Iinclude')
& $compiler @flags tests\vector-test.cpp `
  -o bin\host-tests\vector-test.exe
& $compiler @flags tests\figure-eight-path-test.cpp `
  -o bin\host-tests\figure-eight-path-test.exe
& $compiler @flags tests\motion-fallback-test.cpp `
  src\aon\auton\motion-health.cpp src\aon\auton\fallback-geometry.cpp `
  -o bin\host-tests\motion-fallback-test.exe
& $compiler @flags tests\shadow-sd-directory-test.cpp `
  -o bin\host-tests\shadow-sd-directory-test.exe
& $compiler @flags tests\shadow-sd-write-test.cpp `
  -o bin\host-tests\shadow-sd-write-test.exe
& $compiler @flags tests\shadow-auton-test.cpp `
  src\aon\shadow\recorder.cpp src\aon\shadow\processor.cpp `
  src\aon\shadow\codec.cpp src\aon\shadow\storage.cpp `
  src\aon\shadow\service-state.cpp src\aon\shadow\player.cpp `
  src\aon\shadow\player-pros.cpp `
  -o bin\host-tests\shadow-auton-test.exe
$env:Path = 'C:\Users\jojur\AppData\Local\Programs\CLion\bin\mingw\bin;' + $env:Path
& .\bin\host-tests\vector-test.exe
& .\bin\host-tests\figure-eight-path-test.exe
& .\bin\host-tests\motion-fallback-test.exe
& .\bin\host-tests\shadow-sd-directory-test.exe
& .\bin\host-tests\shadow-sd-write-test.exe
& .\bin\host-tests\shadow-auton-test.exe
```

Expected: all six executables print their pass message and return exit code 0.

- [ ] **Step 2: Run a clean embedded build**

Run:

```powershell
$pros = 'C:\Users\jojur\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-cli-windows\pros.exe'
$toolchain = 'C:\Users\jojur\AppData\Roaming\Code\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr'
$env:PROS_TOOLCHAIN = $toolchain
$env:Path = "$toolchain\bin;$env:Path"
& $pros build clean all
& $pros build
```

Expected: fresh `bin/monolith.elf` and `bin/monolith.bin` are produced, and the
follow-up build reports `Nothing to be done for 'quick'` with exit code 0.

- [ ] **Step 3: Review the final diff and repository state**

Run:

```powershell
git diff --check
git diff -- include/aon/tools/vector.hpp tests/vector-test.cpp
git status -sb
```

Expected: only the focused Vector implementation, test, and this plan are in
scope; no generated binaries are staged.

- [ ] **Step 4: Commit the implementation checkpoint**

Run:

```powershell
git add -- include/aon/tools/vector.hpp tests/vector-test.cpp `
  docs/superpowers/plans/2026-07-31-vector-value-semantics.md
git diff --cached --check
git commit -m "Give Vector deterministic value semantics"
```

- [ ] **Step 5: Push and verify synchronization**

Run:

```powershell
git push origin Testing
git status -sb
git rev-parse HEAD
git rev-parse origin/Testing
```

Expected: `Testing` has no working-tree changes, and both revisions are equal.
