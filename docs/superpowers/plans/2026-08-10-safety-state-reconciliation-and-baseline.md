# Safety-State Reconciliation and Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every enabled autonomous authorization traceable to recorded physical evidence, restore fail-closed defaults where evidence is absent, and publish one accurate baseline handoff before roadmap code begins.

**Architecture:** A dependency-free policy test inventories configuration gates and registered routes for both compile-time robot variants. Documentation records the evidence chain; configuration remains locked unless a completed physical checklist names the exact tested commit. This phase changes no controller constants, route geometry, device ownership, or mechanism behavior.

**Tech Stack:** C++17 host tests, PowerShell structural tests, PROS 4.2.2, GNU Make, Git, Markdown.

## Global Constraints

- Treat Git history and committed physical checklists as evidence; do not infer authorization from compilation or host tests.
- Preserve Controller-X cancellation, drivetrain ownership, native fallbacks, and all automatic encoder-fallback gates.
- Keep `automaticFallbackAuthorized` and `forcedEncoderTestingAuthorized` false for both robots.
- Keep experimental Red Six Block and JerryIO route authorizations false until their individual checklists pass.
- Keep big-robot Shadow playback false.
- Because `docs/testing/2026-07-30-shadow-playback-checklist.md` says every physical playback gate is `Not run`, restore small-robot Shadow playback to false before the baseline commit.
- Do not alter route geometry, mechanism commands, motor ports, reversals, gains, or timeouts.
- Clean-build both configurations and restore `USING_BIG_ROBOT false` before committing robot-code changes.
- Stop after publishing the physical checklist; a human with the robots must perform the gate.

## Execution Status

- Tasks 1 and 2 were completed together in the first fail-closed checkpoint.
- The implementation uses a stronger behavior test than the proposed source
  grep: it links the real `activeRobotConfig()`, converts it through
  `authorizationSnapshot()`, and rejects any enabled unvalidated gate.
- The test was observed failing first for the missing snapshot seam, then for
  the live small-robot Shadow playback authorization, before the flag was
  restored to false.
- The active configuration test passed for both small and big builds. The full
  ten-executable host suite passed, and clean big/restored-small ARM builds
  linked successfully. The only compiler warning was the existing vendored
  `json.hpp` `std::is_pod` deprecation.
- Task 3 is published as
  `docs/testing/2026-08-10-roadmap-baseline-checklist.md`. Its result cells are
  intentionally `Not run`; execution is stopped at the physical robot gate.

---

## File Structure

- `include/aon/config/robot-config.hpp`: existing authorization value types.
- `src/aon/config/robot-config.cpp`: fail-closed per-robot values.
- `tests/authorization-policy-test.cpp`: dependency-free value-policy tests for both robot configurations.
- `tests/authorization-source-test.ps1`: proves current source gates and route registration remain explicit.
- `docs/testing/2026-08-10-roadmap-baseline-checklist.md`: exact physical procedure and result table.
- `docs/CURRENT_HANDOFF.md`: current commit, configuration, authorization matrix, and next gate.

### Task 1: Lock the Authorization Contract in Host Tests

**Files:**
- Modify: `include/aon/config/robot-config.hpp`
- Create: `tests/authorization-policy-test.cpp`
- Create: `tests/authorization-source-test.ps1`

**Interfaces:**
- Consumes: `aon::config::AutonomousAuthorizations`, `ExperimentalRoute`, and `FallbackConfig`.
- Produces: `aon::config::AuthorizationSnapshot` and `safeForUnvalidatedBaseline(const AuthorizationSnapshot&)`.

- [ ] **Step 1: Write the failing value-policy test**

Add a dependency-free snapshot to the configuration header and first write tests for the intended policy:

```cpp
#include "aon/config/robot-config.hpp"

#include <cstdlib>
#include <iostream>

#define CHECK(expression) do { if (!(expression)) std::exit(1); } while (false)

int main() {
  using aon::config::AuthorizationSnapshot;
  using aon::config::safeForUnvalidatedBaseline;

  CHECK(safeForUnvalidatedBaseline({false, false, false, false, false}));
  CHECK(!safeForUnvalidatedBaseline({true, false, false, false, false}));
  CHECK(!safeForUnvalidatedBaseline({false, true, false, false, false}));
  CHECK(!safeForUnvalidatedBaseline({false, false, true, false, false}));
  CHECK(!safeForUnvalidatedBaseline({false, false, false, true, false}));
  CHECK(!safeForUnvalidatedBaseline({false, false, false, false, true}));
  std::cout << "authorization policy tests passed\n";
}
```

- [ ] **Step 2: Run the test and verify it fails**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\authorization-policy-test.cpp -o bin\host-tests\authorization-policy-test.exe
```

Expected: compilation fails because `AuthorizationSnapshot` and `safeForUnvalidatedBaseline` do not exist.

- [ ] **Step 3: Implement the minimal pure policy**

Add to `robot-config.hpp`:

```cpp
struct AuthorizationSnapshot {
  bool automaticEncoderFallback;
  bool forcedEncoderTesting;
  bool shadowPlayback;
  bool redSixBlock;
  bool jerryIoPath;
};

[[nodiscard]] constexpr bool safeForUnvalidatedBaseline(
    const AuthorizationSnapshot& value) noexcept {
  return !value.automaticEncoderFallback && !value.forcedEncoderTesting &&
         !value.shadowPlayback && !value.redSixBlock && !value.jerryIoPath;
}
```

- [ ] **Step 4: Add the source-structure regression test**

`authorization-source-test.ps1` must read `src/aon/config/robot-config.cpp`, `src/aon/auton/routine-selectors.cpp`, and both physical checklists. It fails unless both robot initializer branches contain false values for automatic fallback, forced testing, and experimental routes; the big branch contains false Shadow playback; and any true small Shadow value is accompanied by a checklist without `Not run` in its Results table. For the present evidence, expected source state is false.

- [ ] **Step 5: Run both tests**

Run:

```powershell
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\authorization-policy-test.cpp -o bin\host-tests\authorization-policy-test.exe
.\bin\host-tests\authorization-policy-test.exe
powershell -ExecutionPolicy Bypass -File tests\authorization-source-test.ps1
```

Expected: the C++ test passes; the source test fails while small Shadow playback remains true.

- [ ] **Step 6: Commit the policy seam**

```powershell
git add include/aon/config/robot-config.hpp tests/authorization-policy-test.cpp tests/authorization-source-test.ps1
git commit -m "test(config): define fail-closed authorization policy"
git push origin Testing
```

### Task 2: Restore the Evidence-Supported Configuration

**Files:**
- Modify: `src/aon/config/robot-config.cpp`
- Modify: `tests/hardware-map-test.cpp`

**Interfaces:**
- Consumes: Task 1 `AuthorizationSnapshot` policy and current physical checklist.
- Produces: `baselineAuthorizations(RobotIdentity)` and both robot configurations safe for the unvalidated baseline.

- [ ] **Step 1: Extend the configuration regression test**

Add this pure configuration function to `robot-config.hpp` and make both
`RobotConfig` initializers consume its values:

```cpp
[[nodiscard]] constexpr AuthorizationSnapshot baselineAuthorizations(
    RobotIdentity) noexcept {
  return {false, false, false, false, false};
}
```

Then make `hardware-map-test.cpp` assert:

```cpp
const auto smallAuthorizations = baselineAuthorizations(RobotIdentity::Small);
const auto bigAuthorizations = baselineAuthorizations(RobotIdentity::Big);
CHECK(!smallAuthorizations.shadowPlayback);
CHECK(safeForUnvalidatedBaseline(smallAuthorizations));
CHECK(safeForUnvalidatedBaseline(bigAuthorizations));
```

Do not instantiate PROS hardware in the host test.

- [ ] **Step 2: Run the authorization tests and confirm the failing small flag**

Run the Task 1 commands. Expected: source/configuration assertion fails only for small Shadow playback.

- [ ] **Step 3: Set only small Shadow playback to false**

Change the small `RobotConfig` initializer from:

```cpp
true,  // shadowPlaybackAuthorized; supervised physical test only.
```

to:

```cpp
false,  // shadowPlaybackAuthorized: physical checklist remains incomplete.
```

Do not change any other initializer value.

- [ ] **Step 4: Run focused and existing host tests**

Run:

```powershell
.\bin\host-tests\authorization-policy-test.exe
powershell -ExecutionPolicy Bypass -File tests\authorization-source-test.ps1
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests\hardware-map-test.cpp src\aon\config\hardware-map.cpp -o bin\host-tests\hardware-map-test.exe
.\bin\host-tests\hardware-map-test.exe
powershell -ExecutionPolicy Bypass -File tests\lemlib-validation-slot.ps1
```

Expected: all pass.

- [ ] **Step 5: Clean-build both robot configurations**

Use the installed PROS CLI/toolchain command sequence already recorded in `docs/superpowers/plans/2026-08-03-hardware-map-consistency.md`: build small from a clean tree, change only `USING_BIG_ROBOT` to true, clean-build big, restore false, clean-build small again. Record warnings; do not treat the vendored JSON deprecation as a new project warning.

- [ ] **Step 6: Review the exact behavioral diff**

Run:

```powershell
git diff -- src/aon/config/robot-config.cpp include/aon/config/robot-config.hpp tests
git diff --check
rg -n "automaticFallbackAuthorized|forcedEncoderTestingAuthorized|shadowPlaybackAuthorized|redSixBlock|jerryIoPath" include src tests docs/testing
```

Expected: only the unsupported small Shadow authorization becomes false; all other gates remain false.

- [ ] **Step 7: Commit the fail-closed configuration**

```powershell
git add src/aon/config/robot-config.cpp tests/hardware-map-test.cpp docs/CURRENT_HANDOFF.md
git commit -m "fix(config): restore unvalidated playback lock"
git push origin Testing
```

### Task 3: Publish the Physical Baseline Gate

**Files:**
- Create: `docs/testing/2026-08-10-roadmap-baseline-checklist.md`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: fail-closed build commit, existing LemLib/native checklist requirements, Shadow recording and playback checklists.
- Produces: one auditable gate that authorizes the next roadmap checkpoint only after measured completion.

- [ ] **Step 1: Create the checklist with exact tests**

The checklist must contain result rows for:

1. small-robot cold boot and active configuration confirmation;
2. LemLib figure-eight completion time, final X/Y/heading, crossover behavior, and Controller-X cancellation;
3. native Kevin route GUI selection, drivetrain behavior, mechanisms, and safe stop after a fresh boot;
4. Shadow SD empty-card behavior, five-second recording save, reboot/load, overwrite, delete, and card-removal recovery;
5. drivetrain-only Shadow playback completion;
6. Shadow Controller-X cancellation;
7. one-intake/one-piston Shadow playback;
8. big-robot clean boot/configuration check without enabling playback;
9. tester, date/time, robot identity, firmware commit, SD card format/capacity, and pass/fail notes.

Every result cell begins `Not run`. The document states that a supervised playback build may set only the small Shadow flag true and must restore false after the test.

- [ ] **Step 2: Replace stale handoff claims**

Update `CURRENT_HANDOFF.md` so its current authorization matrix matches source. Retain historical incident notes, but label completed history separately from active state. Name the new checklist as the next physical gate.

- [ ] **Step 3: Verify documentation consistency**

Run:

```powershell
rg -n "shadowPlaybackAuthorized|PLAY LOCKED|Not run|Pending Physical Gate|Current gate" docs src/aon/config/robot-config.cpp
git diff --check
```

Expected: current-state statements say both production configurations are locked; historical statements are not presented as current state.

- [ ] **Step 4: Commit and push the physical gate**

```powershell
git add docs/testing/2026-08-10-roadmap-baseline-checklist.md docs/CURRENT_HANDOFF.md
git commit -m "docs: publish roadmap physical baseline gate"
git push origin Testing
```

- [ ] **Step 5: Stop for physical measurements**

Do not start device ownership changes, characterization motion, estimator-to-LemLib integration, new autonomous behavior, or driver assistance. A human must run the checklist and commit the measured results.

### Task 4: Accept Measured Baseline Results

**Files:**
- Modify: `docs/testing/2026-08-10-roadmap-baseline-checklist.md`
- Modify: `docs/CURRENT_HANDOFF.md`

**Interfaces:**
- Consumes: human-entered measured results from the exact tested commit.
- Produces: authorization to start the next named roadmap plan, or a fail-closed diagnosis checkpoint.

- [ ] **Step 1: Validate completeness without altering measurements**

Reject advancement if any required row remains `Not run`, lacks the tested commit/robot identity, reports failure, or omits Controller-X and stopped-output observations.

- [ ] **Step 2: Record the decision**

If all rows pass, name the next approved plan in `CURRENT_HANDOFF.md`. If any row fails, keep all new authorizations false, link the failing observation, and create a diagnosis plan rather than implementing an assumed fix.

- [ ] **Step 3: Commit measured evidence separately**

```powershell
git add docs/testing/2026-08-10-roadmap-baseline-checklist.md docs/CURRENT_HANDOFF.md
git commit -m "test(robot): record roadmap baseline results"
git push origin Testing
```
