# GUI Guide

## Overview

The GUI system provides six main features:

1. **Registered Autons** — Register functions and select them from the Debug Menu
2. **Auton Runner** — Execute autonomous routines on-demand during testing
3. **Tunable Variables** — Live adjustment of parameters without rebuilding
4. **Data Menu** — Live display of registered numeric values with reset controls
5. **Live Graph** — Real-time visualization of X/Y data (e.g., odometry)
6. **Field Mapper** — Real-time robot path trace on a top-down field view with arc measurement

---

## Table of Contents

- [Debug Mode Toggle](#debug-mode-toggle)
- [Changing Preset Autonomous Routines](#changing-preset-autonomous-routines)
- [Quick Start](#quick-start)
- [Registering Test Functions](#registering-test-functions)
- [Auton Runner](#auton-runner)
- [Tunable Variables](#tunable-variables)
- [Data Menu](#data-menu)
- [Live Graph](#live-graph)
- [Field Mapper](#field-mapper)
- [Complete Example](#complete-example)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)

---

## Debug Mode Toggle

Controlled by `TESTING_AUTONOMOUS` in [constants.hpp](../../constants.hpp):

```cpp
#define TESTING_AUTONOMOUS true   // Uses GuiDebug — full debug features
#define TESTING_AUTONOMOUS false  // Uses Gui — competition mode (default)
```

The implementation in [src/aon/tools/gui/gui.cpp](../../../src/aon/tools/gui/gui.cpp) selects the type at compile time:

```cpp
#if TESTING_AUTONOMOUS
static GuiDebug gui_impl;
#else
static Gui gui_impl;
#endif
Gui& gui = gui_impl;
```

| `TESTING_AUTONOMOUS` | GUI Type | Main Menu |
|---|---|---|
| `true` | `GuiDebug` | Split bar: **AUTONS** + **DEBUG** |
| `false` | `Gui` | Full-width **AUTONS** only |

**Note:** The base `Gui` class provides no-op virtual methods for all debug APIs (`VariableChanger`, `RegisterTestFunction`, `RegisterDataEntry`, etc.) so registration calls compile in both modes. Runtime behavior is only active under `GuiDebug`.

**If failing to switch between them delete bin file and d file**

---

## Changing Preset Autonomous Routines

The **AUTONS** menu shows preset routines for Red, Blue, and Skills. To change them:

### 1. Add a forward declaration

Open [gui.hpp](gui.hpp) and add a declaration inside `namespace aon`:

```cpp
namespace aon {
  int MyNewRedRoutine();
}
```

Implement the function in [autonomous-routines.hpp](../../competition/autonomous-routines.hpp) inside `namespace aon`, returning `int`.

### 2. Update the option arrays

In [gui.hpp](gui.hpp), update the relevant array:

```cpp
AutonOption RedAutonOptions[AutonOptionsCount] = {
  {"Red AUT1", aon::MyNewRedRoutine},
  {"Red AUT2", aon::RedRingsRoutine},
  {"Red AUT3", aon::RedRingsRoutine},
};
```

`AutonOptionsCount` is 3 — don't change it unless you resize all three arrays (`RedAutonOptions`, `BlueAutonOptions`, `SkillsAutonOptions`) together.

### 3. Rebuild

```bash
pros build
```

Navigate to **AUTONS** on the brain to confirm.

> For dynamically registered routines, use **Debug Menu → Registered Autons** instead — see [Registering Test Functions](#registering-test-functions).

---

## Quick Start

Call all `Set*Register` functions **before** `InitializeGui()`.

```cpp
void initialize() {
  aon::gui.SetTestRegister([]{
    aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui.RegisterTestFunction(&BlueRingsRoutine, "Blue Rings");
  });

  aon::gui.SetVariableRegister([]{
    aon::gui.VariableChanger(DRIVE_KP, "Drive kP");
  });

  aon::gui.SetDataRegister([]{
    aon::gui.RegisterDataEntry("X",       [](){ return drivetrain.odom.getX(); });
    aon::gui.RegisterDataEntry("Y",       [](){ return drivetrain.odom.getY(); });
    aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
  });

  aon::gui.SetGraphDataProviders(
    []() { return drivetrain.odom.getX(); },
    []() { return drivetrain.odom.getY(); }
  );

  aon::InitializeGui();
  pros::Task guiLoopTask([]{ aon::gui.RunLoop(); });
}
```

---

## Registering Test Functions

Test functions appear in **Debug Menu → Registered Autons** and can be selected and executed from the Auton Runner.

### API

```cpp
// int-returning function pointer
aon::gui.RegisterTestFunction(&YourFunction, "Display Name");

// void-returning function pointer (wrapped automatically)
aon::gui.RegisterTestFunction(&YourVoidFunction, "Display Name");

// Lambda or std::function
aon::gui.RegisterTestFunction([]() -> int {
  pros::delay(1000);
  return 0;
}, "Display Name");
```

Wrap registrations in a `SetTestRegister` callback:

```cpp
aon::gui.SetTestRegister([]{
  aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
  aon::gui.RegisterTestFunction(&BlueRingsRoutine, "Blue Rings");
});
```

### How It Works

1. `SetTestRegister(callback)` stores the callback.
2. When **Registered Autons** opens, the callback is invoked once to populate the list.
3. Tap a name to select it — the screen navigates automatically to the Auton Runner.

### Notes

- Duplicate names are silently ignored.
- The callback fires lazily when the screen opens, not at startup.
- Functions should return `0` on success, non-zero on failure.

---

## Auton Runner

The Auton Runner (**Debug Menu → Auton Runner**) executes a selected autonomous routine on demand.

### How Execution Works

When **RUN** is pressed:

1. The selected function is registered with `AutonomousReader` under the key `"autonomous"`.
2. `AutonomousReader->ExecuteFunction("autonomous")` is called **synchronously** in the GUI loop task.
3. The screen redraws to show the auton name in **orange** before execution begins.
4. The GUI is completely blocked — touch input is not processed while the routine runs.
5. When the routine returns, the screen updates to **cyan "COMPLETED"**.

> **There is no stop button.** Because execution is synchronous, touch input cannot be received while the auton is running. The only way to stop an auton is to let it run to completion.

### Button States

| Button | Color | Condition | Action |
|--------|-------|-----------|--------|
| **RUN** | Green | Auton selected, not running | Starts execution |
| **RUN** | Gray | No auton selected | Does nothing |

### Status Display

| Color | Meaning |
|-------|---------|
| Green | Auton selected and ready |
| Orange | Auton currently executing — GUI is blocked |
| Cyan | Auton completed and returned |
| Red | No auton selected |

### Screen Layout

```
┌────────────────────────────────────────────────────────┐
│ [BACK]           Auton Runner              [MENU]      │
├────────────────────────────────────────────────────────┤
│ ┌─────────────────────────────────────┐  ┌──────────┐  │
│ │ Selected:                           │  │  VARS    │  │
│ │ Red Rings                (green)    │  └──────────┘  │
│ └─────────────────────────────────────┘                │
│                                                        │
│                   ┌──────────────┐                     │
│                   │     RUN      │  (green / gray)     │
│                   └──────────────┘                     │
└────────────────────────────────────────────────────────┘
```

### VARS Button

The orange **VARS** button navigates to the Variables menu with the Auton Runner set as the return destination. Press **BACK** from Variables to return here. Typical loop:

```
Select Auton → RUN → Observe → VARS → Adjust → RUN again
```

### Workflow

1. Go to **Debug Menu → Registered Autons**, tap a function name → auto-navigates to Auton Runner.
2. Confirm the selected name is shown in green.
3. Tap **RUN** → name turns orange, GUI blocks while the auton executes.
4. When the routine returns → status turns cyan **COMPLETED**.
5. Tap **VARS** to adjust parameters, then **BACK** to return here and run again.

---

## Tunable Variables

Variables appear in **Debug Menu → Variables** with +/− buttons for live adjustment.

### API

```cpp
aon::gui.VariableChanger(variableRef, "Display Name");
```

Wrap registrations in a `SetVariableRegister` callback:

```cpp
aon::gui.SetVariableRegister([]{
  aon::gui.VariableChanger(DRIVE_KP,  "Drive kP");
  aon::gui.VariableChanger(DRIVE_KI,  "Drive kI");
  aon::gui.VariableChanger(MAX_SPEED, "Max Speed");
});
```

### How It Works

1. `SetVariableRegister(callback)` stores the callback.
2. When **Variables** opens, the callback is invoked once.
3. Each entry shows six buttons: `-10`, `-1`, `-0.1` | `+0.1`, `+1`, `+10`.
4. Tapping a button immediately applies the delta to the variable in memory.
5. Up to 2 variables per page; use **PREV**/**NEXT** to paginate.

### Notes

- Pass variables **by reference** — the GUI modifies the actual variable.
- Use global or `inline` variables so changes persist.
- Duplicate names are silently ignored.

---

## Data Menu

The Data screen (**Debug Menu → Data**) shows live numeric values with VARS and RESET shortcuts.

### API

```cpp
aon::gui.SetDataRegister([]{
  aon::gui.RegisterDataEntry("X",       [](){ return drivetrain.odom.getX(); });
  aon::gui.RegisterDataEntry("Y",       [](){ return drivetrain.odom.getY(); });
  aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
});
```

### How It Works

1. `SetDataRegister(callback)` stores the callback.
2. When the Data screen opens, the callback is invoked once.
3. Each entry displays as `Name: value` with three decimal places.
4. Up to 6 entries per page; use **PREV**/**NEXT** for pagination.
5. **VARS** button opens the Variables menu.
6. **RESET** button invokes the registered reset handler.

### RESET Handler

```cpp
aon::gui.RegisterResetHandler("ResetOdom", []{
  drivetrain.odom.resetCurrent(0.0, 0.0, 0.0);
});
```

- The most recently registered handler is the active one.
- No-op on base `Gui`; safe to call in all builds.
- Register before `InitializeGui()` to have it available immediately.

---

## Live Graph

The Live Graph (**Debug Menu → Live Graph**) plots real-time X/Y data with auto-scaling axes.

### API

```cpp
aon::gui.SetGraphDataProviders(
  []() -> double { return /* X */; },
  []() -> double { return /* Y */; }
);
```

### How It Works

1. `SetGraphDataProviders(xFunc, yFunc)` stores the two callbacks.
2. While the Live Graph screen is open, a new point is sampled every ~300 ms.
3. Up to 300 points are kept in a circular buffer; oldest data is dropped as new arrives.
4. Axes auto-scale around the data range.
5. Current X/Y values are shown in the corner.

### Examples

```cpp
// Odometry path
aon::gui.SetGraphDataProviders(
  []() { return drivetrain.odom.getX(); },
  []() { return drivetrain.odom.getY(); }
);

// Time vs motor velocity
aon::gui.SetGraphDataProviders(
  []() { return (double)pros::millis() / 1000.0; },
  []() { return LEFT_MOTORS->get_velocity(); }
);
```

---

## Field Mapper

The Field Mapper (**Debug Menu → Field Mapper**) draws a top-down trace of the robot's path on a 6-tile VEX field. It also lets you measure arc geometry between any two points on the path so you can directly copy the values into `driveAngleOfArc()`.

### API

```cpp
aon::gui->setMapDataProvider([]() -> aon::Pose {
  return {drivetrain.getX(), drivetrain.getY(),
          drivetrain.getTheta() * M_PI / 180.0};
});
```

`Pose` fields:
- `x` — robot X position in **inches**
- `y` — robot Y position in **inches**
- `theta` — robot heading in **radians**

> `getTheta()` returns degrees — multiply by `M_PI / 180.0` before passing it.

### How It Works

1. `setMapDataProvider(callback)` stores the pose provider.
2. While the Field Mapper screen is open, the pose is sampled periodically and appended to the path buffer (up to 1 000 points).
3. The full path is drawn in **cyan**. The most recent position shows a heading arrow.
4. **CLEAR** erases the recorded path and resets arc state.

### Screen Layout

```
┌────────────────────────────────────────────────────────────┐
│ [BACK]           FIELD MAPPER                    [CLEAR]   │
├─────────────────────┬──────────────────────────────────────┤
│                     │  X:  12.34"                          │
│   6-tile field      │  Y:  -5.67"                          │
│   (cyan path,       │  H:  90.0°                           │
│    yellow arc seg,  │  D:  47.83"                          │
│    green dot =      │ ────────────────────                 │
│    arc start)       │  ARC:                                │
│                     │  Radius: 12.50"  ΔHdg: 90°           │
│                     │  ArcLen: 19.63"                      │
│                     │  Chord:  17.68"                      │
│                     │  Inner: 6.97"  Out: 18.03"           │
├─────────────────────┼──────────────────────────────────────┤
│                     │  [MARK S]         [MARK E]           │
└─────────────────────┴──────────────────────────────────────┘
```

### Data Panel

| Field | Description |
|-------|-------------|
| **X** | Current robot X in inches |
| **Y** | Current robot Y in inches |
| **H** | Current heading in degrees |
| **D** | Total path distance traveled in inches |

### Arc Measurement

Use the two buttons at the bottom of the data panel:

| Button | Color | Action |
|--------|-------|--------|
| **MARK S** | Dark green | Mark the current end of the path as the arc start; arc segment highlights yellow |
| **MARK E** | Dark blue | Compute arc from marked start to current end |

Once measured, the arc segment highlights in **yellow** (green dot marks the start) and the panel displays:

| Field | Description | Use in code |
|-------|-------------|-------------|
| **Radius** | Robot-center arc radius in inches | First arg of `driveAngleOfArc(radius, angle)` |
| **ΔHdg** | Total heading change in degrees (shown beside Radius) | Second arg of `driveAngleOfArc(radius, angle)` |
| **ArcLen** | Cumulative path length along the arc in inches | — |
| **Chord** | Straight-line distance start → end in inches | — |
| **Inner** | Inner drive-wheel radius (`Radius − DRIVE_WIDTH/2`) | Reference only |
| **Out** | Outer drive-wheel radius (`Radius + DRIVE_WIDTH/2`) | Reference only |

### Using Arc Results in Autonomous

```cpp
// Field Mapper showed: Radius: 12.50", ΔHdg: 90°, turning right (clockwise)
drivetrain.driveAngleOfArc(12.5, 90.0);   // positive radius = clockwise

// Turning left (counter-clockwise): negate the radius
drivetrain.driveAngleOfArc(-12.5, 90.0);
```

- **Radius** is already the robot-center radius — pass it directly, no adjustment needed.
- **ΔHdg** is the arc angle — use it as the `angle` parameter.
- The Field Mapper does not auto-detect turn direction: use **positive radius for clockwise**, **negative for counter-clockwise**.

### Notes

- Only active under `GuiDebug` (`TESTING_AUTONOMOUS true`). No-op on base `Gui`.
- Call `setMapDataProvider()` **before** `aon::gui->initialize()`.
- Buffer holds up to 1 000 points; once full, new points are dropped until **CLEAR** is pressed.
- Arc results use `DRIVE_WIDTH` from `constants.hpp` for Inner/Out values, so they automatically reflect whichever robot is selected via `USING_BIG_ROBOT`.

---

## Complete Example

```cpp
// main.cpp

int TestRoutine() {
  pros::delay(3000);
  return 0;
}

void initialize() {
  aon::gui.SetTestRegister([]{
    aon::gui.RegisterTestFunction(&TestRoutine,     "Test 3s");
    aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
  });

  aon::gui.SetVariableRegister([]{
    aon::gui.VariableChanger(aon::DRIVE_KP,  "Drive kP");
    aon::gui.VariableChanger(aon::DRIVE_KI,  "Drive kI");
    aon::gui.VariableChanger(aon::MAX_SPEED, "Max Speed");
  });

  aon::gui.SetDataRegister([]{
    aon::gui.RegisterDataEntry("X",       [](){ return drivetrain.odom.getX(); });
    aon::gui.RegisterDataEntry("Y",       [](){ return drivetrain.odom.getY(); });
    aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
  });

  aon::gui.SetGraphDataProviders(
    []() { return drivetrain.odom.getX(); },
    []() { return drivetrain.odom.getY(); }
  );

  aon::gui->setMapDataProvider([]() -> aon::Pose {
    return {drivetrain.getX(), drivetrain.getY(),
            drivetrain.getTheta() * M_PI / 180.0};
  });

  aon::gui.RegisterResetHandler("ResetOdom", []{
    drivetrain.odom.resetCurrent(0.0, 0.0, 0.0);
  });

  aon::InitializeGui();
  pros::Task guiLoopTask([]{ aon::gui.RunLoop(); });
}
```

### Menu Flow

| Step | Screen | What Happens |
|------|--------|--------------|
| 1 | Main Menu | Tap **DEBUG** |
| 2 | Debug Menu | Choose from: Registered Autons, Live Graph, Auton Runner, Variables, Data |
| 3 | Registered Autons | Tap a name → auto-navigates to Auton Runner with it selected |
| 4 | Auton Runner | Tap green **RUN** → GUI blocks, auton runs, screen shows orange |
| 5 | Auton Runner | Routine returns → screen shows cyan **COMPLETED** |
| 6 | Variables (via **VARS**) | Adjust parameters live; **BACK** returns to Auton Runner |

---

## API Reference

### Initialization

| Function | Description |
|----------|-------------|
| `aon::InitializeGui()` | Initialize the GUI; call once in `initialize()` |
| `aon::gui.RunLoop()` | Blocking GUI event loop; run in a dedicated `pros::Task` |

### Test Function Registration

| Function | Description |
|----------|-------------|
| `aon::gui.SetTestRegister(callback)` | Store the callback (invoked lazily when Registered Autons opens) |
| `aon::gui.RegisterTestFunction(func, name)` | Register a function with a display name |

Supported signatures: `int(*)()`, `void(*)()`, `std::function<int()>`, lambda.

### Variable Registration

| Function | Description |
|----------|-------------|
| `aon::gui.SetVariableRegister(callback)` | Store the callback (invoked lazily when Variables opens) |
| `aon::gui.VariableChanger(var, name)` | Register a variable for live +/− adjustment |

### Data Registration

| Function | Description |
|----------|-------------|
| `aon::gui.SetDataRegister(callback)` | Store the callback (invoked lazily when Data opens) |
| `aon::gui.RegisterDataEntry(name, getter)` | Register a `std::function<double()>` getter with a display name |
| `aon::gui.RegisterResetHandler(name, cb)` | Register a named reset handler; last registered is active |

### Graph

| Function | Description |
|----------|-------------|
| `aon::gui.SetGraphDataProviders(xFunc, yFunc)` | Set X and Y data provider callbacks |

### Field Mapper

| Function | Description |
|----------|-------------|
| `aon::gui->setMapDataProvider(poseFunc)` | Set a `std::function<Pose()>` callback; `Pose.theta` must be in **radians** |

### State Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `aon::gui.selectedAuton` | `AutonOption` | `{"None", nullptr}` | Currently selected preset auton |
| `aon::gui.selectedAutonName` | `std::string` | `"None"` | Display name of selected auton |
| `aon::gui.selectedAutonInvoker` | `std::function<int()>` | `nullptr` | Invoker for debug-registered autons |
| `aon::gui.selectedRedAut` | `int` | `0` | Preselect Red auton index (1–3) |
| `aon::gui.selectedBlueAut` | `int` | `0` | Preselect Blue auton index (1–3) |
| `aon::gui.selectedSkill` | `int` | `0` | Preselect Skills auton index (1–3) |

---

## Troubleshooting

### Function not appearing in Registered Autons?

1. Name must be unique — duplicates are silently ignored.
2. Verify `SetTestRegister()` is called before `InitializeGui()`.
3. Close and reopen the Registered Autons screen to re-trigger the callback.

### Auton won't run from Auton Runner?

1. Select an auton from **Registered Autons** or **AUTONS** first — RUN is gray until one is selected.
2. Confirm the function is registered with `RegisterTestFunction()`.

### The GUI freezes when I press RUN — is that a bug?

No. The Auton Runner calls `ExecuteFunction` **synchronously** in the GUI loop task. The screen intentionally shows orange while blocked and cyan when the routine returns. There is no mechanism to stop or interrupt a running auton — it always runs to completion.

### Data screen shows "No Data Registered"?

1. Call `SetDataRegister(...)` before `InitializeGui()`.
2. Confirm `TESTING_AUTONOMOUS` is `true` — the Data screen is only active on `GuiDebug`.
3. Rebuild with `pros build`.

### Graph not updating?

1. Confirm `SetGraphDataProviders()` was called before `InitializeGui()`.
2. Verify the lambdas return valid `double` values (not NaN/infinity).
3. You must be on the **Live Graph** screen — data is only sampled while that screen is active.

### Variable changes not persisting?

1. Variables must be passed **by reference** to `VariableChanger()`.
2. Use global or `inline` scope so the variable outlives the callback.

### BACK from Variables doesn't return to Auton Runner?

This is only supported when navigating via the **VARS** button on the Auton Runner screen. Navigating to Variables from the Debug Menu sets the return destination to the Debug Menu instead.

---

## Files

- [gui.hpp](gui.hpp) — Base `Gui` class, auton option arrays, shared state
- [gui-debug.hpp](gui-debug.hpp) — `GuiDebug` class with full debug API


```cpp
// In constants.hpp
#define TESTING_AUTONOMOUS true   // Uses GuiDebug - full debug features
#define TESTING_AUTONOMOUS false  // Uses Gui - competition mode (default)
```

The implementation in `src/aon/tools/gui/gui.cpp` automatically selects the correct GUI:

```cpp
// Automatic selection based on TESTING_AUTONOMOUS flag:
#if TESTING_AUTONOMOUS
static GuiDebug gui_impl;
#else
static Gui gui_impl;
#endif
Gui& gui = gui_impl;
```

After changing `TESTING_AUTONOMOUS`, rebuild your project to pick up the selected GUI behavior.

**Note about debug-only addons:** The base `Gui` class provides no-op virtual methods for most debug APIs so user code that calls `VariableChanger`, `RegisterTestFunction`, `RegisterDataEntry`, etc., will compile and link even when `Gui` (non-debug) is in use. The full runtime behavior (variables list, registered autons, data screens, live graph) is implemented only in `GuiDebug`.

Options:
- Enable the debug GUI by setting `TESTING_AUTONOMOUS` to `true` in `include/aon/constants.hpp` to get full debug features at runtime.
- Or keep `TESTING_AUTONOMOUS = false` (competition mode) and treat debug registrations as no-ops (they won't populate debug screens at runtime).

### [Registering Test Functions](#registering-test-functions)
```cpp
aon::gui.SetTestRegister([]{
  aon::gui.RegisterTestFunction(&MyFunction, "My Test");
});
```

### [Auton Runner](#auton-runner)
Access via: **DEBUG Menu → Auton Runner** (or select a test from Registered Autons)
- Green **RUN** button: Start the selected auton
- Red **MOV** label: Shown while auton is running — execution is **synchronous** so no input is received; the GUI is blocked until the routine returns
- **VARS** button: Quick access to tunable variables

### [Tunable Variables](#tunable-variables)
```cpp
aon::gui.SetVariableRegister([]{
  aon::gui.VariableChanger(myVariable, "Variable Name");
});
```

### [Data Menu](#data-menu)
```cpp
aon::gui.SetDataRegister([]{  
  aon::gui.RegisterDataEntry("X", [](){ return drivetrain.odom.getX(); });
  aon::gui.RegisterDataEntry("Y", [](){ return drivetrain.odom.getY(); });
  aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
});
```

### [Live Graph](#live-graph)
```cpp
aon::gui.SetGraphDataProviders(
  []() { return data_x; },
  []() { return data_y; }
);
```

---

## Changing Autonomous Routines

The main menu displays preset autonomous routines for Red, Blue, and Skills alliances. To change these:

### 1. Modify the Auton Options (in header)


Open [gui.hpp](gui.hpp) and jump directly to the `AutonOption` arrays:

- [Red Auton Options](gui.hpp#L77)
- [Blue Auton Options](gui.hpp#L82)
- [Skills Auton Options](gui.hpp#L88)

Note: when adding a new preset auton, also add a forward declaration for the function in the `aon` namespace at the top of `gui.hpp` so the GUI can reference it (for example: `int ForwardBackTurnRoutine();`). Then implement the function in [autonomous-routines.hpp](../../competition/autonomous-routines.hpp). Ensure the function is in the `aon::` namespace and returns an `int`.

```cpp
// Forward declarations at top of gui.hpp (inside namespace aon)
namespace aon {
  int RedRingsRoutine();
  int BlueRingsRoutine();
  int ForwardBackTurnRoutine();
  // Add other auton routine declarations as needed


// Auton option arrays (instance members of the Gui class — no static/inline)
AutonOption RedAutonOptions[AutonOptionsCount] = {
  {"Red ForwardBackTurn", aon::ForwardBackTurnRoutine},
  {"Red AUT2", aon::RedRingsRoutine},
  {"Red AUT3", aon::RedRingsRoutine},
};

AutonOption BlueAutonOptions[AutonOptionsCount] = {
  {"Blue AUT1", aon::BlueRingsRoutine},
  {"Blue AUT2", aon::BlueRingsRoutine},
  {"Blue AUT3", aon::BlueRingsRoutine},
};

AutonOption SkillsAutonOptions[AutonOptionsCount] = {
  {"Skills AUT1", aon::RedRingsRoutine},
  {"Skills AUT2", aon::RedRingsRoutine},
  {"Skills AUT3", aon::RedRingsRoutine},
};

}
```

**To change an auton:**

1. Replace the function pointer with your function name
2. Optionally change the display name

Example:
```cpp
AutonOption RedAutonOptions[AutonOptionsCount] = {
  {"Red AUT1", aon::MyNewRedRoutine},      // Changed function
  {"Red AUT2", aon::RedRingsRoutine},
  {"Red Safe", aon::RedSafeRoutine},       // Changed name
};
```

### 2. Rebuild

After modifying, rebuild your project:
```bash
pros build
```

### 3. Test

Navigate to **AUTONS** menu on the brain to see your updated options.

### Notes

- **Names must match**: Ensure the function names match exactly with forward declarations or actual function definitions
- **Naming style**: Functions should be in the `aon::` namespace and return `int` 
- **Array size**: Don't change `AutonOptionsCount` (set to 3) unless you also change all three arrays
- **Only for preset autons**: If you want to dynamically register autons, use the Debug Menu's **Registered Autons** feature instead (see [Registering Test Functions](#registering-test-functions))

---

## Quick Start

### Basic Setup (in `initialize()`)

**Important:** Call all `Set*Register` functions **before** `InitializeGui()` so entries are eagerly seeded during initialization.

```cpp
void initialize() {
  // Register ALL callbacks BEFORE InitializeGui()
  
  // Register variables (optional)
  aon::gui.SetVariableRegister([]{  
    aon::gui.VariableChanger(MAX_RPM_TEST, "Max RPM Test");
  });
  
  // Register test functions (optional)
  aon::gui.SetTestRegister([]{  
    aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui.RegisterTestFunction(&BlueRingsRoutine, "Blue Rings");
  });
  
  // Register data entries (optional)
  aon::gui.SetDataRegister([]{  
    aon::gui.RegisterDataEntry("X", [](){ return drivetrain.odom.getX(); });
    aon::gui.RegisterDataEntry("Y", [](){ return drivetrain.odom.getY(); });
    aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
  });
  
  // Set up live graph (optional)
  aon::gui.SetGraphDataProviders(
    []() { return drivetrain.odom.getX(); },
    []() { return drivetrain.odom.getY(); }
  );
  
  // NOW initialize the GUI (this eagerly seeds all registers above)
  aon::InitializeGui();
```
Note: the initialization screen now renders the primary and secondary messages with a typewriter-style animation (see `src/aon/tools/gui/gui.cpp`).


---

## Registering Test Functions

Test functions appear in **Debug Menu 1: "Registered Autons"** and allow you to select and run autonomous routines.

### API

```cpp
// Register with int-returning function pointer
aon::gui.RegisterTestFunction(&YourAutonFunction, "Display Name");

// Register with void-returning function pointer
aon::gui.RegisterTestFunction(&YourVoidFunction, "Display Name");

// Register with std::function or lambda
aon::gui.RegisterTestFunction([]() -> int {
  // your code
  return 0;
}, "Display Name");
```

### How It Works

1. Call `SetTestRegister(callback)` with a callback containing your `RegisterTestFunction()` calls
2. When you open **Debug Menu → Registered Autons**, the callback is invoked once to populate the list
3. Tap a function to select it
4. Go to **Debug Menu 2 (Auton Runner)** and press **RUN**
5. The auton executes **synchronously** in the GUI loop task via the `AutonomousReader`; the GUI is blocked until the routine returns.

### Example

```cpp
void initialize() {
  aon::InitializeGui();
  
  aon::gui.SetTestRegister([]{
    aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui.RegisterTestFunction(&BlueRingsRoutine, "Blue Rings");
    aon::gui.RegisterTestFunction([]() -> int {
      // Inline test
      return 0;
    }, "Quick Test");
  });
}
```

### Notes

- **Deduplication**: Duplicate names are ignored; later registrations with the same name won't be added
- **Lazy Loading**: The register callback is called when Debug Menu 1 opens, not at startup to avoid constant searching
- **Name Uniqueness**: Each function's display name must be unique
- **Return Value**: Functions should return `0` on success, non-zero on failure

---

## Auton Runner

The **Auton Runner** (Debug Menu 2) allows you to execute autonomous routines **during opcontrol** with full run/stop controls. This is essential for testing autonomous code without repeatedly restarting the program.

### Features

- **Run at your own choice**: Execute autons while in driver control mode

 - **No built-in watchdog**: The GUI does not implement a 30-second watchdog. If you need an execution timeout, implement it in your autonomous routine or in a separate safety task (for example `autonSafety`).
- **Quick Access to Variables**: Jump directly to the Variables menu from Auton Runner
- **Visual Status**: Shows running (orange), completed (cyan), or ready (green) states

### How to Access

1. **Main Menu** → Tap **DEBUG**
2. **Debug Menu** → Tap **Auton Runner**
3. Or: **Registered Autons** → Select a test → Automatically navigates to Auton Runner

### Screen Layout

```
┌────────────────────────────────────────────────────────┐
│ [BACK]           Auton Runner              [MENU]      │
├────────────────────────────────────────────────────────┤
│ ┌──────────────────────────────────┐  ┌─────────┐      │
│ │ Selected:                        │  │  VARS   │      │
│ │ Red Rings                        │  └─────────┘      │
│ └──────────────────────────────────┘                   │
│                                                        │
│                   ┌─────────────────┐                  │
│                   │       RUN       │                  │
│                   └─────────────────┘                  │
└────────────────────────────────────────────────────────┘
```

### Button States

| Button Color | State | Action |
|--------------|-------|--------|
| **Green RUN** | Auton selected, ready | Tap to start auton |
| **Red MOV** | Auton running | Visual indicator only — execution is synchronous, GUI is blocked until routine returns |
| **Gray RUN** | No auton selected | Disabled (select an auton first) |

### Status Display

| Color | Meaning |
|-------|---------|
| **Green** | Auton selected and ready to run |
| **Orange** | Auton currently executing |
| **Cyan** | Auton completed successfully |
| **Red** | No auton selected |

### Workflow Example

1. **Select an Auton**:
   - Go to **Debug Menu → Registered Autons** and tap a test function, OR
   - Go to **AUTONS** menu and select a preset auton

2. **Navigate to Auton Runner**:
   - From Registered Autons: Automatically navigates after selection
   - From anywhere: **DEBUG → Auton Runner**

3. **Run the Auton**:
   - Tap the green **RUN** button
   - The auton executes **synchronously** — the GUI is blocked until the routine returns
   - Status shows orange with auton name

4. **After Completion**:
   - Status shows cyan "COMPLETED"
   - Ready to run again or select a different auton

### Quick Variable Access

The **VARS** button (orange, top-right of the selected auton panel) provides quick access to tunable variables:

- Tap **VARS** to jump to the Variables menu
- Adjust parameters like kP, kI, kD on the fly
- Use **BACK** to return to the Auton Runner
- Run the auton again with updated values

This enables rapid iteration:
```
Select Auton → Run → Observe → VARS → Adjust → Run Again
```

### Notes

- **No built-in watchdog**: The GUI does not implement a 30-second watchdog. If a timeout is required, implement it inside your autonomous routine.
- **Synchronous Execution**: The auton runs synchronously in the GUI loop task — the GUI is fully blocked and no touch input is processed until the routine returns. There is no way to interrupt a running auton.
- **Any Auton Source**: Works with both registered test functions AND preset autons from the AUTONS menu

---

## Tunable Variables

Tunable variables appear in **Debug Menu 3: "Variables"** with +/- buttons to adjust values in real-time.

### API

```cpp
aon::gui.VariableChanger(VariableReference, "Display Name");
```

### How It Works

1. Call `SetVariableRegister(callback)` with a callback containing your `VariableChanger()` calls
2. When you open **Debug Menu 3**, the callback is invoked to populate the list
3. Each variable shows with six buttons:
   - `-10`, `-1`, `-0.1` (decrement)
   - `+0.1`, `+1`, `+10` (increment)
4. Tap buttons to adjust the variable live
5. Changes persist while the program runs

### Example

```cpp
double MAX_RPM_TEST = 100.0;
double DRIVE_KP = 0.5;
double DRIVE_KI = 0.02;

void initialize() {
  aon::InitializeGui();
  
  aon::gui.SetVariableRegister([]{
    aon::gui.VariableChanger(MAX_RPM_TEST, "Max RPM Test");
    aon::gui.VariableChanger(DRIVE_KP, "Drive kP");
    aon::gui.VariableChanger(DRIVE_KI, "Drive kI");
  });
}
```

### Notes

- **Reference Required**: Pass variables by reference; the GUI adjusts the actual variable
- **Deduplication**: Duplicate names are ignored
- **Lazy Loading**: The register callback is called when Debug Menu 3 opens
- **Precision**: Adjustments use double precision; increment/decrement values are fixed (+/-0.1, +/-1, +/-10)

---

## Live Graph

The **Live Graph** screen (Debug Menu → "Live Graph") displays real-time X/Y data with auto-scaling axes.

### API

```cpp
aon::gui.SetGraphDataProviders(
  []() -> double { return /* X data */ },
  []() -> double { return /* Y data */ }
);
```

### How It Works

1. Call `SetGraphDataProviders(xCallback, yCallback)` with lambdas returning double values
2. Open **Debug Menu → Live Graph**
3. The graph displays points as they're collected (300-point circular buffer)
4. Axes auto-scale with 10% padding
5. Current X/Y values shown in corner
6. Refreshes every ~300ms

### Example

```cpp
void initialize() {
  aon::InitializeGui();
  
  // Graph odometry (X/Y position)
  aon::gui.SetGraphDataProviders(
    []() { return drivetrain.odom.getX(); },
    []() { return drivetrain.odom.getY(); }
  );
}
```

### Advanced Example (Custom Data)

```cpp
void initialize() {
  aon::InitializeGui();
  
  // Graph time vs motor velocity
  aon::gui.SetGraphDataProviders(
    []() { return (double)pros::millis() / 1000.0; },  // Time in seconds
    []() { return LEFT_MOTORS->get_velocity(); }       // Motor velocity
  );
}
```

### Notes

- **Any Data Source**: Use any function returning double—not limited to odometry
- **Circular Buffer**: Stores 300 most recent points; oldest data is discarded as new arrives
- **Auto-Scale**: Y-axis scales automatically; toggle via the display
- **Real-Time**: Updates every ~300ms during operation

## Data Menu

The Data screen (Debug Menu → "Data") shows live numeric values registered by user code and provides quick access to variables and odometry reset controls.

- **Access:** Debug Menu → Data
- **Registration (lazy):** The GUI calls the callback set with `aon::gui.SetDataRegister(...)` when the Data screen opens (only if the internal `dataEntries` list is empty). Use that callback to call `RegisterDataEntry(...)` for each value you want displayed.
- **Display / page size:** Shows up to six (6) entries per page (constant `DATA_PER_PAGE = 6`). Each entry is displayed as `Name: value` with values formatted to three decimal places.
- **Pagination:** If more than one page is required, a `Page X/Y` indicator is shown and `PREV` / `NEXT` buttons allow navigation.
- **VARS button:** Opens the Variables menu. If no variables are present and a `SetVariableRegister` callback exists, the GUI will invoke it to populate the variables list.
- **RESET button:** Resets odometry and related sensor state. Reset actions include:
  - clearing/taring encoder positions,
  - zeroing internal encoder and gyro data structs,
  - setting `deltaTheta = 0.0` and zeroing local position accumulators (e.g. `deltaDlocal`, `changeWeb`), and
  - calling `aon::odometry::SetPosition(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y)` and `aon::odometry::SetDegrees(INITIAL_ODOMETRY_THETA)`.
  - If `GYRO_ENABLED` is defined, the gyroscope is tared as part of the reset.

### Using the RESET Button

The Data screen's `RESET` button invokes a user-registered reset handler (if one exists). The GUI provides a small API to register one or more named reset handlers; the active handler is invoked when the button is pressed.

How it works (implementation notes):
- The GUI calls `gui->InvokeResetHandler()` when the RESET button is tapped (see `src/aon/tools/gui/gui-debug/data.cpp`).
- On `GuiDebug`, call `RegisterResetHandler(name, callback)` to register a handler. The last-registered handler with the provided name becomes the active handler.
- On the base `Gui` (competition mode) these calls are no-ops, so register safely in all builds.

Quick tutorial — register a reset handler (example you can put in `initialize()` or your `data.cpp` registration file):

```cpp
// Example: register a reset handler that reinitializes odometry
// Put this in your initialization code (e.g. main.cpp or data.cpp)
aon::gui.RegisterResetHandler("ResetOdom", []{
  // Reset odometry to origin (example API; adapt to your odometry API)
  drivetrain.odom.resetCurrent(0.0, 0.0, 0.0);
  // Optionally tare encoders or gyro here as well:
  // LEFT_MOTORS->tare_position(); RIGHT_MOTORS->tare_position();
});

// If you register multiple handlers, calling RegisterResetHandler again
// with a different name will make that new handler the active one.
```

Notes and tips:
- Call `RegisterResetHandler(...)` before `aon::InitializeGui()` if you want the handler to be available immediately when the Data screen is opened.
- You can register handlers from any translation unit (main, a dedicated `data.cpp`, etc.). The GUI will invoke the active handler regardless of where it was registered.
- If you need multiple reset options, register them under different names; the GUI's debug UI currently uses the most recently-registered name as the active handler.
- For safety: ensure your reset handler runs quickly and avoids blocking the GUI thread (do minimal work or post to a background task if needed).

### Example: register data entries
```cpp
aon::gui.SetDataRegister([]{
  aon::gui.RegisterDataEntry("X", [](){ return drivetrain.odom.getX(); });
  aon::gui.RegisterDataEntry("Y", [](){ return drivetrain.odom.getY(); });
  aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
});
```

Tips:
- Call `SetDataRegister(...)` from `initialize()` (before `aon::InitializeGui()`) if you want the entries to be eagerly seeded at startup.
- Ensure each registered entry has a unique name; duplicate names are ignored.

### Notes
- Page size is fixed to 6 entries per page.
- Values are shown with three decimal digits for readability and compactness.

---

## Debug Mode Toggle

Debug mode is controlled by `TESTING_AUTONOMOUS` in [constants.hpp](../../constants.hpp):

```cpp
// In constants.hpp
#define TESTING_AUTONOMOUS true   // Uses GuiDebug - full debug features
#define TESTING_AUTONOMOUS false  // Uses Gui - competition mode (default)
```

### What Changes

| `TESTING_AUTONOMOUS` | GUI Type | Main Menu |
|---------------------|----------|------------|
| `true` | `GuiDebug` | Split button bar (AUTONS + DEBUG) |
| `false` | `Gui` | Full-width AUTONS button only |

### Use Cases

- **TESTING_AUTONOMOUS = true**: Development/testing with full debug menu access
- **TESTING_AUTONOMOUS = false**: Competition mode with only autonomous selection

---

## Complete Example

Here's a full setup showing all three features:

```cpp
// globals.hpp
namespace aon {
  inline double MAX_RPM_TEST = 100.0;
  inline double DRIVE_KP = 0.5;
  inline double DRIVE_KI = 0.02;
}

// main.cpp
void initialize() {
  // Set up tunable variables
  aon::gui.SetVariableRegister([]{  // Register BEFORE InitializeGui()
    aon::gui.VariableChanger(aon::MAX_RPM_TEST, "Max RPM Test");
    aon::gui.VariableChanger(aon::DRIVE_KP, "Drive kP");
    aon::gui.VariableChanger(aon::DRIVE_KI, "Drive kI");
  });
  
  // Set up test functions
  aon::gui.SetTestRegister([]{
    aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui.RegisterTestFunction(&BlueRingsRoutine, "Blue Rings");
    aon::gui.RegisterTestFunction(&SkillsRoutine, "Skills");
  });
  
  // Register data entries
  aon::gui.SetDataRegister([]{  
    aon::gui.RegisterDataEntry("X", [](){ return drivetrain.odom.getX(); });
    aon::gui.RegisterDataEntry("Y", [](){ return drivetrain.odom.getY(); });
    aon::gui.RegisterDataEntry("Heading", [](){ return drivetrain.odom.getDegrees(); });
  });
  
  // Set up live graph (odometry)
  aon::gui.SetGraphDataProviders(
    []() { return drivetrain.odom.getX(); },
    []() { return drivetrain.odom.getY(); }
  );

  aon::InitializeGui();
  pros::Task guiLoopTask([]{ aon::gui.RunLoop(); });
}

void opcontrol() {
  while (true) {
    // Your driver control code
    aon::operator_control::Run(aon::operator_control::DEFAULT);
    pros::delay(10);
  }
}
```

### Menu Flow

1. **Main Menu**: Shows selected auton, with "AUTONS" and "DEBUG" buttons
2. **Debug Menu**: Five options including "Registered Autons" and "Auton Runner"
3. **Registered Autons** (Debug 1): List of your registered test functions; tap to select and go to Auton Runner
4. **Auton Runner** (Debug 2): Execute selected auton — runs synchronously, GUI blocks until complete; quick VARS access
5. **Variables** (Debug 3): Live-adjustable parameters with +/- buttons
6. **Data** (Debug 4): Live display of registered data values with RESET and VARS buttons
7. **Live Graph**: Real-time X/Y plot with auto-scaling

---

## API Reference

### Initialization

| Function | Purpose |
|----------|---------|
| `aon::InitializeGui()` | Initialize the GUI system; call once in `initialize()` |
| `aon::gui.Initialize()` | Called internally by `InitializeGui()` |

### Test Function Registration

| Function | Purpose |
|----------|---------|
| `aon::gui.SetTestRegister(callback)` | Set callback to register test functions (called lazily when Debug Menu 1 opens) |
| `aon::gui.RegisterTestFunction(func, name)` | Register a test function with given display name |

**Supported signatures:**
- `int (*func)()` - Function pointer returning int
- `void (*func)()` - Function pointer returning void (wrapped to int)
- `std::function<int()>` or lambda - Any callable returning int

### Variable Registration

| Function | Purpose |
|----------|---------|
| `aon::gui.SetVariableRegister(callback)` | Set callback to register variables (called lazily when Debug Menu 3 opens) |
| `aon::gui.VariableChanger(var, name)` | Register a double variable for live adjustment |

### Graph Setup

| Function | Purpose |
|----------|---------|
| `aon::gui.SetGraphDataProviders(xFunc, yFunc)` | Set data provider callbacks for live graph |
| `aon::gui.AddGraphPoint(x, y)` | Manually add data point (usually called internally) |

### Data Registration

| Function | Purpose |
|----------|--------|
| `aon::gui.SetDataRegister(callback)` | Set callback to register data entries (called lazily when Data screen opens) |
| `aon::gui.RegisterDataEntry(name, getter)` | Register a named data entry with a `std::function<double()>` getter |

### Configuration

| Property | Type | Default | Purpose |
|----------|------|---------|---------|

| `aon::gui.selectedAuton` | `AutonOption` | `{ "None", nullptr }` | Currently selected auton (name + routine) |
| `aon::gui.selectedAutonName` | `std::string` | `"None"` | Display name of selected auton |
| `aon::gui.selectedAutonInvoker` | `std::function<int()>` | `nullptr` | Optional invoker used by debug-registered autons |
| `aon::gui.selectedRedAut` | `int` | `0` | Preselect Red auton (1-3, 0=none) |
| `aon::gui.selectedBlueAut` | `int` | `0` | Preselect Blue auton (1-3, 0=none) |
| `aon::gui.selectedSkill` | `int` | `0` | Preselect Skills auton (1-3, 0=none) |

---

## Troubleshooting

### Item not showing in menu?

1. **Check deduplication**: Ensure the name is unique (no duplicate names registered)
2. **Verify lazy loading**: Close and reopen the debug screen to force redraw
3. **Check registration timing**: Ensure `SetTestRegister()` or `SetVariableRegister()` is called before opening the menu
4. **Avoid loops**: Don't call registration inside tight loops; call once in `initialize()` or guarded by a `static bool`

### Data screen shows "No Data Registered"?

1. **Call `SetDataRegister` in `initialize()`**: Ensure `aon::gui.SetDataRegister(...)` is called inside `initialize()` **before** `aon::InitializeGui()` so entries are eagerly seeded during initialization
2. **Use `RegisterDataEntry` inside the callback**: Entries must be registered inside the lambda passed to `SetDataRegister`, not outside it
3. **Check debug mode**: Data screen only works with `GuiDebug` (`TESTING_AUTONOMOUS` must be `true` in [constants.hpp](../../constants.hpp))
4. **Rebuild**: After changing code, rebuild with `pros build`

### Graph not updating?

1. Verify `SetGraphDataProviders()` was called
2. Check that the lambda functions return valid double values
3. Ensure you opened the **Live Graph** screen (not just Debug Menu)
4. Check that data providers aren't returning NaN or infinity

### Auton won't run from Auton Runner?

1. Ensure you selected an auton from **Registered Autons** or **AUTONS** menu first
2. The RUN button should be **green**—if it's gray, no auton is selected
3. Check that the selected function returns 0 (success)
4. Verify the function is registered with `RegisterTestFunction()`

### Auton Runner shows MOV but won't stop the auton?

This is expected behavior. Execution is **synchronous** — `ExecuteFunction` blocks the GUI loop task until the routine returns. The **MOV** label is a visual indicator of running state, not a functional stop button. The only way to stop an auton is to let it run to completion or implement a timeout inside the routine itself.

### Variable adjustments not persisting?

1. Ensure variables are passed **by reference** to `VariableChanger()`
2. Check that the variable scope is correct (use global or static variables)
3. Verify the name is unique in the variable list

---

## Notes & Tips

- **Keep registrations in callbacks**: Use `SetTestRegister()`, `SetVariableRegister()`, and `SetDataRegister()` callbacks to keep registration logic clean. Call all `Set*Register` functions **before** `InitializeGui()` so entries are eagerly seeded during initialization.
- **Use global/static variables**: For variables to persist, declare them at global or static scope
- **Guard against multiple registration**: The GUI deduplicates by name, so duplicate registrations are safe but unnecessary
- **Preselect autons**: You can set `selectedRedAut`, `selectedBlueAut`, or `selectedSkill` (1-3) before `Initialize()` to preselect an auton
- **Competition mode**: Set `TESTING_AUTONOMOUS` to `false` in constants.hpp to use `Gui gui` (no debug menu)

---

## Example: Full Workflow

1. **Initialize** (in `initialize()`)
   ```cpp
   // Set registers BEFORE InitializeGui
   aon::gui.SetTestRegister([]{
     aon::gui.RegisterTestFunction(&RedRingsRoutine, "Red Rings");
   });
   aon::gui.SetVariableRegister([]{
     aon::gui.VariableChanger(aon::DRIVE_KP, "Drive kP");
   });
   aon::gui.SetDataRegister([]{
    aon::gui.RegisterDataEntry("X", [](){ return drivetrain.odom.getX(); });
   });
   aon::InitializeGui();
   ```

2. **Main Menu** appears showing "NO AUTON" with "AUTONS" and "DEBUG" buttons

3. **User taps "DEBUG"** → Debug Menu appears

4. **User taps "Registered Autons"** → Your callback is called; "Red Rings" appears in the list

5. **User taps "Red Rings"** → Automatically navigates to **Auton Runner** with "Red Rings" selected

6. **User taps green "RUN"** → Executes `RedRingsRoutine()` **synchronously** in the GUI loop task
   - Button shows red "MOV" label (visual only — GUI is blocked)
   - Status shows orange "Red Rings"

7. **Routine completes** → Status shows cyan "COMPLETED"; RUN button turns green

8. **User taps "VARS"** → Variables menu opens; adjust `Drive kP` value

9. **User taps "BACK"** → Returns to Auton Runner; run again with new values

---
- [gui.hpp](gui.hpp) - Base GUI class
- [gui-debug.hpp](gui-debug.hpp) - Debug GUI class with full API
