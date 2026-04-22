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
std::unique_ptr<Gui> gui = std::make_unique<GuiDebug>();
#else
std::unique_ptr<Gui> gui = std::make_unique<Gui>();
#endif
```

`aon::gui` is a `std::unique_ptr<Gui>` — all method calls use `->`.

| `TESTING_AUTONOMOUS` | GUI Type | Main Menu |
|---|---|---|
| `true` | `GuiDebug` | Split bar: **AUTONS** + **DEBUG** |
| `false` | `Gui` | Full-width **AUTONS** only |

**Note:** The base `Gui` class provides no-op virtual methods for all debug APIs (`variableChanger`, `registerTestFunction`, `registerDataEntry`, etc.) so registration calls compile in both modes. Runtime behavior is only active under `GuiDebug`.

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
AutonOption redAutonOptions[autonOptionsCount] = {
  {"Red AUT1", aon::MyNewRedRoutine},
  {"Red AUT2", aon::RedRoutine},
  {"Red AUT3", aon::RedRoutine},
};
```

`autonOptionsCount` is 3 — don't change it unless you resize all three arrays (`redAutonOptions`, `blueAutonOptions`, `skillsAutonOptions`) together.

### 3. Rebuild

```bash
pros build
```

Navigate to **AUTONS** on the brain to confirm.

> For dynamically registered routines, use **Debug Menu → Registered Autons** instead — see [Registering Test Functions](#registering-test-functions).

---

## Quick Start

Call all `set*Register` functions **before** starting the GUI task.

```cpp
void initialize() {
  aon::gui->setTestRegister([]{
    aon::gui->registerTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui->registerTestFunction(&BlueRingsRoutine, "Blue Rings");
  });

  aon::gui->setVariableRegister([]{
    aon::gui->variableChanger(DRIVE_KP, "Drive kP");
  });

  aon::gui->setDataRegister([]{
    aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
    aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
    aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
  });

  aon::gui->setGraphDataProviders(
    []() { return drivetrain.getX(); },
    []() { return drivetrain.getY(); }
  );

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}
```

---

## Registering Test Functions

Test functions appear in **Debug Menu → Registered Autons** and can be selected and executed from the Auton Runner.

### API

```cpp
// int-returning function pointer
aon::gui->registerTestFunction(&YourFunction, "Display Name");

// void-returning function pointer (wrapped automatically)
aon::gui->registerTestFunction(&YourVoidFunction, "Display Name");

// Lambda or std::function
aon::gui->registerTestFunction([]() -> int {
  pros::delay(1000);
  return 0;
}, "Display Name");
```

Wrap registrations in a `setTestRegister` callback:

```cpp
aon::gui->setTestRegister([]{
  aon::gui->registerTestFunction(&RedRingsRoutine, "Red Rings");
  aon::gui->registerTestFunction(&BlueRingsRoutine, "Blue Rings");
});
```

### How It Works

1. `setTestRegister(callback)` stores the callback.
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

1. The selected function is called **synchronously** in the GUI loop task.
2. The screen redraws to show the auton name in **orange** before execution begins.
3. The GUI is completely blocked — touch input is not processed while the routine runs.
4. When the routine returns, the screen updates to **cyan "COMPLETED"**.

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
aon::gui->variableChanger(variableRef, "Display Name");
```

Wrap registrations in a `setVariableRegister` callback:

```cpp
aon::gui->setVariableRegister([]{
  aon::gui->variableChanger(DRIVE_KP,  "Drive kP");
  aon::gui->variableChanger(DRIVE_KI,  "Drive kI");
  aon::gui->variableChanger(MAX_SPEED, "Max Speed");
});
```

### How It Works

1. `setVariableRegister(callback)` stores the callback.
2. When **Variables** opens, the callback is invoked once.
3. Each entry shows six buttons: `-10`, `-1`, `-0.1` | `+0.1`, `+1`, `+10`.
4. Tapping a button immediately applies the delta to the variable in memory.
5. Up to 2 variables per page; use **PREV**/**NEXT** to paginate.

### Variable Declaration — Important

Variables must be declared as `inline double` at **global scope** in [globals.hpp](../../globals.hpp) — **not** inside a function or routine. This lets both the GUI registration and the autonomous routine access the same variable by reference.

```cpp
// globals.hpp — declare inline doubles at global scope
inline double INCHES  = -138.0;
inline double ANGLE   =   13.0;
```

They are then **used inside the autonomous routine** so that adjusting them in the Variables menu changes what the routine does on the next run:

```cpp
// autonomous-routines.hpp
int MyRoutine() {
  drivetrain.moveForward(INCHES);
  drivetrain.turnDegrees(ANGLE);
  return 0;
}
```

Registration in `initialize()`:

```cpp
aon::gui->setVariableRegister([]{
  aon::gui->variableChanger(INCHES, "Distance");
  aon::gui->variableChanger(ANGLE,  "Angle");
});
```

Workflow: adjust **Distance** / **Angle** in the Variables menu → press **BACK** → press **RUN** in Auton Runner → observe result → repeat.

### Notes

- Pass variables **by reference** — the GUI modifies the actual variable.
- Declare as `inline double` at global scope in `globals.hpp` so they are accessible from both the registration callback and the routine.
- **Do not declare tunable variables as local variables inside a routine** — local variables go out of scope and the `variableChanger` reference will dangle.
- Duplicate names are silently ignored.

---

## Data Menu

The Data screen (**Debug Menu → Data**) shows live numeric values with VARS and RESET shortcuts.

### API

```cpp
aon::gui->setDataRegister([]{
  aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
  aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
  aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
});
```

### How It Works

1. `setDataRegister(callback)` stores the callback.
2. When the Data screen opens, the callback is invoked once.
3. Each entry displays as `Name: value` with three decimal places.
4. Up to 6 entries per page; use **PREV**/**NEXT** for pagination.
5. **VARS** button opens the Variables menu.
6. **RESET** button invokes the registered reset handler.

### RESET Handler

```cpp
aon::gui->registerResetHandler("ResetOdom", []{
  drivetrain.resetPose(0.0, 0.0, 0.0);
});
```

- The most recently registered handler is the active one.
- No-op on base `Gui`; safe to call in all builds.
- Register before starting the GUI task to have it available immediately.

---

## Live Graph

The Live Graph (**Debug Menu → Live Graph**) plots real-time X/Y data with auto-scaling axes.

### API

```cpp
aon::gui->setGraphDataProviders(
  []() -> double { return /* X */; },
  []() -> double { return /* Y */; }
);
```

### How It Works

1. `setGraphDataProviders(xFunc, yFunc)` stores the two callbacks.
2. While the Live Graph screen is open, a new point is sampled every ~300 ms.
3. Up to 300 points are kept in a circular buffer; oldest data is dropped as new arrives.
4. Axes auto-scale around the data range.
5. Current X/Y values are shown in the corner.

### Examples

```cpp
// Odometry path
aon::gui->setGraphDataProviders(
  []() { return drivetrain.getX(); },
  []() { return drivetrain.getY(); }
);

// Time vs motor velocity
aon::gui->setGraphDataProviders(
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
- Buffer holds up to 600 points; once full, new points are dropped until **CLEAR** is pressed.
- Arc results use `DRIVE_WIDTH` from `constants.hpp` for Inner/Out values, so they automatically reflect whichever robot is selected via `USING_BIG_ROBOT`.

---

## Complete Example

```cpp
// globals.hpp — inline doubles accessible from both the GUI and routines
inline double INCHES  = -138.0;
inline double ANGLE   =   13.0;

// autonomous-routines.hpp — routine reads the globals so changes take effect
int MyRoutine() {
  drivetrain.moveForward(INCHES);
  drivetrain.turnDegrees(ANGLE);
  return 0;
}

// main.cpp
void initialize() {
  aon::gui->setTestRegister([]{
    aon::gui->registerTestFunction(&MyRoutine,      "My Routine");
    aon::gui->registerTestFunction(&RedRingsRoutine, "Red Rings");
  });

  aon::gui->setVariableRegister([]{
    aon::gui->variableChanger(INCHES, "Distance");
    aon::gui->variableChanger(ANGLE,  "Angle");
  });

  aon::gui->setDataRegister([]{
    aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
    aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
    aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
  });

  aon::gui->setGraphDataProviders(
    []() { return drivetrain.getX(); },
    []() { return drivetrain.getY(); }
  );

  aon::gui->setMapDataProvider([]() -> aon::Pose {
    return {drivetrain.getX(), drivetrain.getY(),
            drivetrain.getTheta() * M_PI / 180.0};
  });

  aon::gui.RegisterResetHandler("ResetOdom", []{
    drivetrain.odom.resetCurrent(0.0, 0.0, 0.0);
  });

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
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
| `pros::Task guiLoopTask([]{ aon::gui->initialize(); })` | Start the GUI; call once in `initialize()` |

### Test Function Registration

| Function | Description |
|----------|-------------|
| `aon::gui->setTestRegister(callback)` | Store the callback (invoked lazily when Registered Autons opens) |
| `aon::gui->registerTestFunction(func, name)` | Register a function with a display name |

Supported signatures: `int(*)()`, `void(*)()`, `std::function<int()>`, lambda.

### Variable Registration

| Function | Description |
|----------|-------------|
| `aon::gui->setVariableRegister(callback)` | Store the callback (invoked lazily when Variables opens) |
| `aon::gui->variableChanger(var, name)` | Register an `inline double` global for live +/− adjustment |

### Data Registration

| Function | Description |
|----------|-------------|
| `aon::gui->setDataRegister(callback)` | Store the callback (invoked lazily when Data opens) |
| `aon::gui->registerDataEntry(name, getter)` | Register a `std::function<double()>` getter with a display name |
| `aon::gui->registerResetHandler(name, cb)` | Register a named reset handler; last registered is active |

### Graph

| Function | Description |
|----------|-------------|
| `aon::gui->setGraphDataProviders(xFunc, yFunc)` | Set X and Y data provider callbacks |

### Field Mapper

| Function | Description |
|----------|-------------|
| `aon::gui->setMapDataProvider(poseFunc)` | Set a `std::function<Pose()>` callback; `Pose.theta` must be in **radians** |

### State Properties

Access via `aon::gui->propertyName` (unique_ptr dereference).

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `selectedAuton` | `AutonOption` | `{"None", nullptr}` | Currently selected preset auton |
| `selectedAutonName` | `std::string` | `"None"` | Display name of selected auton |
| `selectedAutonInvoker` | `std::function<int()>` | `nullptr` | Invoker for debug-registered autons |
| `selectedRedAut` | `int` | `0` | Preselect Red auton index (1–3, 0=none) |
| `selectedBlueAut` | `int` | `0` | Preselect Blue auton index (1–3, 0=none) |
| `selectedSkill` | `int` | `0` | Preselect Skills auton index (1–3, 0=none) |

To preselect an auton before the GUI starts (skips manual selection—useful during testing):

```cpp
aon::gui->selectedRedAut = 1;  // preselect Red AUT1 — jump straight to Auton Runner
pros::Task guiLoopTask([]{ aon::gui->initialize(); });
```

---

## Troubleshooting

### Function not appearing in Registered Autons?

1. Name must be unique — duplicates are silently ignored.
2. Verify `setTestRegister()` is called before the GUI task starts.
3. Close and reopen the Registered Autons screen to re-trigger the callback.

### Auton won't run from Auton Runner?

1. Select an auton from **Registered Autons** or **AUTONS** first — RUN is gray until one is selected.
2. Confirm the function is registered with `registerTestFunction()`.

### The GUI freezes when I press RUN — is that a bug?

No. The Auton Runner calls the selected function **synchronously** in the GUI loop task. The screen intentionally shows orange while blocked and cyan when the routine returns. There is no mechanism to stop or interrupt a running auton — it always runs to completion.

### Data screen shows "No Data Registered"?

1. Call `setDataRegister(...)` before the GUI task starts.
2. Confirm `TESTING_AUTONOMOUS` is `true` — the Data screen is only active on `GuiDebug`.
3. Rebuild with `pros build`.

### Graph not updating?

1. Confirm `setGraphDataProviders()` was called before the GUI task starts.
2. Verify the lambdas return valid `double` values (not NaN/infinity).
3. You must be on the **Live Graph** screen — data is only sampled while that screen is active.

### Variable changes not persisting?

1. Variables must be declared as `inline double` at **global scope** in `globals.hpp`.
2. Pass them **by reference** to `variableChanger()`.
3. Do not declare tunable variables as local variables — they will go out of scope.

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
std::unique_ptr<Gui> gui = std::make_unique<GuiDebug>();
#else
std::unique_ptr<Gui> gui = std::make_unique<Gui>();
#endif
```

`aon::gui` is a `std::unique_ptr<Gui>` — use `aon::gui->method()` everywhere.

After changing `TESTING_AUTONOMOUS`, rebuild your project to pick up the selected GUI behavior.

**Note about debug-only addons:** The base `Gui` class provides no-op virtual methods for most debug APIs so user code that calls `variableChanger`, `registerTestFunction`, `registerDataEntry`, etc., will compile and link even when `Gui` (non-debug) is in use. The full runtime behavior (variables list, registered autons, data screens, live graph) is implemented only in `GuiDebug`.

Options:
- Enable the debug GUI by setting `TESTING_AUTONOMOUS` to `true` in `include/aon/constants.hpp` to get full debug features at runtime.
- Or keep `TESTING_AUTONOMOUS = false` (competition mode) and treat debug registrations as no-ops (they won't populate debug screens at runtime).

### [Registering Test Functions](#registering-test-functions)
```cpp
aon::gui->setTestRegister([]{
  aon::gui->registerTestFunction(&MyFunction, "My Test");
});
```

### [Auton Runner](#auton-runner)
Access via: **DEBUG Menu → Auton Runner** (or select a test from Registered Autons)
- Green **RUN** button: Start the selected auton
- Red **MOV** label: Shown while auton is running — execution is **synchronous** so no input is received; the GUI is blocked until the routine returns
- **VARS** button: Quick access to tunable variables

### [Tunable Variables](#tunable-variables)
```cpp
aon::gui->setVariableRegister([]{
  aon::gui->variableChanger(myVariable, "Variable Name");
});
```

### [Data Menu](#data-menu)
```cpp
aon::gui->setDataRegister([]{  
  aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
  aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
  aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
});
```

### [Live Graph](#live-graph)
```cpp
aon::gui->setGraphDataProviders(
  []() { return drivetrain.getX(); },
  []() { return drivetrain.getY(); }
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

Note: when adding a new preset auton, also add a forward declaration for the function in the `aon` namespace at the top of `gui.hpp` so the GUI can reference it (for example: `int ForwardBackTurnRoutine();`). Then implement the function in [autonomous-routines.hpp](../../competition/autonomous-routines.hpp). Ensure the function is in the `aon::routines` namespace and returns an `int`.

```cpp
// Forward declarations at top of gui.hpp (inside namespace aon)
namespace aon {

  namespace routines {
    int RedRingsRoutine();
    int BlueRingsRoutine();
    int ForwardBackTurnRoutine();
    // Add other auton routine declarations as needed
  }


// Auton option arrays (instance members of the Gui class — no static/inline)
AutonOption redAutonOptions[autonOptionsCount] = {
  {"Red ForwardBackTurn", aon::ForwardBackTurnRoutine},
  {"Red AUT2", aon::RedRoutine},
  {"Red AUT3", aon::RedRoutine},
};

AutonOption blueAutonOptions[autonOptionsCount] = {
  {"Blue AUT1", aon::BlueRoutine},
  {"Blue AUT2", aon::BlueRoutine},
  {"Blue AUT3", aon::BlueRoutine},
};

AutonOption skillsAutonOptions[autonOptionsCount] = {
  {"Skills AUT1", aon::RedRoutine},
  {"Skills AUT2", aon::RedRoutine},
  {"Skills AUT3", aon::RedRoutine},
};
```

**To change an auton:**

1. Replace the function pointer with your function name
2. Optionally change the display name

Example:
```cpp
AutonOption redAutonOptions[autonOptionsCount] = {
  {"Red AUT1", aon::MyNewRedRoutine},      // Changed function
  {"Red AUT2", aon::RedRoutine},
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
- **Naming style**: Functions should be in the `aon::routines::` namespace and return `int` 
- **Array size**: Don't change `AutonOptionsCount` (set to 3) unless you also change all three arrays
- **Only for preset autons**: If you want to dynamically register autons, use the Debug Menu's **Registered Autons** feature instead (see [Registering Test Functions](#registering-test-functions))

---

## Quick Start

### Basic Setup (in `initialize()`)

**Important:** Call all `set*Register` functions **before** starting the GUI task so entries are eagerly seeded during initialization.

```cpp
void initialize() {
  // Register ALL callbacks BEFORE starting the GUI task

  // Register variables (optional)
  aon::gui->setVariableRegister([]{  
    aon::gui->variableChanger(INCHES, "Distance");
    aon::gui->variableChanger(ANGLE,  "Angle");
  });
  
  // Register test functions (optional)
  aon::gui->setTestRegister([]{  
    aon::gui->registerTestFunction(&RedRingsRoutine,  "Red Rings");
    aon::gui->registerTestFunction(&BlueRingsRoutine, "Blue Rings");
  });
  
  // Register data entries (optional)
  aon::gui->setDataRegister([]{  
    aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
    aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
    aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
  });
  
  // Set up live graph (optional)
  aon::gui->setGraphDataProviders(
    []() { return drivetrain.getX(); },
    []() { return drivetrain.getY(); }
  );
  
  // NOW start the GUI task (initializes and runs the loop)
  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}
```

Note: the initialization screen renders the primary and secondary messages with a typewriter-style animation (see `src/aon/tools/gui/gui.cpp`).

---

## Registering Test Functions

Test functions appear in **Debug Menu 1: "Registered Autons"** and allow you to select and run autonomous routines.

### API

```cpp
// Register with int-returning function pointer
aon::gui->registerTestFunction(&YourAutonFunction, "Display Name");

// Register with void-returning function pointer
aon::gui->registerTestFunction(&YourVoidFunction, "Display Name");

// Register with std::function or lambda
aon::gui->registerTestFunction([]() -> int {
  // your code
  return 0;
}, "Display Name");
```

### How It Works

1. Call `setTestRegister(callback)` with a callback containing your `registerTestFunction()` calls
2. When you open **Debug Menu → Registered Autons**, the callback is invoked once to populate the list
3. Tap a function to select it
4. Go to **Debug Menu 2 (Auton Runner)** and press **RUN**
5. The auton executes **synchronously** in the GUI loop task; the GUI is blocked until the routine returns.

### Example

```cpp
void initialize() {
  aon::gui->setTestRegister([]{
    aon::gui->registerTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui->registerTestFunction(&BlueRingsRoutine, "Blue Rings");
    aon::gui->registerTestFunction([]() -> int {
      // Inline test
      return 0;
    }, "Quick Test");
  });

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}
```

### Notes

- **Deduplication**: Duplicate names are ignored; later registrations with the same name won't be added
- **Lazy Loading**: The register callback is called when Debug Menu 1 opens, not at startup
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

> **Testing tip**: Set `aon::gui->selectedRedAut = 1;` (or whichever index) before `pros::Task guiLoopTask(...)` to have a preset auton already selected on boot — skip step 1 entirely.

1. **Select an Auton** *(or skip with preselect above)*:
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
- Adjust parameters live
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
aon::gui->variableChanger(variableRef, "Display Name");
```

### Variable Declaration — Important

Variables must be declared as `inline double` at **global scope** in [globals.hpp](../../globals.hpp) — **not** inside a function or routine. This lets the GUI hold a reference that remains valid for the entire program lifetime.

```cpp
// globals.hpp
inline double INCHES  = -138.0;
inline double ANGLE   =   13.0;
```

They are **used inside the autonomous routine** so that adjusting them in the Variables menu changes what the routine does on the next run:

```cpp
// autonomous-routines.hpp
int MyRoutine() {
  drivetrain.moveForward(INCHES);
  drivetrain.turnDegrees(ANGLE);
  return 0;
}
```

### How It Works

1. Call `setVariableRegister(callback)` with a callback containing your `variableChanger()` calls
2. When you open **Debug Menu 3**, the callback is invoked to populate the list
3. Each variable shows with six buttons:
   - `-10`, `-1`, `-0.1` (decrement)
   - `+0.1`, `+1`, `+10` (increment)
4. Tap buttons to adjust the variable live
5. Changes persist while the program runs

### Example

```cpp
// globals.hpp
inline double INCHES  = -138.0;
inline double ANGLE   =   13.0;

// main.cpp
void initialize() {
  aon::gui->setVariableRegister([]{
    aon::gui->variableChanger(INCHES, "Distance");
    aon::gui->variableChanger(ANGLE,  "Angle");
  });

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}
```

### Notes

- **Reference Required**: Pass variables by reference; the GUI adjusts the actual variable
- **Global scope required**: Declare as `inline double` in `globals.hpp` — do not use local variables
- **Use in routine**: The variable must be read inside the autonomous routine for adjustments to have any effect
- **Deduplication**: Duplicate names are ignored
- **Lazy Loading**: The register callback is called when Debug Menu 3 opens
- **Precision**: Adjustments use double precision; increment/decrement values are fixed (+/-0.1, +/-1, +/-10)

---

## Live Graph

The **Live Graph** screen (Debug Menu → "Live Graph") displays real-time X/Y data with auto-scaling axes.

### API

```cpp
aon::gui->setGraphDataProviders(
  []() -> double { return /* X data */ },
  []() -> double { return /* Y data */ }
);
```

### How It Works

1. Call `setGraphDataProviders(xCallback, yCallback)` with lambdas returning double values
2. Open **Debug Menu → Live Graph**
3. The graph displays points as they're collected (300-point circular buffer)
4. Axes auto-scale with 10% padding
5. Current X/Y values shown in corner
6. Refreshes every ~300ms

### Example

```cpp
void initialize() {
  // Graph odometry (X/Y position)
  aon::gui->setGraphDataProviders(
    []() { return drivetrain.getX(); },
    []() { return drivetrain.getY(); }
  );

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}
```

### Advanced Example (Custom Data)

```cpp
void initialize() {
  // Graph time vs motor velocity
  aon::gui->setGraphDataProviders(
    []() { return (double)pros::millis() / 1000.0; },  // Time in seconds
    []() { return LEFT_MOTORS->get_velocity(); }       // Motor velocity
  );

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}
```

### Notes

- **Any Data Source**: Use any function returning double—not limited to odometry
- **Circular Buffer**: Stores 300 most recent points; oldest data is discarded as new arrives
- **Auto-Scale**: Y-axis scales automatically; toggle via the display
- **Real-Time**: Updates every ~300ms during operation

## Data Menu

The Data screen (Debug Menu → "Data") shows live numeric values registered by user code and provides quick access to variables and reset controls.

- **Access:** Debug Menu → Data
- **Registration (lazy):** The GUI calls the callback set with `aon::gui->setDataRegister(...)` when the Data screen opens (only if the internal `dataEntries` list is empty). Use that callback to call `registerDataEntry(...)` for each value you want displayed.
- **Display / page size:** Shows up to six (6) entries per page (constant `DATA_PER_PAGE = 6`). Each entry is displayed as `Name: value` with values formatted to three decimal places.
- **Pagination:** If more than one page is required, a `Page X/Y` indicator is shown and `PREV` / `NEXT` buttons allow navigation.
- **VARS button:** Opens the Variables menu. If no variables are present and a `setVariableRegister` callback exists, the GUI will invoke it to populate the variables list.
- **RESET button:** Invokes the registered reset handler.

### Using the RESET Button

The Data screen's `RESET` button invokes a user-registered reset handler (if one exists). The GUI provides a small API to register one or more named reset handlers; the active handler is invoked when the button is pressed.

How it works (implementation notes):
- The GUI calls `gui->invokeResetHandler()` when the RESET button is tapped.
- On `GuiDebug`, call `registerResetHandler(name, callback)` to register a handler. The last-registered handler with the provided name becomes the active handler.
- On the base `Gui` (competition mode) these calls are no-ops, so register safely in all builds.

Quick tutorial — register a reset handler (example you can put in `initialize()`):

```cpp
aon::gui->registerResetHandler("Reset", []{
  drivetrain.resetPose(0.0, 0.0, 0.0);
});

// If you register multiple handlers, calling registerResetHandler again
// with a different name will make that new handler the active one.
```

Notes and tips:
- Call `registerResetHandler(...)` before starting the GUI task if you want the handler to be available immediately when the Data screen is opened.
- You can register handlers from any translation unit (main, a dedicated `data.cpp`, etc.). The GUI will invoke the active handler regardless of where it was registered.
- If you need multiple reset options, register them under different names; the GUI's debug UI currently uses the most recently-registered name as the active handler.
- For safety: ensure your reset handler runs quickly and avoids blocking the GUI thread (do minimal work or post to a background task if needed).

### Example: register data entries
```cpp
aon::gui->setDataRegister([]{
  aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
  aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
  aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
});
```

Tips:
- Call `setDataRegister(...)` from `initialize()` (before starting the GUI task) if you want the entries to be eagerly seeded at startup.
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

Here's a full setup showing all features:

```cpp
// globals.hpp — inline doubles at global scope, used by both GUI and routine
inline double INCHES      = -138.0;
inline double ANGLE       =   13.0;
inline double TWOINCHES   =  -15.0;
inline double TWOANGLE    = -110.0;

// autonomous-routines.hpp — routine reads the globals
int MyRoutine() {
  drivetrain.moveForward(INCHES);
  drivetrain.turnDegrees(ANGLE);
  drivetrain.moveForward(TWOINCHES);
  drivetrain.turnDegrees(TWOANGLE);
  return 0;
}

// main.cpp
void initialize() {
  // Set up tunable variables — register BEFORE starting the GUI task
  aon::gui->setVariableRegister([]{
    aon::gui->variableChanger(INCHES,    "Distance");
    aon::gui->variableChanger(ANGLE,     "Angle");
    aon::gui->variableChanger(TWOINCHES, "Second Distance");
    aon::gui->variableChanger(TWOANGLE,  "Second Angle");
  });
  
  // Set up test functions
  aon::gui->setTestRegister([]{
    aon::gui->registerTestFunction(&MyRoutine,       "My Routine");
    aon::gui->registerTestFunction(&RedRingsRoutine, "Red Rings");
    aon::gui->registerTestFunction(&SkillsRoutine,   "Skills");
  });
  
  // Register data entries
  aon::gui->setDataRegister([]{  
    aon::gui->registerDataEntry("X",       []{ return drivetrain.getX(); });
    aon::gui->registerDataEntry("Y",       []{ return drivetrain.getY(); });
    aon::gui->registerDataEntry("Heading", []{ return drivetrain.getTheta(); });
  });
  
  // Set up live graph
  aon::gui->setGraphDataProviders(
    []() { return drivetrain.getX(); },
    []() { return drivetrain.getY(); }
  );

  aon::gui->registerResetHandler("Reset", []{
    drivetrain.resetPose(0.0, 0.0, 0.0);
  });

  pros::Task guiLoopTask([]{ aon::gui->initialize(); });
}

void opcontrol() {
  while (true) {
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
5. **Variables** (Debug 3): Live-adjustable inline double globals with +/- buttons
6. **Data** (Debug 4): Live display of registered data values with RESET and VARS buttons
7. **Live Graph**: Real-time X/Y plot with auto-scaling

---

## API Reference

### Initialization

| Function | Purpose |
|----------|---------|
| `pros::Task guiLoopTask([]{ aon::gui->initialize(); })` | Start the GUI task; call once in `initialize()` |

### Test Function Registration

| Function | Purpose |
|----------|---------|
| `aon::gui->setTestRegister(callback)` | Set callback to register test functions (called lazily when Debug Menu 1 opens) |
| `aon::gui->registerTestFunction(func, name)` | Register a test function with given display name |

**Supported signatures:**
- `int (*func)()` - Function pointer returning int
- `void (*func)()` - Function pointer returning void (wrapped to int)
- `std::function<int()>` or lambda - Any callable returning int

### Variable Registration

| Function | Purpose |
|----------|---------|
| `aon::gui->setVariableRegister(callback)` | Set callback to register variables (called lazily when Debug Menu 3 opens) |
| `aon::gui->variableChanger(var, name)` | Register an `inline double` global for live adjustment |

### Graph Setup

| Function | Purpose |
|----------|---------|
| `aon::gui->setGraphDataProviders(xFunc, yFunc)` | Set data provider callbacks for live graph |
| `aon::gui->AddGraphPoint(x, y)` | Manually add data point (usually called internally) |

### Data Registration

| Function | Purpose |
|----------|--------|
| `aon::gui->setDataRegister(callback)` | Set callback to register data entries (called lazily when Data screen opens) |
| `aon::gui->registerDataEntry(name, getter)` | Register a named data entry with a `std::function<double()>` getter |

### Configuration

| Property | Type | Default | Purpose |
|----------|------|---------|---------|
| `aon::gui->selectedAuton` | `AutonOption` | `{ "None", nullptr }` | Currently selected auton (name + routine) |
| `aon::gui->selectedAutonName` | `std::string` | `"None"` | Display name of selected auton |
| `aon::gui->selectedAutonInvoker` | `std::function<int()>` | `nullptr` | Optional invoker used by debug-registered autons |
| `aon::gui->selectedRedAut` | `int` | `0` | Preselect Red auton (1-3, 0=none) |
| `aon::gui->selectedBlueAut` | `int` | `0` | Preselect Blue auton (1-3, 0=none) |
| `aon::gui->selectedSkill` | `int` | `0` | Preselect Skills auton (1-3, 0=none) |

---

## Troubleshooting

### Item not showing in menu?

1. **Check deduplication**: Ensure the name is unique (no duplicate names registered)
2. **Verify lazy loading**: Close and reopen the debug screen to force redraw
3. **Check registration timing**: Ensure `setTestRegister()` or `setVariableRegister()` is called before opening the menu
4. **Avoid loops**: Don't call registration inside tight loops; call once in `initialize()` or guarded by a `static bool`

### Data screen shows "No Data Registered"?

1. **Call `setDataRegister` in `initialize()`**: Ensure `aon::gui->setDataRegister(...)` is called inside `initialize()` **before** starting the GUI task so entries are eagerly seeded during initialization
2. **Use `registerDataEntry` inside the callback**: Entries must be registered inside the lambda passed to `setDataRegister`, not outside it
3. **Check debug mode**: Data screen only works with `GuiDebug` (`TESTING_AUTONOMOUS` must be `true` in [constants.hpp](../../constants.hpp))
4. **Rebuild**: After changing code, rebuild with `pros build`

### Graph not updating?

1. Verify `setGraphDataProviders()` was called
2. Check that the lambda functions return valid double values
3. Ensure you opened the **Live Graph** screen (not just Debug Menu)
4. Check that data providers aren't returning NaN or infinity

### Auton won't run from Auton Runner?

1. Ensure you selected an auton from **Registered Autons** or **AUTONS** menu first
2. The RUN button should be **green**—if it's gray, no auton is selected
3. Check that the selected function returns 0 (success)
4. Verify the function is registered with `registerTestFunction()`

### Auton Runner shows MOV but won't stop the auton?

This is expected behavior. Execution is **synchronous** — the selected function blocks the GUI loop task until it returns. The **MOV** label is a visual indicator of running state, not a functional stop button. The only way to stop an auton is to let it run to completion or implement a timeout inside the routine itself.

### Variable adjustments not persisting?

1. Ensure variables are declared as `inline double` at **global scope** in `globals.hpp`
2. Make sure they are passed **by reference** to `variableChanger()` — the GUI modifies the original value
3. Make sure the variable is **read inside the routine** (not just declared globally)
4. Verify the name is unique in the variable list

---

## Notes & Tips

- **Keep registrations in callbacks**: Use `setTestRegister()`, `setVariableRegister()`, and `setDataRegister()` callbacks to keep registration logic clean. Call all `set*Register` functions **before** starting the GUI task.
- **Use `inline double` globals**: For variables to be tunable, declare them as `inline double` at global scope in `globals.hpp` and use them inside the autonomous routine.
- **Guard against multiple registration**: The GUI deduplicates by name, so duplicate registrations are safe but unnecessary
- **Preselect autons**: Set `aon::gui->selectedRedAut`, `aon::gui->selectedBlueAut`, or `aon::gui->selectedSkill` (1–3) before starting the GUI task. Default is `0` (none). Set to `1` during testing to have Red AUT1 already selected on boot, skipping manual selection.
- **Competition mode**: Set `TESTING_AUTONOMOUS` to `false` in constants.hpp to disable the debug menu

---

## Example: Full Workflow

1. **Initialize** (in `initialize()`)
   ```cpp
   // Set registers BEFORE starting the GUI task
   aon::gui->setTestRegister([]{
     aon::gui->registerTestFunction(&RedRingsRoutine, "Red Rings");
   });
   aon::gui->setVariableRegister([]{
     aon::gui->variableChanger(INCHES, "Distance");
   });
   aon::gui->setDataRegister([]{
     aon::gui->registerDataEntry("X", []{ return drivetrain.getX(); });
   });
   pros::Task guiLoopTask([]{ aon::gui->initialize(); });
   ```

2. **Main Menu** appears showing selected auton with "AUTONS" and "DEBUG" buttons

3. **User taps "DEBUG"** → Debug Menu appears

4. **User taps "Registered Autons"** → Your callback is called; "Red Rings" appears in the list

5. **User taps "Red Rings"** → Automatically navigates to **Auton Runner** with "Red Rings" selected

6. **User taps green "RUN"** → Executes `RedRingsRoutine()` **synchronously** in the GUI loop task
   - Button shows red "MOV" label (visual only — GUI is blocked)
   - Status shows orange "Red Rings"

7. **Routine completes** → Status shows cyan "COMPLETED"; RUN button turns green

8. **User taps "VARS"** → Variables menu opens; adjust `Distance` value

9. **User taps "BACK"** → Returns to Auton Runner; run again with new values

---
- [gui.hpp](gui.hpp) - Base GUI class
- [gui-debug.hpp](gui-debug.hpp) - Debug GUI class with full API
