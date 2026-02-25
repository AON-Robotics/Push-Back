# GUI Guide - V2 (Refactored)

## Overview

The refactored GUI system provides five main features:
1. **Registered Test Functions** - Create your own functions and select them from the Debug Menu
2. **Auton Runner** - Execute autonomous routines during autonomous testing with run/stop controls
3. **Tunable Variables** - Live adjustment of parameters during testing
4. **Data Menu** - Live display of registered numeric values with sensor reset and variable access
5. **Live Graph** - Real-time visualization of X/Y data (e.g., odometry) 
---

## Table of Contents
- [Changing Autonomous Routines](#changing-autonomous-routines)
- [Quick Start](#quick-start)
- [Registering Test Functions](#registering-test-functions)
- [Auton Runner](#auton-runner)
- [Tunable Variables](#tunable-variables)
- [Data Menu](#data-menu)
- [Live Graph](#live-graph)
- [Debug Mode Toggle](#debug-mode-toggle)
- [Complete Example](#complete-example)
- [API Reference](#api-reference)
- [Troubleshooting](#troubleshooting)

---

## Quick Reference

### [Debug Mode Toggle](#debug-mode-toggle)

Switch between the regular GUI (`Gui`) and the debug GUI (`GuiDebug`) by changing the `TESTING_AUTONOMOUS` flag:

Edit `include/aon/constants.hpp` and modify the `TESTING_AUTONOMOUS` define:

```cpp
// In constants.hpp
#define TESTING_AUTONOMOUS true   // Uses GuiDebug - full debug features
#define TESTING_AUTONOMOUS false  // Uses Gui - competition mode (default)
```

The implementation in `src/aon/tools/gui/Gui-V2.cpp` automatically selects the correct GUI:

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
- Red **STOP** button: Stop a running auton immediately
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


Open [Gui-V2.hpp](Gui-V2.hpp) and jump directly to the `AutonOption` arrays:

- [Red Auton Options](Gui-V2.hpp#L87)
- [Blue Auton Options](Gui-V2.hpp#L93)
- [Skills Auton Options](Gui-V2.hpp#L99)

Note: when adding a new preset auton, also add a forward declaration for the function in the `aon` namespace at the top of `Gui-V2.hpp` so the GUI can reference it (for example: `int ForwardBackTurnRoutine();`). Then implement the function in [autonomous-routines.hpp](../../competition/autonomous-routines.hpp). Ensure the function is in the `aon::` namespace and returns an `int`.

```cpp
// Forward declarations at top of Gui-V2.hpp (inside namespace aon)
namespace aon {
  int RedRingsRoutine();
  int BlueRingsRoutine();
  int ForwardBackTurnRoutine();
  // Add other auton routine declarations as needed


// Auton option arrays (inside the Gui class)
static inline AutonOption RedAutonOptions[AutonOptionsCount] = {
  {"Red ForwardBackTurn", aon::ForwardBackTurnRoutine},
  {"Red AUT2", aon::RedRingsRoutine},
  {"Red AUT3", aon::RedRingsRoutine},
};

static inline AutonOption BlueAutonOptions[AutonOptionsCount] = {
  {"Blue AUT1", aon::BlueRingsRoutine},
  {"Blue AUT2", aon::BlueRingsRoutine},
  {"Blue AUT3", aon::BlueRingsRoutine},
};

static inline AutonOption SkillsAutonOptions[AutonOptionsCount] = {
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
static inline AutonOption RedAutonOptions[AutonOptionsCount] = {
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
Note: the initialization screen now renders the primary and secondary messages with a typewriter-style animation (see `src/aon/tools/gui/Gui-V2.cpp`).


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
5. The auton executes in the background via the `AutonomousReader` when triggered by the GUI; it returns when the routine completes.

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
- **Stop Button**: Instantly stop a running auton mid-execution
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
| **Red STOP** | Auton running | Tap to stop immediately |
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
   - The auton executes in the background
   - Status shows orange with auton name

4. **Stop if Needed**:
   - Tap the red **STOP** button at any time
   - Motors stop immediately and reconfigure for driver control

5. **After Completion**:
   - Status shows cyan "COMPLETED"
   - Motors automatically reconfigure for opcontrol
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

- **No built-in watchdog**: The GUI does not implement a 30-second watchdog. If a timeout is required, add it to your auton or use the existing `autonSafety` task to abort execution under your chosen conditions.
- **Safe Stop**: STOP button kills the auton task and calls `aon::STOP()` to halt all motors
- **Any Auton Source**: Works with both registered test functions AND preset autons from the AUTONS menu
- **Background Execution**: Auton runs in its own task, allowing the GUI to remain responsive

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
  aon::InitializeGui();
  
  // Set up tunable variables
  aon::gui.SetVariableRegister([]{
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
4. **Auton Runner** (Debug 2): Execute selected auton with RUN/STOP buttons; quick VARS access
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
4. Look for the 30-second watchdog timer—the auton may be timing out
5. Verify the function is registered with `RegisterTestFunction()`

### Auton Runner STOP button not working?

1. The button only appears as red **STOP** while an auton is running
2. If the auton finished naturally, the button reverts to green **RUN**
3. After pressing STOP, motors are reconfigured for opcontrol automatically

### Motors behaving strangely after running auton?

1. The Auton Runner configures motors to HOLD mode during autonomous
2. After completion or STOP, motors should auto-reconfigure for opcontrol
3. If issues persist, manually call `aon::Configure(true)` to reset motor modes

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

6. **User taps green "RUN"** → Executes `RedRingsRoutine()` in background task
   - Button turns red "STOP"
   - Status shows orange "Red Rings"

7. **Option A - Let it complete**:
   - After routine completes → Status shows cyan "COMPLETED"
   - Motors reconfigure for opcontrol

8. **Option B - Stop early**:
   - User taps red "STOP" → Auton task killed immediately
   - Motors reconfigure for opcontrol

9. **User taps "VARS"** → Variables menu opens; adjust `Drive kP` value

10. **User taps "BACK"** → Returns to Auton Runner; run again with new values

---
- [Gui-V2.hpp](Gui-V2.hpp) - Base GUI class
- [Gui-V2-Debug.hpp](Gui-V2-Debug.hpp) - Debug GUI class with full API
