#ifndef AON_TOOLS_GUI_V2_HPP_
#define AON_TOOLS_GUI_V2_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "../../../api.h"
#include "../function-reader.hpp"
#include "../gui-image-generator/gui-images.hpp"
#include "../../competition/autonomous-routines.hpp"
#include "../../competition/operator-control.hpp"
#include "../../sensing/odometry.hpp"
#include "../../globals.hpp"


/*
====================================================================================================================
 GUI USAGE: Registering Test Functions and Tunable Variables
====================================================================================================================

What these do
- Registered Tests (Debug Menu 1): Any function you register shows up as a selectable item. Selecting it sets the
  current autonomous to that function (it doesn’t run immediately). Run it from Debug Menu 2 (Auton Runner).
- Tunable Variables (Debug Menu 3): Any variable you register can be adjusted live with +/- buttons on the brain.

Where to register (do it once, not every loop)
- Good places: `initialize()` after the GUI starts, or the first pass of `opcontrol()` guarded by a static flag (In main.cpp).
- Avoid: calling inside tight loops every iteration.

----------------------------------------------------------------------------------------------------
1) Register a test/auton function (appears in Debug Menu 1: "Registered Autons")

APIs available (deduplicated by name):
  // int()-returning function pointer
  aon::gui::registerTestFunction(&SomeAutonRoutineReturningInt, "My Auton");

  // void()-returning function pointer (wrapped to int internally)
  aon::gui::registerTestFunction(&SomeDiagnosticVoidFunction, "Drive Tune");

  // std::function<int()> or lambda
  aon::gui::registerTestFunction([]() -> int {
    // ... do something ...
    return 0;
  }, "Lambda Test");

Example placement (once):
  void opcontrol() {
    static bool seeded = false;
    if (!seeded) {
      aon::gui::registerTestFunction(&anEvenBetterRoutine, "An Even Better Routine");
      seeded = true;
    }
    // ... driver control loop ...
  }

How it flows:
- Open Debug Menu 1 → tap your item → GUI sets the selected auton invoker and returns to Main Menu.
- Go to Debug Menu 2 → press RUN to execute. RUNNING state auto-returns to Main Menu when done/timeout.

----------------------------------------------------------------------------------------------------
2) Expose live-tunable variables (appears in Debug Menu 3: "Variables")

Define a registered once using `setVariableRegister(...)`, then add variables inside with `VariableChanger(var, name)`:
  aon::gui::setVariableRegister([]{
    // Names must be unique; duplicates are ignored
    aon::gui::VariableChanger(MAX_RPM_TEST, "Max RPM Test");
    // aon::gui::VariableChanger(kP, "Drive kP"); EXAMPLE OF OTHER VARIABLE
    // aon::gui::VariableChanger(kI, "Drive kI"); EXAMPLE OF OTHER VARIABLE
  });

Example placement (once):
  void initialize() {
    aon::gui::Initialize();
    aon::gui::setVariableRegister([]{
      aon::gui::VariableChanger(MAX_RPM_TEST, "Max RPM Test");
    });
    // ... other init ...
  }

When do variables appear?
- When you open Debug Menu 3, if the internal list is empty and a registrar is set, the GUI calls your registrar
  to populate entries. Reopen the screen after updating your registrar to see new items.

Notes & tips
- Deduplication: `registerTestFunction` and `VariableChanger` both ignore duplicate names.
- Don’t block: Keep registrar code quick (no long delays). It may run on the main thread when a screen opens.
- Preselecting Auton: You can also preselect via `selectedRedAut/selectedBlueAut/selectedSkill` (1..3) or by directly
  assigning `selectedAutonRoutine` + `selectedAutonName` before/after `Initialize()`; the GUI reflects it in Main Menu.
- Running Safely: The Auton Runner has a 30s watchdog that stops drive/intake/arm and returns to Main Menu on timeout.

Troubleshooting
- Item not showing? Ensure the registered function was set before opening its Debug screen, the name is unique, and the code runs
  only once (not inside a tight loop). If needed, close and reopen the Debug screen to force a redraw.

----------------------------------------------------------------------------------------------------
Final, full example (as used in opcontrol during testing)

  void opcontrol() {
    aon::ConfigureMotors();
    while (true) {
      #if TESTING_AUTONOMOUS
      // Run the autonomous test setup only once, then fall back to normal driver control
      static bool ran_auton_once = false;
      if (!ran_auton_once && !aon::operator_control::g_guiAutonActive) {
        // Seed GUI registrars once outside the tight loop
        aon::gui::setVariableRegister([]{
          // Expose test RPM for live tuning
          aon::gui::VariableChanger(MAX_RPM_TEST, "Max RPM Test");
        });
        // Make a test/auton visible in Debug Menu 1
        aon::gui::registerTestFunction(anEvenBetterRoutine, "An Even Better Routine");

        // Prepare drivetrain and execute autonomous (one-time)
        aon::ConfigureMotors(false); // Set drivetrain to hold for auton testing
        aon::AutonomousReader->ExecuteFunction("autonomous");
        ran_auton_once = true;
        pros::delay(100);
      }
      // After first run, keep controller responsive
      aon::operator_control::Run(aon::operator_control::DEFAULT);
      #else
      aon::operator_control::Run(aon::operator_control::DEFAULT);
      #endif
      pros::delay(10);
    }
  }

GUI outcome:
- Debug 1: "An Even Better Routine" appears; tapping selects it.
- Debug 2: Shows selected auton; RUN executes with a 30s watchdog.
- Debug 3: "Max RPM Test" appears with +/- adjusters for live tuning.
*/

namespace aon {

inline std::unique_ptr<FunctionReader<int>> AutonomousReader =
    std::make_unique<FunctionReader<int>>();

namespace gui {
//Displays the currently selected auton routine
inline int (*selectedAutonRoutine)() = nullptr;
inline std::string selectedAutonName = "None";

//Change from 1 to 3 to pre select auton
inline int selectedRedAut = 0;
inline int selectedBlueAut = 0;
inline int selectedSkill = 0;

// List-based mapping for auton options per alliance
enum class Alliance { Red, Blue, Skills };
struct AutonOption {
  const char* name;
  int (*routine)();
};

// Auton routines for each alliance and their names
// Any changes should be done here 

static constexpr AutonOption kRedAutonOptions[] = {
  {"Red AUT1", aon::RedRingsRoutine},
  {"Red AUT2", aon::RedRingsRoutine},
  {"Red AUT3", aon::RedRingsRoutine},
};
static constexpr AutonOption kBlueAutonOptions[] = {
  {"Blue AUT1", aon::BlueRingsRoutine},
  {"Blue AUT2", aon::BlueRingsRoutine},
  {"Blue AUT3", aon::BlueRingsRoutine},
};
static constexpr AutonOption kSkillsAutonOptions[] = {
  {"Skills AUT1", aon::RedRingsRoutine},
  {"Skills AUT2", aon::RedRingsRoutine},
  {"Skills AUT3", aon::RedRingsRoutine},
};



#pragma region GUI Global State & Registries
//    _____ _    _ _____    _____ _       _           _    _____ _        _                  _____            _     _        _           
//   / ____| |  | |_   _|  / ____| |     | |         | |  / ____| |      | |         ___    |  __ \          (_)   | |      (_)          
//  | |  __| |  | | | |   | |  __| | ___ | |__   __ _| | | (___ | |_ __ _| |_ ___   ( _ )   | |__) |___  __ _ _ ___| |_ _ __ _  ___  ___ 
//  | | |_ | |  | | | |   | | |_ | |/ _ \| '_ \ / _` | |  \___ \| __/ _` | __/ _ \  / _ \/\ |  _  // _ \/ _` | / __| __| '__| |/ _ \/ __|
//  | |__| | |__| |_| |_  | |__| | | (_) | |_) | (_| | |  ____) | || (_| | ||  __/ | (_>  < | | \ \  __/ (_| | \__ \ |_| |  | |  __/\__ \
//   \_____|\____/|_____|  \_____|_|\___/|_.__/ \__,_|_| |_____/ \__\__,_|\__\___|  \___/\/ |_|  \_\___|\__, |_|___/\__|_|  |_|\___||___/
//                                                                                                       __/ |                           
//                                                                                                      |___/                            

// GUI screen states

enum Screen {
  kMainMenu,
  kAutonMenu,
  kRedAutons,
  kBlueAutons,
  kSkillAutons,
  kDebugMenu,
  kDebugging,
  kDebugging2,
  kDebugging3,
  kDebugging4,
};
inline Screen CurrentScreen = kMainMenu;
inline Screen PreviousScreen = kMainMenu;

// Current selected auton thunk and invoker to bridge std::function to raw pointer
inline std::function<int()> g_selectedAutonInvoker = nullptr;

// Persistent GUI loop task handle
inline std::unique_ptr<pros::Task> g_guiLoopTask{nullptr};

// Container to store test/auton functions and their names (use int()-returning for auton compatibility)
inline std::vector<std::pair<std::string, std::function<int()>>> testFunctions;

// Optional lazy Register that user code can set to seed tests on demand
inline std::function<void()> g_testRegister = nullptr;

// Track if a GUI-initiated autonomous is currently running to prevent re-entry
inline bool g_autonRunning = false;

// Variable changer registry: name + pointer to double to modify live
struct VariableEntry {
  std::string name;
  double* ptr;
};
inline std::vector<VariableEntry> variableEntries;
inline std::function<void()> g_variableRegister = nullptr;

// API: register a variable to be editable in Debug Menu 3
inline void VariableChanger(double& variableRef, const std::string& name) {
  // prevent duplicate names
  for (const auto& e : variableEntries) {
    if (e.name == name) return;
  }
  variableEntries.push_back(VariableEntry{name, &variableRef});
}

// Allow user code to provide a Register (similar to tests)
inline void setVariableRegister(const std::function<void()>& Register) { g_variableRegister = Register; }

// Allow user code to provide a Register that calls registerTestFunction(...)
inline void setTestRegister(const std::function<void()>& Register) { g_testRegister = Register; }

// Internal helper to add unique entries by name
static void addTestFunctionInternal(const std::string& name, std::function<int()> fn) {
  // Prevent duplicates by name
  for (const auto& [existingName, _] : testFunctions) {
    if (existingName == name) {
      return; // Already registered
    }
  }
  testFunctions.emplace_back(name, std::move(fn));
}


inline void registerTestFunction(int (*func)(), const std::string& name) {
  addTestFunctionInternal(name, func);
}

// Overload: register a std::function<int()>
inline void registerTestFunction(const std::function<int()>& func, const std::string& name) {
  addTestFunctionInternal(name, func);
}

// Overload: allow void() functions; wrap to int() by returning 0
inline void registerTestFunction(void (*func)(), const std::string& name) {
  addTestFunctionInternal(name, [func]() -> int {
    if (func) func();
    return 0;
  });
}


static int InvokeSelectedAuton() {
  if (g_selectedAutonInvoker) return g_selectedAutonInvoker();
  return 0;
}
// Needed for organization 
static void DisplayMainMenu();
static void DisplayDebugMenu2();
static void DisplayDebugMenu3();
static void DisplayDebugMenu4();

// Select auton by alliance and 1-based index using the option lists
static void SelectAutonByList(Alliance alliance, int index1Based) {
  if (index1Based < 1) index1Based = 1;
  if (index1Based > 3) index1Based = 3;

  const AutonOption* options = nullptr;
  switch (alliance) {
    case Alliance::Red:
      options = kRedAutonOptions;
      selectedRedAut = index1Based;
      selectedBlueAut = 0;
      selectedSkill = 0;
      break;
    case Alliance::Blue:
      options = kBlueAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = index1Based;
      selectedSkill = 0;
      break;
    case Alliance::Skills:
      options = kSkillsAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = 0;
      selectedSkill = index1Based;
      break;
  }
  const AutonOption& choice = options[index1Based - 1];
  selectedAutonName = choice.name;
  selectedAutonRoutine = choice.routine;

  if (CurrentScreen == kMainMenu) {
    DisplayMainMenu();
  }
}


static void ApplyPreselectedAuton() {
  // If user already directly set selectedAutonRoutine, don't override.
  if (selectedAutonRoutine != nullptr && selectedAutonName != "None" &&
      (selectedRedAut == 0 && selectedBlueAut == 0 && selectedSkill == 0)) {
    return; // Nothing to map; routine already chosen
  }

  // Helper lambda to refresh main menu after setting selection (if GUI already showing)
  auto refreshMenu = [] {
    if (CurrentScreen == kMainMenu) {
      DisplayMainMenu();
    }
  };

  if (selectedRedAut > 0) {
    SelectAutonByList(Alliance::Red, selectedRedAut);
    return; // Red takes precedence
  }

  if (selectedBlueAut > 0) {
    SelectAutonByList(Alliance::Blue, selectedBlueAut);
    return; // Blue next
  }

  if (selectedSkill > 0) {
    SelectAutonByList(Alliance::Skills, selectedSkill);
  }
}

#pragma endregion

#pragma region GUI_DISPLAY_FUNCTIONS00

//   _____  _           _                 
//  |  __ \(_)         | |                
//  | |  | |_ ___ _ __ | | __ _ _   _ ___ 
//  | |  | | / __| '_ \| |/ _` | | | / __|
//  | |__| | \__ \ |_) | | (_| | |_| \__ \
//  |_____/|_|___/ .__/|_|\__,_|\__, |___/
//               | |             __/ |    
//               |_|            |___/     

// Function to display the initialization message
static int DisplayInitializationMessageWithReturn() {
  // Clear the screen completely
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set the pen color to white for the text
  pros::screen::set_pen(COLOR_WHITE);

  // Display the initialization message at the top center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "Jarvis Win the match");

  // Display the secondary message at the center in large text
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "Right away AON");

  return 0; // Return an integer as required
}

// MAIN MENU
static void DisplayMainMenu() {
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Draw the AON logo higher at the top center
  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2, (BRAIN_SCREEN_HEIGHT - 225) / 4);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE); // Default color for "NO AUTON"
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON"); // Moved to top center
  } else {
    // Placeholder for dynamic color based on selected auton
    pros::screen::set_pen(COLOR_GREEN); // Example: Green for Skills
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  // Draw the "AUTONS" button in the bottom-left corner
  pros::screen::set_eraser(COLOR_GREEN);
  pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, 60, BRAIN_SCREEN_HEIGHT - 40, "AUTONS"); // Positioned above the block

  // Draw the "DEBUG" button in the bottom-right corner
  pros::screen::set_eraser(COLOR_GRAY);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 + 70, BRAIN_SCREEN_HEIGHT - 40, "DEBUG"); // Positioned above the block
}


// DEBUG HUB MENU
static void DisplayDebugMenu() {
  // Hub for debug options: Registered Autons, Auton Runner, Variables, Odometry
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Title
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "DEBUG MENU");

  // BACK button
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(10, 10, 90, 40);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 18, "BACK");

  // Buttons for debug options
  int y = 60;
  const char* options[] = {"Registered Autons", "Auton Runner", "Variables", "Odometry"};
  for (int i = 0; i < 4; ++i) {
    pros::screen::set_eraser(COLOR_LIGHT_GRAY);
    pros::screen::erase_rect(50, y, BRAIN_SCREEN_WIDTH - 50, y + 40);
    pros::screen::set_pen(COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_MEDIUM, 60, y + 10, options[i]);
    y += 50;
  }
}

// REGISTERED AUTONS SCREEN
static void DisplayRegisteredAutonsMenu() {
  // Registered Autons list with header and navigation
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Header bar (black) with title
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase_rect(0, 0, BRAIN_SCREEN_WIDTH, 40);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH/2 - 100, 10, "Registered Autons");

  // BACK button (top-left in header)
  int backX1 = 10;
  int backY1 = 6;
  int backX2 = backX1 + 80;
  int backY2 = backY1 + 28;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 10, backY1 + 6, "BACK");

  // NEXT button (top-right in header)
  int nextY1 = 6;
  int nextY2 = nextY1 + 28;
  int nextX1 = BRAIN_SCREEN_WIDTH - 120;
  int nextX2 = BRAIN_SCREEN_WIDTH - 40;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(nextX1, nextY1, nextX2, nextY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, nextX1 + 12, nextY1 + 6, "NEXT");

  // Header underline
  pros::screen::set_eraser(COLOR_LIGHT_GRAY);
  pros::screen::erase_rect(0, 42, BRAIN_SCREEN_WIDTH, 44);

  // If no tests are registered yet and a Register is available, seed them now
  if (testFunctions.empty() && g_testRegister) {
    g_testRegister();
  }

  // Check if there are any registered test functions
  if (testFunctions.empty()) {
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "No Test Functions Registered");
    return; // Exit early if no functions are registered
  }

  // Display buttons for each registered test function
  int yOffset = 70; // Start a bit lower to avoid any overlap with title/NEXT
  for (size_t i = 0; i < testFunctions.size(); ++i) {
    const auto& [name, _] = testFunctions[i];

    // Draw a button for the test function
    pros::screen::set_eraser(COLOR_LIGHT_GRAY);
    pros::screen::erase_rect(50, yOffset, BRAIN_SCREEN_WIDTH - 50, yOffset + 40);
    pros::screen::set_pen(COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_MEDIUM, 60, yOffset + 10, name.c_str());

    yOffset += 50; // Move to the next button position
  }
}


// AUTON MENU
static void DisplayAutonMenu() {
  // Main AUTONS hub: shows current selection and navigates to Red/Blue/Skills submenus
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  aon::drawMatrixLogo();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE); // Default color for "NO AUTON"
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON"); // Moved to top center
  } else {
    // Placeholder for dynamic color based on selected auton
    pros::screen::set_pen(COLOR_GREEN); // Example: Green for Skills
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  // Draw the "BLUE" box on the center right with reduced height
  pros::screen::set_eraser(COLOR_BLUE);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH - 130, BRAIN_SCREEN_HEIGHT / 2 - 30, BRAIN_SCREEN_WIDTH - 30, BRAIN_SCREEN_HEIGHT / 2 + 30);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH - 117, BRAIN_SCREEN_HEIGHT / 2 - 10, "BLUE"); // Positioned above the block

  // Draw the "RED" box on the center left with reduced height
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase_rect(45, BRAIN_SCREEN_HEIGHT / 2 - 30, 145, BRAIN_SCREEN_HEIGHT / 2 + 30);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, 67, BRAIN_SCREEN_HEIGHT / 2 - 10, "RED"); // Positioned above the block

  // Draw the "SKILLS" box at the bottom center
  pros::screen::set_eraser(COLOR_GREEN);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2 - 60, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 80, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 85, "SKILLS"); // Centered on the button
}


// DEBUG SCREEN 2
static void DisplayDebugMenu2() {
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Header bar (black)
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase_rect(0, 0, BRAIN_SCREEN_WIDTH, 40);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH/2 - 70, 10, "Auton Runner");
  // Back button (top-left)
  const int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 10, backY1 + 6, "BACK");

  // Menu button (top-right)
  const int menuY1 = 6;
  const int menuY2 = menuY1 + 28;
  const int menuX1 = BRAIN_SCREEN_WIDTH - 120;
  const int menuX2 = BRAIN_SCREEN_WIDTH - 40;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(menuX1, menuY1, menuX2, menuY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, menuX1 + 6, menuY1 + 6, "MENU");
  // Header underline
  pros::screen::set_eraser(COLOR_LIGHT_GRAY);
  pros::screen::erase_rect(0, 42, BRAIN_SCREEN_WIDTH, 44);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH/2 - 70, 10, "Auton Runner");

  // Selected auton panel (card)
  const bool hasAuton = (selectedAutonRoutine != nullptr) || static_cast<bool>(g_selectedAutonInvoker);
  // Leave space on the far-right for the VARS button
  const int cardX1 = 16, cardY1 = 50, cardX2 = BRAIN_SCREEN_WIDTH - 160, cardY2 = 145;
  // Border
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(cardX1, cardY1, cardX2, cardY2);
  // Inner
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase_rect(cardX1 + 2, cardY1 + 2, cardX2 - 2, cardY2 - 2);
  // Label and name
  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_MEDIUM, cardX1 + 10, cardY1 + 8, "Selected:");
  pros::screen::set_pen(hasAuton ? COLOR_GREEN : COLOR_RED);
  const char* nameText = hasAuton ? selectedAutonName.c_str() : "NO AUTON";
  // Move the selected auton name further left (left-aligned in card)
  pros::screen::print(pros::E_TEXT_LARGE, cardX1 + 14, cardY1 + 48, nameText);

  // Button to Variable Changer (Debug Menu 3) – draw after card so it stays visible
  const int vcY1 = 70; // lowered below header
  const int vcY2 = vcY1 + 40; // slightly taller for visibility
  const int vcX1 = BRAIN_SCREEN_WIDTH - 140; // farther right
  const int vcX2 = BRAIN_SCREEN_WIDTH - 40;  // width 100
  pros::screen::set_eraser(COLOR_ORANGE);
  pros::screen::erase_rect(vcX1, vcY1, vcX2, vcY2);
  // Orange button with black label
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_MEDIUM, vcX1 + 18, vcY1 + 8, "VARS");

  // RUN button lower and shifted left
  const int runCenterX = BRAIN_SCREEN_WIDTH/2 - 60; // shift left
  int runX1 = runCenterX - (g_autonRunning ? 150 : 60);
  const int runY1 = BRAIN_SCREEN_HEIGHT - 70; // lower
  int runX2 = runCenterX + (g_autonRunning ? 150 : 60);
  const int runY2 = BRAIN_SCREEN_HEIGHT - 20;
  // RUN button with border
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(runX1 - 2, runY1 - 2, runX2 + 2, runY2 + 2);
  pros::screen::set_eraser((hasAuton && !g_autonRunning) ? COLOR_GREEN : (g_autonRunning ? COLOR_ORANGE : COLOR_DARK_GRAY));
  pros::screen::erase_rect(runX1, runY1, runX2, runY2);
  pros::screen::set_pen(COLOR_WHITE);
  // Center text for both labels
  const char* btnText = g_autonRunning ? "RUNNING" : "RUN";
  const int textOffset = g_autonRunning ? 65 : 30; // wider text when RUNNING
  pros::screen::print(pros::E_TEXT_LARGE, runCenterX - textOffset, runY1 + 15, btnText);
}

// DEBUG SCREEN 3 (Variable Changer)
static void DisplayDebugMenu3() {
  // Variables/Tuner screen: header with BACK and MENU; each variable has 6 adjusters
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "VARIABLES");

  // BACK button (returns to Debug Menu 2)
  const int backY1 = 10;
  const int backY2 = backY1 + 30;
  const int backX2 = BRAIN_SCREEN_WIDTH / 2 - 110; // symmetric opposite of NEXT
  const int backX1 = backX2 - 80; // width 80 like NEXT
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 8, backY1 + 8, "BACK");

  // MENU button (returns to Main Menu)
  const int menuY1 = 10;
  const int menuY2 = menuY1 + 30;
  const int menuX1 = BRAIN_SCREEN_WIDTH - 120; // move far right
  const int menuX2 = BRAIN_SCREEN_WIDTH - 40;  // keep width ~80
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(menuX1, menuY1, menuX2, menuY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, menuX1 + 6, menuY1 + 8, "MENU");

  // If no variables and Register provided, seed now
  if (variableEntries.empty() && g_variableRegister) g_variableRegister();

  if (variableEntries.empty()) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "No Variables Registered");
    return;
  }

  int y = 60; // start below title
  for (size_t i = 0; i < variableEntries.size(); ++i) {
    const auto& e = variableEntries[i];
    // Name and current value
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, e.name.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH - 140, y, "%0.3f", *(e.ptr));

    // Buttons: -10, -1, -0.1 | +0.1, +1, +10
    struct Btn { int x1, y1, x2, y2; const char* text; double delta; };
    Btn btns[6];
    int bx = 20; int by = y + 20; int bw = 70; int bh = 30; int gap = 10;
    // Left side (decrements)
    btns[0] = {bx, by, bx + bw, by + bh, "-10", -10.0}; bx += bw + gap;
    btns[1] = {bx, by, bx + bw, by + bh, "-1", -1.0}; bx += bw + gap;
    btns[2] = {bx, by, bx + bw, by + bh, "-0.1", -0.1};
    // Right side (increments)
    bx = BRAIN_SCREEN_WIDTH - (bw * 3 + gap * 2) - 20; // align right group
    btns[3] = {bx, by, bx + bw, by + bh, "+0.1", +0.1}; bx += bw + gap;
    btns[4] = {bx, by, bx + bw, by + bh, "+1", +1.0}; bx += bw + gap;
    btns[5] = {bx, by, bx + bw, by + bh, "+10", +10.0};

    for (int b = 0; b < 6; ++b) {
      pros::screen::set_eraser(btns[b].delta < 0 ? COLOR_DARK_RED : COLOR_DARK_GREEN);
      pros::screen::erase_rect(btns[b].x1, btns[b].y1, btns[b].x2, btns[b].y2);
      pros::screen::set_pen(COLOR_WHITE);
      pros::screen::print(pros::E_TEXT_MEDIUM, btns[b].x1 + 10, btns[b].y1 + 8, btns[b].text);
    }

    y += 70; // next variable row
  }
}


// DEBUG MENU 4: ODOMETRY & SENSORS
static void DisplayDebugMenu4() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Title
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "ODOMETRY & SENSORS");

  // BACK button
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(10, 10, 90, 40);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 18, "BACK");

  // Display odometry values
  int y = 60;
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "X: %.2f in", aon::odometry::GetX()); y += 30;
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Y: %.2f in", aon::odometry::GetY()); y += 30;
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Theta: %.2f deg", aon::odometry::GetDegrees()); y += 30;

  // Display inertial sensor data
  #if GYRO_ENABLED
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Gyro Heading: %.2f deg", aon::odometry::gyroscope.get_heading()); y += 30;
  #endif

  // Encoder values
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Enc Right: %.2f deg", aon::odometry::encoderRight.get_position() / 100.0); y += 30;
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, "Enc Back: %.2f deg", -aon::odometry::encoderBack.get_position() / 100.0); y += 30;
}


// RED AUTON MENU
static void DisplayRedAutonMenu() {
  // Red-side autons list with three option buttons; selection updates name + routine
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to red
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "RED" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "RED");

  // Draw three blocks at the bottom and place text in the center of each rectangle
  pros::screen::set_eraser(COLOR_LIGHT_PINK);
  pros::screen::erase_rect(50, BRAIN_SCREEN_HEIGHT - 100, 150, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, 63, BRAIN_SCREEN_HEIGHT - 87, "AUT1");

  pros::screen::set_eraser(COLOR_CRIMSON);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 50, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 - 40, BRAIN_SCREEN_HEIGHT - 87, "AUT2");

  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH - 50, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH - 140, BRAIN_SCREEN_HEIGHT - 87, "AUT3");
}

// BLUE AUTON MENU
static void DisplayBlueAutonMenu() {
  // Blue-side autons list; same layout as Red
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to blue
  pros::screen::set_eraser(COLOR_BLUE);
  pros::screen::erase();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "BLUE" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "BLUE");

  // Draw three blocks at the bottom and place text in the center of each rectangle
  pros::screen::set_eraser(COLOR_SKY_BLUE);
  pros::screen::erase_rect(50, BRAIN_SCREEN_HEIGHT - 100, 150, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, 63, BRAIN_SCREEN_HEIGHT - 87, "AUT1");

  pros::screen::set_eraser(COLOR_STEEL_BLUE);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 50, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 - 40, BRAIN_SCREEN_HEIGHT - 87, "AUT2");

  pros::screen::set_eraser(COLOR_BLUE);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH - 50, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH - 140, BRAIN_SCREEN_HEIGHT - 87, "AUT3");
}

// SKILLS MENU
static void DisplaySkillsMenu() {
  // Skills autons list; same layout as alliance screens with green background
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to green
  pros::screen::set_eraser(COLOR_GREEN);
  pros::screen::erase();

  // Add a delay to allow the screen to load
  pros::delay(300); // 300ms delay

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "SKILLS" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "SKILLS");

  // Draw three blocks at the bottom and place text in the center of each rectangle
  pros::screen::set_eraser(COLOR_LIGHT_GREEN);
  pros::screen::erase_rect(50, BRAIN_SCREEN_HEIGHT - 100, 150, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, 63, BRAIN_SCREEN_HEIGHT - 87, "AUT1");

  pros::screen::set_eraser(COLOR_YELLOW_GREEN);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 50, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 - 40, BRAIN_SCREEN_HEIGHT - 87, "AUT2");

  pros::screen::set_eraser(COLOR_GREEN);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH - 50, BRAIN_SCREEN_HEIGHT - 50);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH - 140, BRAIN_SCREEN_HEIGHT - 87, "AUT3");
}

#pragma endregion

#pragma region GUI_TOUCH_HANDLERS
//   _______               _       _    _                 _ _               
//  |__   __|             | |     | |  | |               | | |              
//     | | ___  _   _  ___| |__   | |__| | __ _ _ __   __| | | ___ _ __ ___ 
//     | |/ _ \| | | |/ __| '_ \  |  __  |/ _` | '_ \ / _` | |/ _ \ '__/ __|
//     | | (_) | |_| | (__| | | | | |  | | (_| | | | | (_| | |  __/ |  \__ \
//     |_|\___/ \__,_|\___|_| |_| |_|  |_|\__,_|_| |_|\__,_|_|\___|_|  |___/
                                                                         
                                                                         
// HANDLE MAIN MENU TOUCH
static void HandleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) {
  // Check if the "AUTONS" button is pressed (bottom-left corner)
  if (touchStatus.x < BRAIN_SCREEN_WIDTH / 2 && touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
    // Ensure we are not already in the Autonomous Menu
    if (CurrentScreen != kAutonMenu) {
      // Clear the "AUTONS" button area
      pros::screen::set_eraser(COLOR_BLACK);
      pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT);

      // Display the Autonomous Menu
      DisplayAutonMenu();
      CurrentScreen = kAutonMenu; // Update the current screen state
    }
  }

  // Check if the "DEBUG" button is pressed (bottom-right corner)
  else if (touchStatus.x > BRAIN_SCREEN_WIDTH / 2 && touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
    // Clear the "DEBUG" button area
    pros::screen::set_eraser(COLOR_BLACK);
    pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);

    // Ensure debug items are up-to-date when opening the Debug Menu
    if (g_testRegister) {
      g_testRegister();
    }

    // Display the Debug Menu when the DEBUG button is pressed
    DisplayDebugMenu();
    CurrentScreen = kDebugMenu; // Update the current screen state
    pros::delay(200); // Debounce to prevent accidental clicks
  }
}


// HANDLE AUTON MENU TOUCH
static void HandleAutonMenuTouch() {
  // Routes touches in AUTONS hub to Red/Blue/Skills screens
  // Ensure the touch handling is only active for the Auton Menu screen
  if (CurrentScreen != kAutonMenu) return;

  // Get the touch coordinates
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();

  // Check if the screen is being touched
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // Check if the "BLUE" button is pressed (center-right) with reduced height
    if (x > BRAIN_SCREEN_WIDTH - 130 && x < BRAIN_SCREEN_WIDTH - 30 &&
        y > BRAIN_SCREEN_HEIGHT / 2 - 30 && y < BRAIN_SCREEN_HEIGHT / 2 + 30) {
      DisplayBlueAutonMenu();
      CurrentScreen = kBlueAutons; // Update the current screen state
    }

    // Check if the "RED" button is pressed (center-left) with reduced height
    else if (x > 30 && x < 130 &&
             y > BRAIN_SCREEN_HEIGHT / 2 - 30 && y < BRAIN_SCREEN_HEIGHT / 2 + 30) {
      DisplayRedAutonMenu();
      CurrentScreen = kRedAutons; // Update the current screen state
    }

    // Check if the "SKILLS" button is pressed (bottom-center)
    else if (x > BRAIN_SCREEN_WIDTH / 2 - 60 && x < BRAIN_SCREEN_WIDTH / 2 + 80 &&
             y > BRAIN_SCREEN_HEIGHT - 100 && y < BRAIN_SCREEN_HEIGHT - 50) {
      DisplaySkillsMenu();
      CurrentScreen = kSkillAutons; // Update the current screen state
    }
  }
}

// HANDLE REGISTERED AUTONS MENU TOUCH
static void HandleRegisteredAutonsMenuTouch() {
  // Header navigation (BACK/NEXT) plus selection of registered autons
  // Get the touch coordinates
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();

  // Check if the screen is being touched
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // Check if BACK button (top-left in header) is pressed
    {
      int backX1 = 10;
      int backY1 = 6;
      int backX2 = backX1 + 80;
      int backY2 = backY1 + 28;
      if (x >= backX1 && x <= backX2 && y >= backY1 && y <= backY2) {
        // Return to Main Menu
        DisplayMainMenu();
        CurrentScreen = kMainMenu;
        pros::delay(300);
        return;
      }
    }

    // Check if NEXT button (top-right in header) is pressed
    int nextY1 = 6;
    int nextY2 = nextY1 + 28;
    int nextX1 = BRAIN_SCREEN_WIDTH - 120;
    int nextX2 = BRAIN_SCREEN_WIDTH - 40;
    if (x >= nextX1 && x <= nextX2 && y >= nextY1 && y <= nextY2) {
      // Go to second debug screen
      DisplayDebugMenu2();
      CurrentScreen = kDebugging2;
      pros::delay(300);
      return;
    }

    // Check which button is pressed
    int yOffset = 70; // Start below the title (matches DisplayDebugMenu rendering)
    for (size_t i = 0; i < testFunctions.size(); ++i) {
      if (x >= 50 && x <= BRAIN_SCREEN_WIDTH - 50 && y >= yOffset && y <= yOffset + 40) {
  // Set the selected auton to the tapped test function
        const auto& [name, fn] = testFunctions[i];
        selectedAutonName = name;
        g_selectedAutonInvoker = fn; // store selection for future use (no run here)
        // Keep behavior: go back to main menu to reflect selection
        DisplayMainMenu();
        CurrentScreen = kMainMenu;
        return;
      }
      yOffset += 50; // Move to the next button position
    }
  }
}

// HANDLE DEBUG MENU TOUCH (HUB)
static void HandleDebugMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // BACK button
    if (x >= 10 && x <= 90 && y >= 10 && y <= 40) {
      DisplayMainMenu();
      CurrentScreen = kMainMenu;
      pros::delay(300);
      return;
    }

    // Buttons for debug options
    int yy = 60;
    for (int i = 0; i < 4; ++i) {
      if (x >= 50 && x <= BRAIN_SCREEN_WIDTH - 50 && y >= yy && y <= yy + 40) {
        switch (i) {
          case 0: DisplayRegisteredAutonsMenu(); CurrentScreen = kDebugging; break;
          case 1: DisplayDebugMenu2(); CurrentScreen = kDebugging2; break;
          case 2: DisplayDebugMenu3(); CurrentScreen = kDebugging3; break;
          case 3: DisplayDebugMenu4(); CurrentScreen = kDebugging4; break;
        }
        pros::delay(400);
        return;
      }
      yy += 50;
    }
  }
}

// HANDLE RED BLUE SKILLS MENU TOUCH
static void HandleREDBLUESKILLSMenuTouch() {
  // Shared touch handler for Red/Blue/Skills menus using consistent button bounds
  // Get the touch coordinates
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();

  // Check if the screen is being touched
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;
    // Handle touch based on the current screen using list-driven button mapping
    switch (CurrentScreen) {
      case kRedAutons:
      case kBlueAutons:
      case kSkillAutons: {
        struct Btn { int x1, x2; };
        constexpr int y1 = BRAIN_SCREEN_HEIGHT - 100, y2 = BRAIN_SCREEN_HEIGHT - 50;
        Btn btns[3] = {
          {50, 150},
          {BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_WIDTH / 2 + 50},
          {BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_WIDTH - 50}
        };

        int selectedIdx = -1;
        for (int i = 0; i < 3; ++i) {
          if (x >= btns[i].x1 && x <= btns[i].x2 && y >= y1 && y <= y2) {
            selectedIdx = i + 1; // 1-based index for lists
            break;
          }
        }

        if (selectedIdx > 0) {
          if (CurrentScreen == kRedAutons) SelectAutonByList(Alliance::Red, selectedIdx);
          else if (CurrentScreen == kBlueAutons) SelectAutonByList(Alliance::Blue, selectedIdx);
          else if (CurrentScreen == kSkillAutons) SelectAutonByList(Alliance::Skills, selectedIdx);

          // After selection, go back to main to reflect the chosen auton
          DisplayMainMenu();
          CurrentScreen = kMainMenu;
        }
        break;
      }

      default:
        break;
    }
  }
}

/// HANDLE DEBUG SCREEN 2 TOUCH
static void HandleDebugMenu2Touch() {
  // Handles header nav (BACK/MENU), VARS navigation, and RUN execution + watchdog
  const auto touchStatus = pros::screen::touch_status();
  if (touchStatus.touch_status <= 0) return;

  // BACK button (top-left in header)
  {
    const int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
    const bool inBack = touchStatus.x >= backX1 && touchStatus.x <= backX2 && touchStatus.y >= backY1 && touchStatus.y <= backY2;
    if (inBack) {
      DisplayDebugMenu();
      CurrentScreen = kDebugging;
      pros::delay(300);
      return;
    }
  }

  // MENU button (top-right in header)
  {
    const int menuY1 = 6;
    const int menuY2 = menuY1 + 28;
    const int menuX1 = BRAIN_SCREEN_WIDTH - 120;
    const int menuX2 = BRAIN_SCREEN_WIDTH - 40;
    const bool inMenu = touchStatus.x >= menuX1 && touchStatus.x <= menuX2 && touchStatus.y >= menuY1 && touchStatus.y <= menuY2;
    if (inMenu) {
      DisplayMainMenu();
      CurrentScreen = kMainMenu;
      pros::delay(300);
      return;
    }
  }

  // Check Variables button
  {
    const int vcY1 = 70;
    const int vcY2 = vcY1 + 40;
    const int vcX1 = BRAIN_SCREEN_WIDTH - 140;
    const int vcX2 = BRAIN_SCREEN_WIDTH - 40;
    const bool inVC = touchStatus.x >= vcX1 && touchStatus.x <= vcX2 && touchStatus.y >= vcY1 && touchStatus.y <= vcY2;
    if (inVC) {
      // Seed variables if needed
      if (variableEntries.empty() && g_variableRegister) g_variableRegister();
      DisplayDebugMenu3();
      CurrentScreen = kDebugging3;
      pros::delay(300);
      return;
    }
  }

  // RUN button bounds (must match DisplayDebugMenu2)
  const int runCenterX = BRAIN_SCREEN_WIDTH/2 - 60;
  int runX1 = runCenterX - (g_autonRunning ? 150 : 60);
  const int runY1 = BRAIN_SCREEN_HEIGHT - 70;
  int runX2 = runCenterX + (g_autonRunning ? 150 : 60);
  const int runY2 = BRAIN_SCREEN_HEIGHT - 20;

  const bool inRunButton = touchStatus.x >= runX1 && touchStatus.x <= runX2 &&
                           touchStatus.y >= runY1 && touchStatus.y <= runY2;
  if (!inRunButton) return;

  // Only run if a routine is selected
  if (selectedAutonRoutine == nullptr && !g_selectedAutonInvoker) {
    return;
  }

  // Prevent re-entry if one is already running
  if (g_autonRunning) return;

  // Mark driver control suppressed during auton
  aon::operator_control::g_guiAutonActive = true;
  g_autonRunning = true;

  // Ensure invoker is set
  if (!g_selectedAutonInvoker && selectedAutonRoutine) {
    // Wrap raw routine pointer into invoker for consistent calling
    g_selectedAutonInvoker = [=]() -> int { return selectedAutonRoutine ? selectedAutonRoutine() : 0; };
  }

  // Start auton task
  static std::unique_ptr<pros::Task> autonTask;
  autonTask = std::make_unique<pros::Task>([] {
    if (g_selectedAutonInvoker) {
      (void)g_selectedAutonInvoker();
    }
    // When auton finishes naturally, restore state and return to main menu
    drivetrain.stop();
    intake.stop();
    orbit.stop();
    aon::operator_control::g_guiAutonActive = false;
    g_autonRunning = false;
    g_selectedAutonInvoker = nullptr;
    DisplayMainMenu();
    CurrentScreen = kMainMenu;
  });

  // Watchdog task: stop after 30s
  static std::unique_ptr<pros::Task> watchdogTask;
  watchdogTask = std::make_unique<pros::Task>([] {
    // Poll for up to 30s; exit early if auton finishes
    int elapsed = 0;
    while (elapsed < 30000 && g_autonRunning) {
      pros::delay(50);
      elapsed += 50;
    }
    if (!g_autonRunning) {
      return; // auton ended naturally; nothing to do
    }
    // Timeout: stop all major actuators and restore state
    drivetrain.stop();
    intake.stop();
    orbit.stop();
    aon::operator_control::g_guiAutonActive = false;
    g_autonRunning = false;
    g_selectedAutonInvoker = nullptr;
    DisplayMainMenu();
    CurrentScreen = kMainMenu;
  });

  // Refresh RUN button to indicate action started
  pros::screen::set_eraser(COLOR_ORANGE);
  pros::screen::erase_rect(runX1, runY1, runX2, runY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE, runCenterX - 65, runY1 + 15, "RUNNING");
}


static void HandleDebugMenu3Touch() {
  // Debounced header navigation and variable adjustment; redraw after each change
  static uint32_t lastTouchMs = 0; // debounce timer
  const auto touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;
  // Debounce: ignore rapid successive touches within 300ms
  uint32_t now = pros::millis();
  if (now - lastTouchMs < 300) {
    return;
  }

  // MENU button detection (return to main menu)
  {
    const int menuY1 = 10;
    const int menuY2 = menuY1 + 30;
    const int menuX1 = BRAIN_SCREEN_WIDTH - 120;
    const int menuX2 = BRAIN_SCREEN_WIDTH - 40;
    if (touch.x >= menuX1 && touch.x <= menuX2 && touch.y >= menuY1 && touch.y <= menuY2) {
      DisplayMainMenu();
      CurrentScreen = kMainMenu;
      lastTouchMs = now;
      return;
    }
  }

  // BACK button detection
  {
    const int backY1 = 10;
    const int backY2 = backY1 + 30;
    const int backX2 = BRAIN_SCREEN_WIDTH / 2 - 110;
    const int backX1 = backX2 - 80;
    if (touch.x >= backX1 && touch.x <= backX2 && touch.y >= backY1 && touch.y <= backY2) {
      DisplayDebugMenu2();
      CurrentScreen = kDebugging2;
      lastTouchMs = now;
      pros::delay(300);
      return;
    }
  }

  int y = 60; // start position synced with DisplayDebugMenu3
  for (size_t i = 0; i < variableEntries.size(); ++i) {
    auto& e = variableEntries[i];
    struct Btn { int x1, y1, x2, y2; double delta; };
    Btn btns[6];
    int bx = 20; int by = y + 20; int bw = 70; int bh = 30; int gap = 10;
    // Left side (decrements)
    btns[0] = {bx, by, bx + bw, by + bh, -10.0}; bx += bw + gap;
    btns[1] = {bx, by, bx + bw, by + bh, -1.0}; bx += bw + gap;
    btns[2] = {bx, by, bx + bw, by + bh, -0.1};
    // Right side (increments)
    bx = BRAIN_SCREEN_WIDTH - (bw * 3 + gap * 2) - 20;
    btns[3] = {bx, by, bx + bw, by + bh, +0.1}; bx += bw + gap;
    btns[4] = {bx, by, bx + bw, by + bh, +1.0}; bx += bw + gap;
    btns[5] = {bx, by, bx + bw, by + bh, +10.0};

    for (int b = 0; b < 6; ++b) {
      if (touch.x >= btns[b].x1 && touch.x <= btns[b].x2 && touch.y >= btns[b].y1 && touch.y <= btns[b].y2) {
        if (e.ptr) {
          *(e.ptr) += btns[b].delta;
          // Optional: clamp or format as needed
          DisplayDebugMenu3(); // refresh UI to show updated value
          lastTouchMs = now;
        }
        return;
      }
    }
    y += 70;
  }
}

// HANDLE DEBUG MENU 4 TOUCH
static void HandleDebugMenu4Touch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // BACK button
    if (x >= 10 && x <= 90 && y >= 10 && y <= 40) {
      DisplayDebugMenu();
      CurrentScreen = kDebugMenu;
      pros::delay(300);
      return;
    }
  }
}

#pragma endregion

// INITIALIZE
static void Initialize() {
  // Initialize odometry
  aon::odometry::Initialize();

  // Store functions in the reader
  AutonomousReader->AddFunction("Initialization", &DisplayInitializationMessageWithReturn);

  // Console output for debugging or motivational purposes
  std::cout << "Start" << std::endl;

  // Necessary delay for the Simplified Brain Screen API to initialize
  pros::delay(5);

  // Seed test functions at startup if a Register is provided
  if (g_testRegister) {
    g_testRegister();
  }

  // Optional: seed variable changer entries if a Register is provided
  if (g_variableRegister) {
    g_variableRegister();
  }

  CurrentScreen = kMainMenu;
  // Display initialization message
  DisplayInitializationMessageWithReturn();
  // Keep initialization message visible briefly before showing main menu
  pros::delay(3000);
  // Display the main menu screen
  DisplayMainMenu();

  // Apply any preselected auton choice the user configured before initialization
  ApplyPreselectedAuton();

  // Launch consolidated GUI loop task and return immediately
    if (!g_guiLoopTask) {
    g_guiLoopTask = std::make_unique<pros::Task>([] {
      while (true) {
        // General touch handling based on current screen
        pros::screen_touch_status_s_t TouchStatus = pros::screen::touch_status();
        if (TouchStatus.touch_status > 0) {
          switch (CurrentScreen) {
            case kMainMenu:
              HandleMainMenuTouch(TouchStatus);
              break;
            case kAutonMenu:
              HandleAutonMenuTouch();
              break;
            case kRedAutons:
            case kBlueAutons:
            case kSkillAutons:
              HandleREDBLUESKILLSMenuTouch();
              break;
            case kDebugMenu:
              HandleDebugMenuTouch();
              break;
            case kDebugging:
              HandleRegisteredAutonsMenuTouch();
              break;
            case kDebugging2:
              HandleDebugMenu2Touch();
              break;
            case kDebugging3:
              HandleDebugMenu3Touch();
              break;
            case kDebugging4:
              HandleDebugMenu4Touch();
              break;
            default:
              break;
          }
        }

        pros::delay(30); // Check touch status every 30 ms

        // Periodic refresh for screens that need real-time updates
        static int refreshCounter = 0;
        if (++refreshCounter >= 10) { // every ~300ms
          if (CurrentScreen == kDebugging4) {
            DisplayDebugMenu4();
          }
          refreshCounter = 0;
        }
      }
    });
  }
}



}  // namespace gui
}  // namespace aon

#endif  // AON_TOOLS_GUI_V2_HPP_

   