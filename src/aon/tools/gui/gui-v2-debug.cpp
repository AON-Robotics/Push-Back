#include "../../../include/aon/tools/gui/gui-v2-debug.hpp"
namespace aon {

// Forward declarations for debug subsystem functions
void DisplayRegisteredAutonsMenu(GuiDebug* gui);
void DisplayAutonRunner(GuiDebug* gui);

void HandleRegisteredAutonsMenuTouch(GuiDebug* gui);
void HandleAutonRunnerTouch(GuiDebug* gui);

void DisplayVariablesMenu(GuiDebug* gui);
void HandleVariablesMenuTouch(GuiDebug* gui);

void DisplayDataMenu(GuiDebug* gui);
void HandleDataMenuTouch(GuiDebug* gui);

void DisplayLiveGraph(GuiDebug* gui);
void HandleLiveGraphTouch(GuiDebug* gui);

// ============================================================================
// Debug Registration Methods
// ============================================================================

void GuiDebug::AddTestFunctionInternal(const std::string& name, std::function<int()> fn) {
  // Prevent duplicates by name
  for (const auto& [existingName, _] : testFunctions) {
    if (existingName == name) {
      return;
    }
  }
  testFunctions.emplace_back(name, std::move(fn));
}

void GuiDebug::RegisterTestFunction(int (*func)(), const std::string& name) {
  AddTestFunctionInternal(name, func);
}

void GuiDebug::RegisterTestFunction(const std::function<int()>& func, const std::string& name) {
  AddTestFunctionInternal(name, func);
}

void GuiDebug::RegisterTestFunction(void (*func)(), const std::string& name) {
  AddTestFunctionInternal(name, [func]() -> int {
    if (func) func();
    return 0;
  });
}

void GuiDebug::VariableChanger(double& variableRef, const std::string& name) {
  // Prevent duplicate names
  for (const auto& e : variableEntries) {
    if (e.name == name) return;
  }
  variableEntries.push_back({name, &variableRef});
}

void GuiDebug::SetVariableRegister(const std::function<void()>& Register) {
  variableRegister = Register;
}

void GuiDebug::SetTestRegister(const std::function<void()>& Register) {
  testRegister = Register;
}

void GuiDebug::RegisterDataEntry(const std::string& name, std::function<double()> getter) {
  for (const auto& e : dataEntries) {
    if (e.name == name) return;
  }
  dataEntries.push_back({name, std::move(getter)});
}

void GuiDebug::SetDataRegister(const std::function<void()>& Register) {
  dataRegister = Register;
}

int GuiDebug::InvokeSelectedAuton() {
  if (selectedAutonInvoker) return selectedAutonInvoker();
  if (selectedAuton.routine) return selectedAuton.routine();
  return 0;
}

// ============================================================================
// Live Graph Methods
// ============================================================================

void GuiDebug::SetGraphDataProviders(std::function<double()> getX, std::function<double()> getY) {
  graphGetX = getX;
  graphGetY = getY;
}

void GuiDebug::AddGraphPoint(double x, double y) {
  graphBuffer[graphBufferIndex].x = x;
  graphBuffer[graphBufferIndex].y = y;
  graphBufferIndex = (graphBufferIndex + 1) % GRAPH_BUFFER_SIZE;
  
  // Auto-scale if enabled
  if (graphAutoScale) {
    graphMinX = graphMaxX = graphBuffer[0].x;
    graphMinY = graphMaxY = graphBuffer[0].y;
    for (int i = 0; i < GRAPH_BUFFER_SIZE; i++) {
      if (graphBuffer[i].x < graphMinX) graphMinX = graphBuffer[i].x;
      if (graphBuffer[i].x > graphMaxX) graphMaxX = graphBuffer[i].x;
      if (graphBuffer[i].y < graphMinY) graphMinY = graphBuffer[i].y;
      if (graphBuffer[i].y > graphMaxY) graphMaxY = graphBuffer[i].y;
    }
    // Add 10% padding
    double xPad = (graphMaxX - graphMinX) * 0.1;
    double yPad = (graphMaxY - graphMinY) * 0.1;
    graphMinX -= xPad;
    graphMaxX += xPad;
    graphMinY -= yPad;
    graphMaxY += yPad;
  }
}

// ============================================================================
// Debug Display Methods (Delegated to Subsystems)
// ============================================================================

void GuiDebug::DisplayDebugMenu() {
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
  int btnWidth = 100, btnHeight = 55, gap = 10;
  int startX = 20, startY = 65;
  
  struct BtnInfo {
    const char* text1;
    const char* text2;
    int col, row;
  };
  
  BtnInfo buttons[] = {
    {"Registered", "Autons", 0, 0},
    {"Live", "Graph", 1, 0},
    {"Variables", "", 2, 0},
    {"Auton", "Runner", 0, 1},
    {"Data", "", 1, 1}
  };
  
  for (int i = 0; i < 5; ++i) {
    int x = startX + buttons[i].col * (btnWidth + gap);
    int y = startY + buttons[i].row * (btnHeight + gap);
    
    pros::screen::set_eraser(COLOR_LIGHT_GRAY);
    pros::screen::erase_rect(x, y, x + btnWidth, y + btnHeight);
    pros::screen::set_pen(COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_SMALL, x + 5, y + 8, buttons[i].text1);
    if (buttons[i].text2[0] != '\0') {
      pros::screen::print(pros::E_TEXT_SMALL, x + 5, y + 28, buttons[i].text2);
    }
  }
}



// ============================================================================
// Main Menu Touch Handler Override
// ============================================================================

void GuiDebug::HandleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) {
  if (TESTING_AUTONOMOUS) {
    // Debug mode: buttons split in half
    // Check if the "AUTONS" button is pressed (bottom-left corner)
    if (touchStatus.x < BRAIN_SCREEN_WIDTH / 2 && touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
      if (CurrentScreen != AutonMenu) {
        pros::screen::set_eraser(COLOR_BLACK);
        pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT);

        DisplayAutonMenu();
        CurrentScreen = AutonMenu;
      }
    }
    // Check if the "DEBUG" button is pressed (bottom-right corner)
    else if (touchStatus.x > BRAIN_SCREEN_WIDTH / 2 && touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
      pros::screen::set_eraser(COLOR_BLACK);
      pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);
      
      DisplayDebugMenu();
      CurrentScreen = DebugMenu;
      pros::delay(300);
    }
  } else {
    // Non-debug mode: autons button takes full width
    // Check if the "AUTONS" button is pressed (full width bottom)
    if (touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
      if (CurrentScreen != AutonMenu) {
        pros::screen::set_eraser(COLOR_BLACK);
        pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);

        DisplayAutonMenu();
        CurrentScreen = AutonMenu;
      }
    }
  }
}

// ============================================================================
// Debug Touch Handlers (Delegated to Subsystems)
// ============================================================================

void GuiDebug::HandleDebugMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // BACK button
    if (x >= 10 && x <= 90 && y >= 10 && y <= 40) {
      DisplayMainMenu();
      CurrentScreen = MainMenu;
      pros::delay(300);
      return;
    }

    int btnWidth = 100, btnHeight = 55, gap = 10;
    int startX = 20, startY = 65;
    struct BtnPos { int col, row, index; };
    BtnPos buttons[] = {
      {0, 0, 0}, // Registered Autons
      {1, 0, 1}, // Live Graph
      {2, 0, 3}, // Variables
      {0, 1, 2}, // Auton Runner
      {1, 1, 4}  // Odometry
    };
    
    for (int i = 0; i < 5; ++i) {
      int btnX = startX + buttons[i].col * (btnWidth + gap);
      int btnY = startY + buttons[i].row * (btnHeight + gap);
      
      if (x >= btnX && x <= btnX + btnWidth && y >= btnY && y <= btnY + btnHeight) {
        switch (buttons[i].index) {
          case 0: ::aon::DisplayRegisteredAutonsMenu(this); CurrentScreen = RegisteredFunctions; break;
          case 1: ::aon::DisplayLiveGraph(this); CurrentScreen = LiveGraph; break;
          case 2: ::aon::DisplayAutonRunner(this); CurrentScreen = AutonRunner; break;
          case 3: PreviousScreen = DebugMenu; ::aon::DisplayVariablesMenu(this); CurrentScreen = VARS; break;
          case 4: ::aon::DisplayDataMenu(this); CurrentScreen = DATA; break;
        }
        pros::delay(400);
        return;
      }
    }
  }
}



// ============================================================================
// Overridden GUI Loop
// ============================================================================

void GuiDebug::RunGuiLoop() {
  bool lastAutonState = false;
  auto lastScreen = CurrentScreen;
  
  while (true) {
    pros::screen_touch_status_s_t TouchStatus = pros::screen::touch_status();
    if (TouchStatus.touch_status > 0) {
      switch (CurrentScreen) {
        case MainMenu:
          HandleMainMenuTouch(TouchStatus);
          break;
        case AutonMenu:
          HandleAutonMenuTouch();
          break;
        case RedAutons:
          HandleRedAutonMenuTouch();
          break;
        case BlueAutons:
          HandleBlueAutonMenuTouch();
          break;
        case SkillAutons:
          HandleSkillsMenuTouch();
          break;
        case DebugMenu:
          HandleDebugMenuTouch();
          break;
        case RegisteredFunctions:
          ::aon::HandleRegisteredAutonsMenuTouch(this);
          break;
        case AutonRunner:
          ::aon::HandleAutonRunnerTouch(this);
          break;
        case VARS:
          ::aon::HandleVariablesMenuTouch(this);
          break;
        case DATA:
          ::aon::HandleDataMenuTouch(this);
          break;
        case LiveGraph:
          ::aon::HandleLiveGraphTouch(this);
          break;
        default:
          break;
      }
    }

    pros::delay(30);

    // Redraw only when necessary
    bool screenChanged = (CurrentScreen != lastScreen);
    bool autonStateChanged = (autonRunning != lastAutonState);
    
    if (screenChanged) {
      lastScreen = CurrentScreen;
      // Screen was changed by handler, it already redraws
    }
    
    if (autonStateChanged) {
      lastAutonState = autonRunning;
      // Redraw AutonRunner when auton state changes
      if (CurrentScreen == AutonRunner) {
        ::aon::DisplayAutonRunner(this);
      }
    }
    
    // For screens with real-time updates, refresh periodically
    static int refreshCounter = 0;
    if (++refreshCounter >= 10) {  // Every 300ms
      if (CurrentScreen == DATA) {
        ::aon::DisplayDataMenu(this);
      } else if (CurrentScreen == LiveGraph) {
        ::aon::DisplayLiveGraph(this);
        if (graphGetX && graphGetY) {
          AddGraphPoint(graphGetX(), graphGetY());
        }
      }
      refreshCounter = 0;
    }
  }
}

// ============================================================================
// Main Menu Override - Conditional Button Sizing
// ============================================================================

void GuiDebug::DisplayMainMenu() {
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2, (BRAIN_SCREEN_HEIGHT - 225) / 4);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE); // Default color for "NO AUTON"
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::set_pen(COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  if (TESTING_AUTONOMOUS) {
    // Debug mode: split button bar in half
    // Draw the "AUTONS" button in the bottom-left corner (half width)
    pros::screen::set_eraser(COLOR_GREEN);
    pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, 60, BRAIN_SCREEN_HEIGHT - 40, "AUTONS");

    // Draw the "DEBUG" button in the bottom-right corner (half width)
    pros::screen::set_eraser(COLOR_GRAY);
    pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 + 70, BRAIN_SCREEN_HEIGHT - 40, "DEBUG");
  } else {
    // Non-debug mode: auton button takes full width
    pros::screen::set_eraser(COLOR_GREEN);
    pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_LARGE, BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 40, "AUTONS");
  }
}

// ============================================================================
// Initialize
// ============================================================================

void GuiDebug::Initialize() {
  // Call base class initialization first
  Gui::Initialize();

  // Seed test functions at startup if a register is provided
  if (testRegister) {
    testRegister();
  }

  // Seed variable changer entries if a register is provided
  if (variableRegister) {
    variableRegister();
  }

  // Seed data entries if a register is provided
  if (dataRegister) {
    dataRegister();
  }
}

}  // namespace aon
