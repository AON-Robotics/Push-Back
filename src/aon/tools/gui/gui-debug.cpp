#include "../../../../include/aon/tools/gui/gui-debug.hpp"
namespace aon {

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

void GuiDebug::registerTestFunction(int (*func)(), const std::string& name) {
  AddTestFunctionInternal(name, func);
}

void GuiDebug::registerTestFunction(const std::function<int()>& func, const std::string& name) {
  AddTestFunctionInternal(name, func);
}

void GuiDebug::registerTestFunction(void (*func)(), const std::string& name) {
  AddTestFunctionInternal(name, [func]() -> int {
    if (func) func();
    return 0;
  });
}

void GuiDebug::setVariableRegister(const std::function<void()>& Register) {
  variableRegister = Register;
}

void GuiDebug::setTestRegister(const std::function<void()>& Register) {
  testRegister = Register;
}

void GuiDebug::registerDataEntry(const std::string& name, std::function<double()> getter) {
  for (const auto& e : dataEntries) {
    if (e.name == name) return;
  }
  dataEntries.push_back({name, std::move(getter)});
}

void GuiDebug::setDataRegister(const std::function<void()>& Register) {
  dataRegister = Register;
}

int GuiDebug::invokeSelectedAuton() {
  if (selectedAutonInvoker) return selectedAutonInvoker();
  if (selectedAuton.routine) return selectedAuton.routine();
  return 0;
}

// ============================================================================
// Live Graph Methods
// ============================================================================

void GuiDebug::setGraphDataProviders(std::function<double()> getX, std::function<double()> getY) {
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
// Field Mapper Methods
// ============================================================================

void GuiDebug::setMapDataProvider(std::function<Pose()> getPose) {
  mapGetPose = std::move(getPose);
}

void GuiDebug::AddMapPoint(double x, double y, double theta) {
  if (mapBufferCount >= MAP_BUFFER_SIZE) return;
  if (mapBufferCount > 0) {
    const auto& prev = mapBuffer[mapBufferCount - 1];
    double dx = x - prev.x;
    double dy = y - prev.y;
    mapTotalDist += std::sqrt(dx * dx + dy * dy);
  }
  mapBuffer[mapBufferCount++] = {x, y, theta};
}

void GuiDebug::ClearMapPath() {
  mapBufferCount = 0;
  mapTotalDist   = 0.0;
  arcStartIndex  = -1;
  dispEndIndex   = -1;
  arcMeasured    = false;
  arcResult      = {};
  mapMode        = MapMode::SELECT;
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
  const int cols = 3, rows = 2, gap = 10;
  const int startX = 20, startY = 65;
  const int btnWidth = (BRAIN_SCREEN_WIDTH - 2 * startX - (cols - 1) * gap) / cols;
  const int btnHeight = (BRAIN_SCREEN_HEIGHT - startY - startX - (rows - 1) * gap) / rows;
  
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
    {"Data", "", 1, 1},
    {"Field", "Map", 2, 1},
  };
  
  for (int i = 0; i < 6; ++i) {
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

void GuiDebug::handleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) {
  if (TESTING_AUTONOMOUS) {
    // Debug mode: buttons split in half
    // Check if the "AUTONS" button is pressed (bottom-left corner)
    if (touchStatus.x < BRAIN_SCREEN_WIDTH / 2 && touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
      if (currentScreen != AutonMenu) {
        pros::screen::set_eraser(COLOR_BLACK);
        pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT);

        displayAutonMenu();
        currentScreen = AutonMenu;
      }
    }
    // Check if the "DEBUG" button is pressed (bottom-right corner)
    else if (touchStatus.x > BRAIN_SCREEN_WIDTH / 2 && touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
      pros::screen::set_eraser(COLOR_BLACK);
      pros::screen::erase_rect(BRAIN_SCREEN_WIDTH / 2, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);
      
      DisplayDebugMenu();
      currentScreen = DebugMenu;
      pros::delay(300);
    }
  } else {
    // Non-debug mode: autons button takes full width
    // Check if the "AUTONS" button is pressed (full width bottom)
    if (touchStatus.y > BRAIN_SCREEN_HEIGHT - 50) {
      if (currentScreen != AutonMenu) {
        pros::screen::set_eraser(COLOR_BLACK);
        pros::screen::erase_rect(0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT);

        displayAutonMenu();
        currentScreen = AutonMenu;
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
      displayMainMenu();
      currentScreen = MainMenu;
      pros::delay(300);
      return;
    }

    const int cols = 3, rows = 2, gap = 10;
    const int startX = 20, startY = 65;
    const int btnWidth = (BRAIN_SCREEN_WIDTH - 2 * startX - (cols - 1) * gap) / cols;
    const int btnHeight = (BRAIN_SCREEN_HEIGHT - startY - startX - (rows - 1) * gap) / rows;

    for (int i = 0; i < cols * rows; ++i) {
      int btnX = startX + (i % cols) * (btnWidth + gap);
      int btnY = startY + (i / cols) * (btnHeight + gap);
      
      if (x >= btnX && x <= btnX + btnWidth && y >= btnY && y <= btnY + btnHeight) {
        switch (i) {
          case 0: DisplayRegisteredAutonsMenu(); currentScreen = RegisteredFunctions; break;
          case 1: DisplayLiveGraph(); currentScreen = LiveGraph; break;
          case 2: previousScreen = DebugMenu; DisplayVariablesMenu(); currentScreen = VARS; break;
          case 3: DisplayAutonRunner(); currentScreen = AutonRunner; break;
          case 4: DisplayDataMenu(); currentScreen = DATA; break;
          case 5: DisplayFieldMapper(); currentScreen = FieldMapper; break;
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

void GuiDebug::mainLoop() {
  bool lastAutonState = false;
  auto lastScreen = currentScreen;
  
  while (true) {
    pros::screen_touch_status_s_t TouchStatus = pros::screen::touch_status();
    if (TouchStatus.touch_status > 0) {
      switch (currentScreen) {
        case MainMenu:
          handleMainMenuTouch(TouchStatus);
          break;
        case AutonMenu:
          handleAutonMenuTouch();
          break;
        case RedAutons:
          handleRedAutonMenuTouch();
          break;
        case BlueAutons:
          handleBlueAutonMenuTouch();
          break;
        case SkillAutons:
          handleSkillsMenuTouch();
          break;
        case DebugMenu:
          HandleDebugMenuTouch();
          break;
        case RegisteredFunctions:
          HandleRegisteredAutonsMenuTouch();
          break;
        case AutonRunner:
          HandleAutonRunnerTouch();
          break;
        case VARS:
          HandleVariablesMenuTouch();
          break;
        case DATA:
          HandleDataMenuTouch();
          break;
        case LiveGraph:
          HandleLiveGraphTouch();
          break;
        case FieldMapper:
          HandleFieldMapperTouch();
          break;
        default:
          break;
      }
    }

    pros::delay(30);

    // Redraw only when necessary
    bool screenChanged = (currentScreen != lastScreen);
    bool autonStateChanged = (autonRunning != lastAutonState);
    
    if (screenChanged) {
      lastScreen = currentScreen;
      // Screen was changed by handler, it already redraws
    }
    
    if (autonStateChanged) {
      lastAutonState = autonRunning;
      // Redraw AutonRunner when auton state changes
      if (currentScreen == AutonRunner) {
        DisplayAutonRunner();
      }
    }
    
    // For screens with real-time updates, refresh periodically
    static int refreshCounter = 0;
    if (++refreshCounter >= 10) {  // Every 300ms
      if (currentScreen == DATA) {
        DisplayDataMenu();
      } else if (currentScreen == LiveGraph) {
        DisplayLiveGraph();
        if (graphGetX && graphGetY) {
          AddGraphPoint(graphGetX(), graphGetY());
        }
      } else if (currentScreen == FieldMapper) {
        if (mapGetPose) {
          Pose p = mapGetPose();
          AddMapPoint(p.x, p.y, p.theta);
        }
        DisplayFieldMapper();
      }
      refreshCounter = 0;
    }
  }
}

// ============================================================================
// Main Menu Override - Conditional Button Sizing
// ============================================================================

void GuiDebug::displayMainMenu() {
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

void GuiDebug::initialize() {
  // Call base class initialization first
  Gui::initialize();

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
