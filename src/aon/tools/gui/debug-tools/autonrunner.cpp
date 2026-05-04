#include "../../../../../include/aon/tools/gui/gui-debug.hpp"
#include <cmath>
#include <string>

namespace aon {

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void GuiDebug::DisplayRegisteredAutonsMenu() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Header
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH / 2 - 100, 10,
                      "Registered Autons");

  // BACK button
  int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 10, backY1 + 6, "BACK");

  // NEXT button
  int nextY1 = 6, nextY2 = nextY1 + 28;
  int nextX1 = BRAIN_SCREEN_WIDTH - 120, nextX2 = BRAIN_SCREEN_WIDTH - 40;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(nextX1, nextY1, nextX2, nextY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, nextX1 + 12, nextY1 + 6, "NEXT");

  // Header underline
  pros::screen::set_eraser(COLOR_LIGHT_GRAY);
  pros::screen::erase_rect(0, 42, BRAIN_SCREEN_WIDTH, 44);

  // Seed test functions if needed
  if (testFunctions.empty() && testRegister) {
    testRegister();
  }

  if (testFunctions.empty()) {
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3,
                        "No Test Functions Registered");
    return;
  }

  // Display buttons for each registered test function
  int yOffset = 70;
  for (size_t i = 0; i < testFunctions.size(); ++i) {
    const auto& [name, _] = testFunctions[i];

    pros::screen::set_eraser(COLOR_LIGHT_GRAY);
    pros::screen::erase_rect(50, yOffset, BRAIN_SCREEN_WIDTH - 50,
                             yOffset + 40);
    pros::screen::set_pen(COLOR_BLACK);
    pros::screen::print(pros::E_TEXT_MEDIUM, 60, yOffset + 10, name.c_str());

    yOffset += 50;
  }
}


void GuiDebug::DisplayAutonRunner() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  // aon::drawSafeCourners();

  // Header
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH / 2 - 70, 10,
                      "Auton Runner");

  // Back button
  const int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 10, backY1 + 6, "BACK");

  // Menu button
  const int menuY1 = 6, menuY2 = menuY1 + 28;
  const int menuX1 = BRAIN_SCREEN_WIDTH - 120, menuX2 = BRAIN_SCREEN_WIDTH - 40;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(menuX1, menuY1, menuX2, menuY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, menuX1 + 6, menuY1 + 6, "MENU");

  // Header underline
  pros::screen::set_eraser(COLOR_LIGHT_GRAY);
  pros::screen::erase_rect(0, 42, BRAIN_SCREEN_WIDTH, 44);

  // Selected auton panel - check both debug test functions AND normal autons
  const bool hasDebugAuton = static_cast<bool>(selectedAutonInvoker);
  const bool hasNormalAuton = (selectedAuton.routine != nullptr);
  const bool hasAuton = hasDebugAuton || hasNormalAuton;
  const int cardX1 = 16, cardY1 = 50, cardX2 = BRAIN_SCREEN_WIDTH - 160,
            cardY2 = 145;

  // Border
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(cardX1, cardY1, cardX2, cardY2);

  // Inner
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase_rect(cardX1 + 2, cardY1 + 2, cardX2 - 2, cardY2 - 2);

  // Label and name
  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_MEDIUM, cardX1 + 10, cardY1 + 8,
                      "Selected:");

  // Show different states: running, completed, or selected/none
  if (autonRunning) {
    pros::screen::set_pen(COLOR_ORANGE);
    pros::screen::print(pros::E_TEXT_LARGE, cardX1 + 14, cardY1 + 48,
                        selectedAutonName.c_str());
  } else if (autonCompleted) {
    pros::screen::set_pen(COLOR_CYAN);
    pros::screen::print(pros::E_TEXT_LARGE, cardX1 + 14, cardY1 + 48,
                        "COMPLETED");
  } else {
    pros::screen::set_pen(hasAuton ? COLOR_GREEN : COLOR_RED);
    const char* nameText =
        hasAuton ? selectedAutonName.c_str() : "NO AUTON";
    pros::screen::print(pros::E_TEXT_LARGE, cardX1 + 14, cardY1 + 48, nameText);
  }

  // VARS button
  const int varsY1 = 70, varsY2 = varsY1 + 40;
  const int varsX1 = BRAIN_SCREEN_WIDTH - 140, varsX2 = BRAIN_SCREEN_WIDTH - 40;
  pros::screen::set_eraser(COLOR_ORANGE);
  pros::screen::erase_rect(varsX1, varsY1, varsX2, varsY2);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_MEDIUM, varsX1 + 18, varsY1 + 8, "VARS");

  // RESET button
  const int resetY1 = varsY2 + 10, resetY2 = resetY1 + 40;
  const int resetX1 = varsX1, resetX2 = varsX2;
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase_rect(resetX1, resetY1, resetX2, resetY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, resetX1 + 8, resetY1 + 8, "RESET");

  // Bottom RUN/MOV button - centered
  const int btnY1 = BRAIN_SCREEN_HEIGHT - 70;
  const int btnY2 = BRAIN_SCREEN_HEIGHT - 20;
  const int btnWidth = 125;
  const int runX1 = (BRAIN_SCREEN_WIDTH - btnWidth) / 2;
  const int runX2 = runX1 + btnWidth;

  // Border
  pros::screen::set_eraser(COLOR_ANTIQUE_WHITE);
  pros::screen::erase_rect(runX1 - 2, btnY1 - 2, runX2 + 2, btnY2 + 2);

  // Button color based on state
  if (autonRunning) {
    pros::screen::set_eraser(COLOR_RED);  // Red MOV button when running
  } else if (hasAuton) {
    pros::screen::set_eraser(COLOR_GREEN);  // Green RUN button when ready
  } else {
    pros::screen::set_eraser(COLOR_DARK_GRAY);  // Gray when no auton
  }
  pros::screen::erase_rect(runX1, btnY1, runX2, btnY2);

  // RUN/MOV text
  pros::screen::set_pen(COLOR_WHITE);
  const char* runBtnText = autonRunning ? "MOV" : "RUN";
  const int runTextX = runX1 + (btnWidth / 2) - (autonRunning ? 25 : 29);
  pros::screen::print(pros::E_TEXT_LARGE, runTextX, btnY1 + 13, runBtnText);
}


// ============================================================================
// TOUCH HANDLERS
// ============================================================================

void GuiDebug::HandleRegisteredAutonsMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();

  if (touch.touch_status <= 0) return;

  int x = touch.x;
  int y = touch.y;

  // BACK button
  {
    int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
    if (x >= backX1 && x <= backX2 && y >= backY1 && y <= backY2) {
      DisplayDebugMenu();
      currentScreen = DebugMenu;
      pros::delay(300);
      return;
    }
  }

  // NEXT button
  int nextY1 = 6, nextY2 = nextY1 + 28;
  int nextX1 = BRAIN_SCREEN_WIDTH - 120, nextX2 = BRAIN_SCREEN_WIDTH - 40;
  if (x >= nextX1 && x <= nextX2 && y >= nextY1 && y <= nextY2) {
    DisplayAutonRunner();
    currentScreen = AutonRunner;
    pros::delay(300);
    return;
  }

  // Check which test function button is pressed
  int yOffset = 70;
  for (size_t i = 0; i < testFunctions.size(); ++i) {
    if (x >= 50 && x <= BRAIN_SCREEN_WIDTH - 50 && y >= yOffset &&
        y <= yOffset + 40) {
      const auto& [name, fn] = testFunctions[i];
      selectedAutonName = name;
      selectedAutonInvoker = fn;
      autonCompleted = false;  // Reset completed flag on new selection
      
      autonomousReader->AddFunction("autonomous", [this]{ return invokeSelectedAuton(); });

      DisplayAutonRunner();
      currentScreen = AutonRunner;
      pros::delay(300);
      return;
    }
    yOffset += 50;
  }
}

void GuiDebug::HandleAutonRunnerTouch() {
  const auto touchStatus = pros::screen::touch_status();
  if (touchStatus.touch_status <= 0) return;

  int x = touchStatus.x;
  int y = touchStatus.y;

  // BACK button
  {
    const int backX1 = 10, backY1 = 6, backX2 = backX1 + 80,
              backY2 = backY1 + 28;
    if (x >= backX1 && x <= backX2 && y >= backY1 && y <= backY2) {
      DisplayDebugMenu();
      currentScreen = DebugMenu;
      pros::delay(300);
      return;
    }
  }

  // MENU button
  {
    const int menuY1 = 6, menuY2 = menuY1 + 28;
    const int menuX1 = BRAIN_SCREEN_WIDTH - 120, menuX2 = BRAIN_SCREEN_WIDTH - 40;
    if (x >= menuX1 && x <= menuX2 && y >= menuY1 && y <= menuY2) {
      displayMainMenu();
      currentScreen = MainMenu;
      pros::delay(300);
      return;
    }
  }

  // VARS button
  {
    const int varsY1 = 70, varsY2 = varsY1 + 40;
    const int varsX1 = BRAIN_SCREEN_WIDTH - 140, varsX2 = BRAIN_SCREEN_WIDTH - 40;
    if (x >= varsX1 && x <= varsX2 && y >= varsY1 && y <= varsY2) {
      if (variableEntries.empty() && variableRegister) {
        variableRegister();
      }
      previousScreen = AutonRunner;
      DisplayVariablesMenu();
      currentScreen = VARS;
      pros::delay(300);
      return;
    }
  }

    // RESET button
  {
    const int varsY2 = 70 + 40;
    const int resetY1 = varsY2 + 10, resetY2 = resetY1 + 40;
    const int resetX1 = BRAIN_SCREEN_WIDTH - 140, resetX2 = BRAIN_SCREEN_WIDTH - 40;
    if (x >= resetX1 && x <= resetX2 && y >= resetY1 && y <= resetY2) {
      invokeResetHandler();
      DisplayAutonRunner();
      pros::delay(300);
      return;
    }
  }

  
  // Bottom RUN/MOV button coordinates (must match display)
  const int btnY1 = BRAIN_SCREEN_HEIGHT - 70;
  const int btnY2 = BRAIN_SCREEN_HEIGHT - 20;
  const int btnWidth = 150;
  const int runX1 = (BRAIN_SCREEN_WIDTH - btnWidth) / 2;
  const int runX2 = runX1 + btnWidth;

  // Check if touch is in button area
  if (y < btnY1 || y > btnY2) {
    return;  // Not in button row
  }

  // Check RUN/MOV button
  if (x < runX1 || x > runX2) {
    return;  // Not on RUN button
  }


  // RUN/MOV button
  bool hasAuton = (selectedAuton.routine != nullptr) ||
                  static_cast<bool>(selectedAutonInvoker);
  if (!hasAuton) return;

  autonomousReader->AddFunction("autonomous", [this]{ return invokeSelectedAuton(); });
  autonRunning = true;
  autonCompleted = false;
  DisplayAutonRunner();  // show orange running state

  autonomousReader->ExecuteFunction("autonomous");

  autonRunning = false;
  autonCompleted = true;
  DisplayAutonRunner();  // show cyan completed state

  pros::delay(300);
}

}  // namespace aon
