#include "../../../../../include/aon/tools/gui/gui-debug.hpp"

namespace aon {

// Pagination state for data menu
static int dataPage = 0;
static constexpr int DATA_PER_PAGE = 6;

void GuiDebug::DisplayDataMenu() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Title
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "DATA");

  // BACK button (top left)
  const int backX1 = 10, backY1 = 10, backX2 = 80, backY2 = 38;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 8, backY1 + 6, "BACK");

  // VARS button
  const int varsX1 = BRAIN_SCREEN_WIDTH - 180, varsY1 = 10;
  const int varsX2 = varsX1 + 70, varsY2 = varsY1 + 28;
  pros::screen::set_eraser(COLOR_ORANGE);
  pros::screen::erase_rect(varsX1, varsY1, varsX2, varsY2);
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::print(pros::E_TEXT_MEDIUM, varsX1 + 10, varsY1 + 6, "VARS");

  // RESET button (top right)
  const int resetX1 = BRAIN_SCREEN_WIDTH - 90, resetY1 = 10;
  const int resetX2 = resetX1 + 80, resetY2 = resetY1 + 28;
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase_rect(resetX1, resetY1, resetX2, resetY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, resetX1 + 6, resetY1 + 6, "RESET");

  if (dataEntries.empty() && dataRegister) dataRegister();

  if (dataEntries.empty()) {
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "No Data Registered");
    return;
  }

  // Display total number of entries
  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  char totalEntriesText[32];
  snprintf(totalEntriesText, sizeof(totalEntriesText), "Total Entries: %d", (int)dataEntries.size());
  pros::screen::print(pros::E_TEXT_SMALL, 10, 50, totalEntriesText);

  // Calculate pagination
  int totalPages = ((int)dataEntries.size() + DATA_PER_PAGE - 1) / DATA_PER_PAGE;
  if (dataPage >= totalPages) dataPage = totalPages - 1;
  if (dataPage < 0) dataPage = 0;

  int startIdx = dataPage * DATA_PER_PAGE;
  int endIdx = std::min(startIdx + DATA_PER_PAGE, (int)dataEntries.size());

  // Page indicator
  if (totalPages > 1) {
    pros::screen::set_pen(COLOR_LIGHT_GRAY);
    char pageText[32];
    snprintf(pageText, sizeof(pageText), "Page %d/%d", dataPage + 1, totalPages);
    pros::screen::print(pros::E_TEXT_SMALL, BRAIN_SCREEN_WIDTH / 2 - 30, 28, pageText);
  }

  // Display data entries
  int y = 70; // Adjusted to avoid overlap with total entries text
  for (int i = startIdx; i < endIdx; ++i) {
    const auto& entry = dataEntries[i];
    double value = entry.getter ? entry.getter() : 0.0;

    // Build display strings via snprintf (matches VariableAdjuster pattern)
    char nameStr[64];
    char valueStr[32];
    snprintf(nameStr, sizeof(nameStr), "%s:", entry.name.c_str());
    snprintf(valueStr, sizeof(valueStr), "%.3f", value);

    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, nameStr);
    pros::screen::set_pen(COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH - 140, y, valueStr);

    y += 28;
  }

  // PREV button (bottom left)
  if (dataPage > 0) {
    const int prevX1 = 20, prevY1 = BRAIN_SCREEN_HEIGHT - 35;
    const int prevX2 = prevX1 + 80, prevY2 = prevY1 + 30;
    pros::screen::set_eraser(COLOR_DARK_GRAY);
    pros::screen::erase_rect(prevX1, prevY1, prevX2, prevY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, prevX1 + 12, prevY1 + 8, "PREV");
  }

  // NEXT button (bottom right)
  if (dataPage < totalPages - 1) {
    const int nextX1 = BRAIN_SCREEN_WIDTH - 100, nextY1 = BRAIN_SCREEN_HEIGHT - 35;
    const int nextX2 = nextX1 + 80, nextY2 = nextY1 + 30;
    pros::screen::set_eraser(COLOR_DARK_GRAY);
    pros::screen::erase_rect(nextX1, nextY1, nextX2, nextY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, nextX1 + 12, nextY1 + 8, "NEXT");
  }
}

void GuiDebug::HandleDataMenuTouch() {
  static uint32_t lastTouchMs = 0;
  const auto touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  uint32_t now = pros::millis();
  if (now - lastTouchMs < 300) return;

  int x = touch.x;
  int y = touch.y;

  // BACK button
  {
    const int backX1 = 10, backY1 = 10, backX2 = 80, backY2 = 38;
    if (x >= backX1 && x <= backX2 && y >= backY1 && y <= backY2) {
      DisplayDebugMenu();
      currentScreen = DebugMenu;
      dataPage = 0;
      lastTouchMs = now + 300; // Add extra delay to prevent double-click
      pros::delay(300);
      return;
    }
  }

  // VARS button
  {
    const int varsX1 = BRAIN_SCREEN_WIDTH - 180, varsY1 = 10;
    const int varsX2 = varsX1 + 70, varsY2 = varsY1 + 28;
    if (x >= varsX1 && x <= varsX2 && y >= varsY1 && y <= varsY2) {
      if (variableEntries.empty() && variableRegister) {
        variableRegister();
      }
      previousScreen = DATA;
      DisplayVariablesMenu();
      currentScreen = VARS;
      lastTouchMs = now;
      return;
    }
  }

  // RESET button - resets set variable to indicated value (if user has registered a handler for this)
  {
    const int resetX1 = BRAIN_SCREEN_WIDTH - 90, resetY1 = 10;
    const int resetX2 = resetX1 + 80, resetY2 = resetY1 + 28;
    if (x >= resetX1 && x <= resetX2 && y >= resetY1 && y <= resetY2) {
      // Invoke the user-registered reset handler if available
      invokeResetHandler();

      DisplayDataMenu();
      lastTouchMs = now;
      return;
    }
  }
  // Pagination - PREV/NEXT
  {
    int totalPages = ((int)dataEntries.size() + DATA_PER_PAGE - 1) / DATA_PER_PAGE;

    // PREV button
    if (dataPage > 0) {
      const int prevX1 = 20, prevY1 = BRAIN_SCREEN_HEIGHT - 35;
      const int prevX2 = prevX1 + 80, prevY2 = prevY1 + 30;
      if (x >= prevX1 && x <= prevX2 && y >= prevY1 && y <= prevY2) {
        dataPage--;
        DisplayDataMenu();
        lastTouchMs = now;
        return;
      }
    }

    // NEXT button
    if (dataPage < totalPages - 1) {
      const int nextX1 = BRAIN_SCREEN_WIDTH - 100, nextY1 = BRAIN_SCREEN_HEIGHT - 35;
      const int nextX2 = nextX1 + 80, nextY2 = nextY1 + 30;
      if (x >= nextX1 && x <= nextX2 && y >= nextY1 && y <= nextY2) {
        dataPage++;
        DisplayDataMenu();
        lastTouchMs = now;
        return;
      }
    }
  }
}

}  // namespace aon
