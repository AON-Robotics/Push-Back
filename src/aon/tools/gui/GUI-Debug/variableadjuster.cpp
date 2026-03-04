#include "../../../../../include/aon/tools/gui/gui-debug.hpp"

namespace aon {

struct Btn { int x1, y1, x2, y2; const char* text; double delta; };

// Pagination state for variables menu
static int variablesPage = 0;
static constexpr int VARS_PER_PAGE = 2;

void GuiDebug::DisplayVariablesMenu() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "VARIABLES");

  // BACK button
  const int backY1 = 10, backY2 = backY1 + 30;
  const int backX2 = BRAIN_SCREEN_WIDTH / 2 - 110, backX1 = backX2 - 80;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 8, backY1 + 8, "BACK");

  // MENU button
  const int menuY1 = 10, menuY2 = menuY1 + 30;
  const int menuX1 = BRAIN_SCREEN_WIDTH - 120, menuX2 = BRAIN_SCREEN_WIDTH - 40;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(menuX1, menuY1, menuX2, menuY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, menuX1 + 6, menuY1 + 8, "MENU");

  // Seed variables if needed
  if (variableEntries.empty() && variableRegister) variableRegister();

  if (variableEntries.empty()) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "No Variables Registered");
    return;
  }

  // Calculate pagination
  int totalPages = (variableEntries.size() + VARS_PER_PAGE - 1) / VARS_PER_PAGE;
  if (variablesPage >= totalPages) variablesPage = totalPages - 1;
  if (variablesPage < 0) variablesPage = 0;

  int startIdx = variablesPage * VARS_PER_PAGE;
  int endIdx = std::min(startIdx + VARS_PER_PAGE, (int)variableEntries.size());

  // Page indicator
  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  char pageText[32];
  snprintf(pageText, sizeof(pageText), "Page %d/%d", variablesPage + 1, totalPages);
  pros::screen::print(pros::E_TEXT_SMALL, BRAIN_SCREEN_WIDTH / 2 - 30, BRAIN_SCREEN_HEIGHT - 20, pageText);

  // PREV button (bottom left)
  if (variablesPage > 0) {
    const int prevX1 = 20, prevY1 = BRAIN_SCREEN_HEIGHT - 35;
    const int prevX2 = prevX1 + 80, prevY2 = prevY1 + 30;
    pros::screen::set_eraser(COLOR_DARK_GRAY);
    pros::screen::erase_rect(prevX1, prevY1, prevX2, prevY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, prevX1 + 12, prevY1 + 8, "PREV");
  }

  // NEXT button (bottom right)
  if (variablesPage < totalPages - 1) {
    const int nextX1 = BRAIN_SCREEN_WIDTH - 100, nextY1 = BRAIN_SCREEN_HEIGHT - 35;
    const int nextX2 = nextX1 + 80, nextY2 = nextY1 + 30;
    pros::screen::set_eraser(COLOR_DARK_GRAY);
    pros::screen::erase_rect(nextX1, nextY1, nextX2, nextY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, nextX1 + 12, nextY1 + 8, "NEXT");
  }

  int y = 50;
  for (int i = startIdx; i < endIdx; ++i) {
    const auto& e = variableEntries[i];
    
    // Name and current value
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_MEDIUM, 20, y, e.name.c_str());
    pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH - 140, y, "%0.3f", e.get());

    // Buttons: -10, -1, -0.1 | +0.1, +1, +10
    Btn btns[6];
    int bx = 20, by = y + 20, bw = 70, bh = 30, gap = 10;
    
    // Left side (decrements)
    btns[0] = {bx, by, bx + bw, by + bh, "-10", -10.0}; bx += bw + gap;
    btns[1] = {bx, by, bx + bw, by + bh, "-1", -1.0}; bx += bw + gap;
    btns[2] = {bx, by, bx + bw, by + bh, "-0.1", -0.1};
    
    // Right side (increments)
    bx = BRAIN_SCREEN_WIDTH - (bw * 3 + gap * 2) - 20;
    btns[3] = {bx, by, bx + bw, by + bh, "+0.1", +0.1}; bx += bw + gap;
    btns[4] = {bx, by, bx + bw, by + bh, "+1", +1.0}; bx += bw + gap;
    btns[5] = {bx, by, bx + bw, by + bh, "+10", +10.0};

    for (int b = 0; b < 6; ++b) {
      pros::screen::set_eraser(btns[b].delta < 0 ? COLOR_DARK_RED : COLOR_DARK_GREEN);
      pros::screen::erase_rect(btns[b].x1, btns[b].y1, btns[b].x2, btns[b].y2);
      pros::screen::set_pen(COLOR_WHITE);
      pros::screen::print(pros::E_TEXT_MEDIUM, btns[b].x1 + 10, btns[b].y1 + 8, btns[b].text);
    }

    y += 70;
  }
}

void GuiDebug::HandleVariablesMenuTouch() {
  static uint32_t lastTouchMs = 0;
  const auto touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;
  
  uint32_t now = pros::millis();
  if (now - lastTouchMs < 300) {
    return;
  }

  // MENU button
  {
    const int menuY1 = 10, menuY2 = menuY1 + 30;
    const int menuX1 = BRAIN_SCREEN_WIDTH - 120, menuX2 = BRAIN_SCREEN_WIDTH - 40;
    if (touch.x >= menuX1 && touch.x <= menuX2 && touch.y >= menuY1 && touch.y <= menuY2) {
      DisplayMainMenu();
      CurrentScreen = MainMenu;
      variablesPage = 0;  // Reset page when leaving
      lastTouchMs = now;
      return;
    }
  }

  // BACK button
  {
    const int backY1 = 10, backY2 = backY1 + 30;
    const int backX2 = BRAIN_SCREEN_WIDTH / 2 - 110, backX1 = backX2 - 80;
    if (touch.x >= backX1 && touch.x <= backX2 && touch.y >= backY1 && touch.y <= backY2) {
      CurrentScreen = PreviousScreen;
      variablesPage = 0;  // Reset page when leaving
      if (PreviousScreen == DebugMenu) DisplayDebugMenu();
      else if (PreviousScreen == AutonRunner) DisplayAutonRunner();
      lastTouchMs = now;
      pros::delay(300);
      return;
    }
  }

  // PREV button
  {
    int totalPages = (variableEntries.size() + VARS_PER_PAGE - 1) / VARS_PER_PAGE;
    if (variablesPage > 0) {
      const int prevX1 = 20, prevY1 = BRAIN_SCREEN_HEIGHT - 35;
      const int prevX2 = prevX1 + 80, prevY2 = prevY1 + 30;
      if (touch.x >= prevX1 && touch.x <= prevX2 && touch.y >= prevY1 && touch.y <= prevY2) {
        variablesPage--;
        DisplayVariablesMenu();
        lastTouchMs = now;
        return;
      }
    }

    // NEXT button
    if (variablesPage < totalPages - 1) {
      const int nextX1 = BRAIN_SCREEN_WIDTH - 100, nextY1 = BRAIN_SCREEN_HEIGHT - 35;
      const int nextX2 = nextX1 + 80, nextY2 = nextY1 + 30;
      if (touch.x >= nextX1 && touch.x <= nextX2 && touch.y >= nextY1 && touch.y <= nextY2) {
        variablesPage++;
        DisplayVariablesMenu();
        lastTouchMs = now;
        return;
      }
    }
  }

  // Calculate which variables are on this page
  int startIdx = variablesPage * VARS_PER_PAGE;
  int endIdx = std::min(startIdx + VARS_PER_PAGE, (int)variableEntries.size());

  int y = 50;
  for (int i = startIdx; i < endIdx; ++i) {
    auto& e = variableEntries[i];
    Btn btns[6];
    int bx = 20, by = y + 20, bw = 70, bh = 30, gap = 10;
    
    btns[0] = {bx, by, bx + bw, by + bh, "-10", -10.0}; bx += bw + gap;
    btns[1] = {bx, by, bx + bw, by + bh, "-1",  -1.0};  bx += bw + gap;
    btns[2] = {bx, by, bx + bw, by + bh, "-0.1", -0.1};
    
    bx = BRAIN_SCREEN_WIDTH - (bw * 3 + gap * 2) - 20;
    btns[3] = {bx, by, bx + bw, by + bh, "+0.1", +0.1};  bx += bw + gap;
    btns[4] = {bx, by, bx + bw, by + bh, "+1",   +1.0};  bx += bw + gap;
    btns[5] = {bx, by, bx + bw, by + bh, "+10",  +10.0};

    for (int b = 0; b < 6; ++b) {
      if (touch.x >= btns[b].x1 && touch.x <= btns[b].x2 && 
          touch.y >= btns[b].y1 && touch.y <= btns[b].y2) {
        if (e.apply) {
          e.apply(btns[b].delta);
          DisplayVariablesMenu();
          lastTouchMs = now;
        }
        return;
      }
    }
    y += 70;
  }
}

}  // namespace aon
