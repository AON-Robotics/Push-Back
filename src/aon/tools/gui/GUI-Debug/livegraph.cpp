#include "../../../../../include/aon/tools/gui/gui-v2-debug.hpp"
#include <algorithm>

namespace aon {

void DisplayLiveGraph(GuiDebug* gui) {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Title
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "LIVE GRAPH");

  // BACK button
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(10, 10, 90, 40);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 20, 18, "BACK");

  // Graph bounds
  const int graphX1 = 40, graphY1 = 50;
  const int graphX2 = BRAIN_SCREEN_WIDTH - 40, graphY2 = BRAIN_SCREEN_HEIGHT - 60;
  const int graphW = graphX2 - graphX1;
  const int graphH = graphY2 - graphY1;

  // Draw axis
  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::draw_line(graphX1, graphY1, graphX1, graphY2); // Y axis
  pros::screen::draw_line(graphX1, graphY2, graphX2, graphY2); // X axis

  // Draw grid
  pros::screen::set_pen(COLOR_DARK_GRAY);
  for (int i = 0; i <= 5; i++) {
    int x = graphX1 + (i * graphW / 5);
    int y = graphY2 + 3;
    if (i > 0 && i < 5) {
      pros::screen::draw_line(x, graphY2, x, graphY2 + 5);
    }
  }

  // Draw data points
  if (gui->graphBuffer[0].x != 0 || gui->graphBuffer[0].y != 0) {
    pros::screen::set_pen(COLOR_GREEN);
    
    for (int i = 1; i < GuiDebug::GRAPH_BUFFER_SIZE; i++) {
      int prevIdx = (i - 1);
      double prevX = gui->graphBuffer[prevIdx].x;
      double prevY = gui->graphBuffer[prevIdx].y;
      
      double currX = gui->graphBuffer[i].x;
      double currY = gui->graphBuffer[i].y;
      
      if (gui->graphMaxX - gui->graphMinX < 0.001 || gui->graphMaxY - gui->graphMinY < 0.001) continue;
      
      int screenPrevX = graphX1 + (int)((prevX - gui->graphMinX) / (gui->graphMaxX - gui->graphMinX) * graphW);
      int screenPrevY = graphY2 - (int)((prevY - gui->graphMinY) / (gui->graphMaxY - gui->graphMinY) * graphH);
      
      int screenCurrX = graphX1 + (int)((currX - gui->graphMinX) / (gui->graphMaxX - gui->graphMinX) * graphW);
      int screenCurrY = graphY2 - (int)((currY - gui->graphMinY) / (gui->graphMaxY - gui->graphMinY) * graphH);
      
      screenPrevX = std::max(graphX1, std::min(graphX2, screenPrevX));
      screenPrevY = std::max(graphY1, std::min(graphY2, screenPrevY));
      screenCurrX = std::max(graphX1, std::min(graphX2, screenCurrX));
      screenCurrY = std::max(graphY1, std::min(graphY2, screenCurrY));
      
      pros::screen::draw_line(screenPrevX, screenPrevY, screenCurrX, screenCurrY);
    }
  }

  // Display axis labels
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_SMALL, graphX1 - 35, graphY1 - 5, "%.1f", gui->graphMaxY);
  pros::screen::print(pros::E_TEXT_SMALL, graphX1 - 35, graphY2 - 5, "%.1f", gui->graphMinY);
  pros::screen::print(pros::E_TEXT_SMALL, graphX1 - 5, graphY2 + 8, "%.1f", gui->graphMinX);
  pros::screen::print(pros::E_TEXT_SMALL, graphX2 - 20, graphY2 + 8, "%.1f", gui->graphMaxX);

  // Display current values
  pros::screen::print(pros::E_TEXT_SMALL, 40, BRAIN_SCREEN_HEIGHT - 40, "X: %.2f", 
    gui->graphBuffer[gui->graphBufferIndex > 0 ? gui->graphBufferIndex - 1 : GuiDebug::GRAPH_BUFFER_SIZE - 1].x);
  pros::screen::print(pros::E_TEXT_SMALL, BRAIN_SCREEN_WIDTH / 2 + 20, BRAIN_SCREEN_HEIGHT - 40, "Y: %.2f", 
    gui->graphBuffer[gui->graphBufferIndex > 0 ? gui->graphBufferIndex - 1 : GuiDebug::GRAPH_BUFFER_SIZE - 1].y);
}

void HandleLiveGraphTouch(GuiDebug* gui) {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // BACK button
    if (x >= 10 && x <= 90 && y >= 10 && y <= 40) {
      gui->DisplayDebugMenu();
      gui->CurrentScreen = DebugMenu;
      pros::delay(300);
      return;
    }
  }
}

}  // namespace aon
