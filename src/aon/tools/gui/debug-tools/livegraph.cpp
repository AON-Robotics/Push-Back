#include "../../../../../include/aon/tools/gui/gui-debug.hpp"
#include <algorithm>

namespace aon {

void GuiDebug::DisplayLiveGraph() {
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
  if (graphBuffer[0].x != 0 || graphBuffer[0].y != 0) {
    pros::screen::set_pen(COLOR_GREEN);
    
    for (int i = 1; i < GuiDebug::GRAPH_BUFFER_SIZE; i++) {
      int prevIdx = (i - 1);
      double prevX = graphBuffer[prevIdx].x;
      double prevY = graphBuffer[prevIdx].y;
      
      double currX = graphBuffer[i].x;
      double currY = graphBuffer[i].y;
      
      if (graphMaxX - graphMinX < 0.001 || graphMaxY - graphMinY < 0.001) continue;
      
      int screenPrevX = graphX1 + (int)((prevX - graphMinX) / (graphMaxX - graphMinX) * graphW);
      int screenPrevY = graphY2 - (int)((prevY - graphMinY) / (graphMaxY - graphMinY) * graphH);
      
      int screenCurrX = graphX1 + (int)((currX - graphMinX) / (graphMaxX - graphMinX) * graphW);
      int screenCurrY = graphY2 - (int)((currY - graphMinY) / (graphMaxY - graphMinY) * graphH);
      
      screenPrevX = std::max(graphX1, std::min(graphX2, screenPrevX));
      screenPrevY = std::max(graphY1, std::min(graphY2, screenPrevY));
      screenCurrX = std::max(graphX1, std::min(graphX2, screenCurrX));
      screenCurrY = std::max(graphY1, std::min(graphY2, screenCurrY));
      
      pros::screen::draw_line(screenPrevX, screenPrevY, screenCurrX, screenCurrY);
    }
  }

  // Display axis labels
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_SMALL, graphX1 - 35, graphY1 - 5, "%.1f", graphMaxY);
  pros::screen::print(pros::E_TEXT_SMALL, graphX1 - 35, graphY2 - 5, "%.1f", graphMinY);
  pros::screen::print(pros::E_TEXT_SMALL, graphX1 - 5, graphY2 + 8, "%.1f", graphMinX);
  pros::screen::print(pros::E_TEXT_SMALL, graphX2 - 20, graphY2 + 8, "%.1f", graphMaxX);

  // Display current values
  pros::screen::print(pros::E_TEXT_SMALL, 40, BRAIN_SCREEN_HEIGHT - 40, "X: %.2f", 
    graphBuffer[graphBufferIndex > 0 ? graphBufferIndex - 1 : GuiDebug::GRAPH_BUFFER_SIZE - 1].x);
  pros::screen::print(pros::E_TEXT_SMALL, BRAIN_SCREEN_WIDTH / 2 + 20, BRAIN_SCREEN_HEIGHT - 40, "Y: %.2f", 
    graphBuffer[graphBufferIndex > 0 ? graphBufferIndex - 1 : GuiDebug::GRAPH_BUFFER_SIZE - 1].y);
}

void GuiDebug::HandleLiveGraphTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status > 0) {
    int x = touch.x;
    int y = touch.y;

    // BACK button
    if (x >= 10 && x <= 90 && y >= 10 && y <= 40) {
      DisplayDebugMenu();
      currentScreen = DebugMenu;
      pros::delay(300);
      return;
    }
  }
}

}  // namespace aon
