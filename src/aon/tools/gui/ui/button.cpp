#include "../../../../../include/aon/tools/gui/ui/button.hpp"

namespace aon {
namespace ui {

void drawButton(const Button& b, int textFmt) {
  // Draw background
  pros::screen::set_eraser(b.bg);
  pros::screen::erase_rect(b.x1, b.y1, b.x2, b.y2);

  // Calculate text dimensions based on format
  // Reduced char widths to shift text right
  int charWidth = 12;  // default for MEDIUM
  int charHeight = 16;
  if (textFmt == pros::E_TEXT_SMALL) {
    charWidth = 8;
    charHeight = 12;
  } else if (textFmt == pros::E_TEXT_LARGE) {
    charWidth = 18;
    charHeight = 20;
  }

  // Calculate label length and center text in button
  int labelLen = 0;
  for (const char* p = b.label; *p; ++p) ++labelLen;

  int textWidth = labelLen * charWidth;
  int textX = b.x1 + (b.x2 - b.x1 - textWidth) / 2;
  int textY = b.y1 + (b.y2 - b.y1 - charHeight) / 2;

  // Draw label
  pros::screen::set_pen(b.fg);
  pros::screen::print(static_cast<pros::text_format_e_t>(textFmt), textX, textY, b.label);
}

bool hitButton(const Button& b, int x, int y) {
  return (x >= b.x1 && x <= b.x2 && y >= b.y1 && y <= b.y2);
}

}  // namespace ui
}  // namespace aon
