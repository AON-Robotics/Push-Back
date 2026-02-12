#pragma once
#ifndef AON_TOOLS_GUI_UI_BUTTON_HPP_
#define AON_TOOLS_GUI_UI_BUTTON_HPP_

#include <cstdint>
#include <string>
#include "../../../../api.h"

namespace aon {
namespace ui {

/**
 * Simple button descriptor for GUI rendering and hit-testing.
 */
// Forward declare Button so the free helpers may be declared before the class
class Button;

// Free helper declarations (implemented in Button.cpp)
void drawButton(const Button& b, int textFmt = pros::E_TEXT_MEDIUM);
bool hitButton(const Button& b, int x, int y);

class Button {
 public:
  // Public layout and style fields so small helpers and free functions
  // can access them consistently across the codebase.
  int x1, y1, x2, y2;
  const char* label;
  std::uint32_t bg;
  std::uint32_t fg;

  // Convenience member wrappers that forward to the free helpers below.
  void draw(int textFmt = pros::E_TEXT_MEDIUM) { drawButton(*this, textFmt); }
  bool isHit(int x, int y) const { return hitButton(*this, x, y); }
};

}  // namespace ui
}  // namespace aon

#endif  // AON_TOOLS_GUI_UI_BUTTON_HPP_
