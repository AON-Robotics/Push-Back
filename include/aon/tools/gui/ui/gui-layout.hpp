#pragma once
#ifndef AON_TOOLS_GUI_LAYOUT_HPP_
#define AON_TOOLS_GUI_LAYOUT_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include "../../../../api.h"
#include "aon/constants.hpp"

namespace aon {
namespace ui {

class Button {
 public:
  int x1, y1, x2, y2;
  std::string label;
  std::uint32_t bg;
  std::uint32_t fg;
  std::function<void()> onPress = nullptr;

  void draw(int textFmt = pros::E_TEXT_MEDIUM) const {
    pros::screen::set_eraser(bg);
    pros::screen::erase_rect(x1, y1, x2, y2);

    int charWidth = 12;
    int charHeight = 16;
    if (textFmt == pros::E_TEXT_SMALL) {
      charWidth = 8;
      charHeight = 12;
    } else if (textFmt == pros::E_TEXT_LARGE) {
      charWidth = 18;
      charHeight = 20;
    }

    int textWidth = static_cast<int>(label.size()) * charWidth;
    int textX = x1 + (x2 - x1 - textWidth) / 2;
    int textY = y1 + (y2 - y1 - charHeight) / 2;

    pros::screen::set_pen(fg);
    pros::screen::print(static_cast<pros::text_format_e_t>(textFmt), textX, textY, label.c_str());
  }

  bool isHit(int x, int y) const {
    return (x >= x1 && x <= x2 && y >= y1 && y <= y2);
  }
};

}  // namespace ui

// Back buttons
inline const ui::Button backBtnGray  = {10, 10, 70, 40, "BACK", COLOR_DARK_GRAY,  COLOR_WHITE};
inline const ui::Button backBtnRed   = {10, 10, 70, 40, "BACK", COLOR_DARK_RED,   COLOR_WHITE};
inline const ui::Button BackBtnBlue  = {10, 10, 70, 40, "BACK", COLOR_DARK_BLUE,  COLOR_WHITE};
inline const ui::Button BackBtnGreen = {10, 10, 70, 40, "BACK", COLOR_DARK_GREEN, COLOR_WHITE};

// Main menu buttons
inline const ui::Button AutonsBtn = {0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT, "AUTONS", COLOR_GREEN, COLOR_WHITE};
inline const ui::Button redBtn    = {45, BRAIN_SCREEN_HEIGHT / 2 - 30, 145, BRAIN_SCREEN_HEIGHT / 2 + 30, "RED", COLOR_RED, COLOR_WHITE};
inline const ui::Button blueBtn   = {BRAIN_SCREEN_WIDTH - 130, BRAIN_SCREEN_HEIGHT / 2 - 30, BRAIN_SCREEN_WIDTH - 30, BRAIN_SCREEN_HEIGHT / 2 + 30, "BLUE", COLOR_BLUE, COLOR_WHITE};
inline const ui::Button skillsBtn = {BRAIN_SCREEN_WIDTH / 2 - 60, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 80, BRAIN_SCREEN_HEIGHT - 50, "SKILLS", COLOR_GREEN, COLOR_WHITE};

// Auton selection buttons
inline const ui::Button aut1Btn = {50, BRAIN_SCREEN_HEIGHT - 100, 150, BRAIN_SCREEN_HEIGHT - 50, "AUT1", 0, COLOR_BLACK};
inline       ui::Button Aut2Btn = {BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 50, BRAIN_SCREEN_HEIGHT - 50, "AUT2", 0, COLOR_BLACK};
inline const ui::Button Aut3Btn = {BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH - 50, BRAIN_SCREEN_HEIGHT - 50, "AUT3", 0, COLOR_BLACK};

}  // namespace aon

#endif  // AON_TOOLS_GUI_LAYOUT_HPP_
