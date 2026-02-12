#include "../../../include/aon/tools/gui/gui-v2.hpp"
#include "../../../include/aon/tools/gui/gui-v2-debug.hpp"
#include "../../../include/aon/constants.hpp"
#include "../../../include/aon/tools/gui/ui/button.hpp"
#include "../../../include/aon/tools/gui/ui/gui-v2-layout.hpp"

namespace aon {

void Gui::DisplayMainMenu() {
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Draw the AON logo higher at the top center
  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2, (BRAIN_SCREEN_HEIGHT - 225) / 4);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE); // Default color for "NO AUTON"
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::set_pen(COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  // Draw the "AUTONS" button using UI helper
  ui::drawButton(AutonsBtn, pros::E_TEXT_LARGE);
}

void Gui::DisplayAutonMenu() {
  // Main AUTONS hub: shows current selection and navigates to Red/Blue/Skills submenus
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  aon::drawAutonSelections();

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::set_pen(COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, selectedAutonName.c_str());
  }

  // Draw navigation buttons using UI helpers
  ui::drawButton(BackBtnGray);
  ui::drawButton(BlueBtn, pros::E_TEXT_LARGE);
  ui::drawButton(RedBtn, pros::E_TEXT_LARGE);
  ui::drawButton(SkillsBtn, pros::E_TEXT_LARGE);
}

void Gui::DisplayRedAutonMenu() {
  // Red-side autons list with three option buttons
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to red
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase();

  // Draw BACK button
  ui::drawButton(BackBtnRed);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "RED" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "RED");

  // Draw auton selection buttons with red theme colors
  ui::Button aut1 = Aut1Btn; aut1.bg = COLOR_LIGHT_PINK;
  ui::Button aut2 = Aut2Btn; aut2.bg = COLOR_CRIMSON;
  ui::Button aut3 = Aut3Btn; aut3.bg = COLOR_RED;
  ui::drawButton(aut1, pros::E_TEXT_LARGE);
  ui::drawButton(aut2, pros::E_TEXT_LARGE);
  ui::drawButton(aut3, pros::E_TEXT_LARGE);
}

void Gui::DisplayBlueAutonMenu() {
  // Blue-side autons list
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to blue
  pros::screen::set_eraser(COLOR_BLUE);
  pros::screen::erase();

  // Draw BACK button
  ui::drawButton(BackBtnBlue);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "BLUE" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "BLUE");

  // Draw auton selection buttons with blue theme colors
  ui::Button aut1 = Aut1Btn; aut1.bg = COLOR_SKY_BLUE;
  ui::Button aut2 = Aut2Btn; aut2.bg = COLOR_STEEL_BLUE;
  ui::Button aut3 = Aut3Btn; aut3.bg = COLOR_BLUE;
  ui::drawButton(aut1, pros::E_TEXT_LARGE);
  ui::drawButton(aut2, pros::E_TEXT_LARGE);
  ui::drawButton(aut3, pros::E_TEXT_LARGE);
}

void Gui::DisplaySkillsMenu() {
  // Skills autons list
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to green
  pros::screen::set_eraser(COLOR_GREEN);
  pros::screen::erase();

  // Add a delay to allow the screen to load
  pros::delay(300);

  // Draw BACK button
  ui::drawButton(BackBtnGreen);

  // Display the current selected autonomous routine at the top center
  pros::screen::set_pen(COLOR_WHITE);
  if (selectedAutonName == "None") {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "NO AUTON");
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, selectedAutonName.c_str());
  }

  // Display "SKILLS" in the center
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 4, "SKILLS");

  // Draw auton selection buttons with green theme colors
  ui::Button aut1 = Aut1Btn; aut1.bg = COLOR_LIGHT_GREEN;
  ui::Button aut2 = Aut2Btn; aut2.bg = COLOR_YELLOW_GREEN;
  ui::Button aut3 = Aut3Btn; aut3.bg = COLOR_GREEN;
  ui::drawButton(aut1, pros::E_TEXT_LARGE);
  ui::drawButton(aut2, pros::E_TEXT_LARGE);
  ui::drawButton(aut3, pros::E_TEXT_LARGE);
}

}  // namespace aon
