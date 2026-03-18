#include "../../../include/aon/tools/gui/gui.hpp"
#include "../../../include/aon/tools/gui/gui-debug.hpp"
#include "../../../include/aon/constants.hpp"
#include "../../../include/aon/tools/gui/ui/gui-layout.hpp"

namespace aon {

void Gui::displayMainMenu() {
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

  // Draw the "AUTONS" button using UI helper
  AutonsBtn.draw(pros::E_TEXT_LARGE);
}

void Gui::displayAutonMenu() {
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
  backBtnGray.draw();
  blueBtn.draw(pros::E_TEXT_LARGE);
  redBtn.draw(pros::E_TEXT_LARGE);
  skillsBtn.draw(pros::E_TEXT_LARGE);
}

void Gui::displayRedAutonMenu() {
  // Red-side autons list with three option buttons
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to red
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase();

  // Draw BACK button
  backBtnRed.draw();

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
  ui::Button aut1 = aut1Btn; aut1.bg = COLOR_LIGHT_PINK;
  ui::Button aut2 = Aut2Btn; aut2.bg = COLOR_CRIMSON;
  ui::Button aut3 = Aut3Btn; aut3.bg = COLOR_RED;
  aut1.draw(pros::E_TEXT_LARGE);
  aut2.draw(pros::E_TEXT_LARGE);
  aut3.draw(pros::E_TEXT_LARGE);
}

void Gui::displayBlueAutonMenu() {
  // Blue-side autons list
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to blue
  pros::screen::set_eraser(COLOR_BLUE);
  pros::screen::erase();

  // Draw BACK button
  BackBtnBlue.draw();

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
  ui::Button aut1 = aut1Btn; aut1.bg = COLOR_SKY_BLUE;
  ui::Button aut2 = Aut2Btn; aut2.bg = COLOR_STEEL_BLUE;
  ui::Button aut3 = Aut3Btn; aut3.bg = COLOR_BLUE;
  aut1.draw(pros::E_TEXT_LARGE);
  aut2.draw(pros::E_TEXT_LARGE);
  aut3.draw(pros::E_TEXT_LARGE);
}

void Gui::displaySkillsMenu() {
  // Skills autons list
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Set background to green
  pros::screen::set_eraser(COLOR_GREEN);
  pros::screen::erase();

  // Add a delay to allow the screen to load
  pros::delay(300);

  // Draw BACK button
  BackBtnGreen.draw();

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
  ui::Button aut1 = aut1Btn; aut1.bg = COLOR_LIGHT_GREEN;
  ui::Button aut2 = Aut2Btn; aut2.bg = COLOR_YELLOW_GREEN;
  ui::Button aut3 = Aut3Btn; aut3.bg = COLOR_GREEN;
  aut1.draw(pros::E_TEXT_LARGE);
  aut2.draw(pros::E_TEXT_LARGE);
  aut3.draw(pros::E_TEXT_LARGE);
}

}  // namespace aon
