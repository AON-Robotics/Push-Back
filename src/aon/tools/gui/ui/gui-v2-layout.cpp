#include "../../../include/aon/tools/gui/ui/gui-v2-layout.hpp"
#include "../../../include/aon/constants.hpp"

namespace aon {

// Button Layout Constants
const ui::Button BackBtnGray   = {10, 10, 70, 40, "BACK", COLOR_DARK_GRAY, COLOR_WHITE};
const ui::Button BackBtnRed    = {10, 10, 70, 40, "BACK", COLOR_DARK_RED, COLOR_WHITE};
const ui::Button BackBtnBlue   = {10, 10, 70, 40, "BACK", COLOR_DARK_BLUE, COLOR_WHITE};
const ui::Button BackBtnGreen  = {10, 10, 70, 40, "BACK", COLOR_DARK_GREEN, COLOR_WHITE};

const ui::Button AutonsBtn     = {0, BRAIN_SCREEN_HEIGHT - 50, BRAIN_SCREEN_WIDTH, BRAIN_SCREEN_HEIGHT, "AUTONS", COLOR_GREEN, COLOR_WHITE};
const ui::Button RedBtn        = {45, BRAIN_SCREEN_HEIGHT / 2 - 30, 145, BRAIN_SCREEN_HEIGHT / 2 + 30, "RED", COLOR_RED, COLOR_WHITE};
const ui::Button BlueBtn       = {BRAIN_SCREEN_WIDTH - 130, BRAIN_SCREEN_HEIGHT / 2 - 30, BRAIN_SCREEN_WIDTH - 30, BRAIN_SCREEN_HEIGHT / 2 + 30, "BLUE", COLOR_BLUE, COLOR_WHITE};
const ui::Button SkillsBtn     = {BRAIN_SCREEN_WIDTH / 2 - 60, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 80, BRAIN_SCREEN_HEIGHT - 50, "SKILLS", COLOR_GREEN, COLOR_WHITE};

// Auton selection buttons (bottom row)
const ui::Button Aut1Btn       = {50, BRAIN_SCREEN_HEIGHT - 100, 150, BRAIN_SCREEN_HEIGHT - 50, "AUT1", 0, COLOR_BLACK};
ui::Button Aut2Btn             = {BRAIN_SCREEN_WIDTH / 2 - 50, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH / 2 + 50, BRAIN_SCREEN_HEIGHT - 50, "AUT2", 0, COLOR_BLACK};
const ui::Button Aut3Btn       = {BRAIN_SCREEN_WIDTH - 150, BRAIN_SCREEN_HEIGHT - 100, BRAIN_SCREEN_WIDTH - 50, BRAIN_SCREEN_HEIGHT - 50, "AUT3", 0, COLOR_BLACK};

}  // namespace aon
