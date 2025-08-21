#ifndef AON_TOOLS_GUI_BASE_GUI_HPP_
#define AON_TOOLS_GUI_BASE_GUI_HPP_

#include <iostream>
#include <memory>
#include <string>
#include "../../../api.h"  // importing pros
#include "../function-reader.hpp"

#include "../gui-image-generator/gui-images.hpp"
#include "../../competition/autonomous-routines.hpp"
#include "../../competition/operator-control.hpp"

namespace aon {
// Function reader pointer
// Will store the addresses of the autonomous sequences
inline std::unique_ptr<FunctionReader<int>> AutonomousReader =
    std::make_unique<FunctionReader<int>>();

/**
 * NAMESPACE GUI
 *
 * This set of methods will be used to display a basic gui
 * We will use the python generator to generate the template for the "buttons"
 * To create the ilussion of buttons we will separate the screen in 12 spots
 * In those ranges the last touch will always be scanned to see if it's
 * attempting to access a screen that differs from the last one
 *
 * */
namespace gui {

// Prototypes
static void DrawCurrentScreen();
static void DrawButtonBlock(const std::uint32_t block_color, int block_start_x,
                            int block_start_y, int block_end_x, int block_end_y,
                            const std::uint32_t text_color,
                            pros::text_format_e_t text_format, int text_x,
                            int text_y, std::string text);
static void DrawButtonBlock(const std::uint32_t block_color, int block_start_x,
                            int block_start_y, int block_end_x, int block_end_y,
                            const std::uint32_t text_color,
                            pros::text_format_e_t text_format, int line,
                            std::string text);
static void DrawNABlock(int block_start_x, int block_start_y, int block_end_x,
                        int block_end_y, int text_x, int text_y);
static void DrawPressedBlock(int block_start_x, int block_start_y,
                             int block_end_x, int block_end_y);
static void HandleButtonPress();
static int TestSequence();

// Variables for screens
enum Screen {
  kMainMenu,
  kSelectRed,
  kSelectBlue,
  kSelectSkill,
  kDebugging,
  kWaiting
};
static Screen CurrentScreen = kMainMenu;
static Screen PreviousScreen = kMainMenu;
static bool drawn = false;
static bool is_waiting = false;
static pros::screen_touch_status_s_t TouchStatus;

/// Static x locations for segments to be displayed in screen
static int blocks_x[] = {0, BRAIN_SCREEN_WIDTH / 3,
                         BRAIN_SCREEN_WIDTH - BRAIN_SCREEN_WIDTH / 3,
                         BRAIN_SCREEN_WIDTH};

/// Static y locations for segments to be displayed in screen
static int blocks_y[] = {0, BRAIN_SCREEN_HEIGHT / 4, BRAIN_SCREEN_HEIGHT / 2,
                         BRAIN_SCREEN_HEIGHT - BRAIN_SCREEN_HEIGHT / 4,
                         BRAIN_SCREEN_HEIGHT};

const int menu_block_x = blocks_x[1] - 60;
const int menu_block_y = blocks_y[1];
const int menu_text_x = BRAIN_SCREEN_WIDTH / 5 - BRAIN_SCREEN_WIDTH / 10 - 40;
const int menu_text_y = menu_block_y - BRAIN_SCREEN_HEIGHT / 6;

/// Autonomous Constants
const int lower_block_text_y = BRAIN_SCREEN_HEIGHT - BRAIN_SCREEN_HEIGHT / 6;
const int lower_block_1_text_x =
    BRAIN_SCREEN_WIDTH / 6 - BRAIN_SCREEN_WIDTH / 12;
const int lower_block_2_text_x =
    BRAIN_SCREEN_WIDTH / 2 - BRAIN_SCREEN_WIDTH / 12;
const int lower_block_3_text_x = BRAIN_SCREEN_WIDTH - BRAIN_SCREEN_WIDTH / 4;

/// Initial GUI Method
static void Initialize() {
  // store functions in reader
  AutonomousReader->AddFunction("Test", &TestSequence);

  std::cout << "Initialitzing SAPI GUI" << std::endl;

  // Necessary for the Simplified Brain Screen API to work
  pros::delay(20);

  // Set background color to WHITE_SMOKE
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Draw team logo on the center of the screen
  pros::screen::set_pen(COLOR_WHITE_SMOKE);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "Initializing SAPI GUI");
  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2,
                   (BRAIN_SCREEN_HEIGHT - 225) / 2);
  pros::delay(1000);

  /// Main Loop
  // while (!selected) {
  while (true) {
    /// Draw current screen once
    if (!drawn) DrawCurrentScreen();

    /// Read touch screen
    TouchStatus = pros::screen::touch_status();

    /// Handle something that is touching the screen
    if (TouchStatus.touch_status > 0) HandleButtonPress();

    pros::delay(20);  // check TouchStatus every 20 ms
  }
}

/// Method to render each screen based on needs/specifications
static void DrawCurrentScreen() {
  // CLEAN SCREEN AND REDRAW BG & LOGO
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();
  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2,
                   (BRAIN_SCREEN_HEIGHT - 225) / 2);
  switch (CurrentScreen) {
      //    __  __   _   ___ _  _   __  __ ___ _  _ _   _
      //   |  \/  | /_\ |_ _| \| | |  \/  | __| \| | | | |
      //   | |\/| |/ _ \ | || .` | | |\/| | _|| .` | |_| |
      //   |_|  |_/_/ \_\___|_|\_| |_|  |_|___|_|\_|\___/
      //
    case kMainMenu:
      // DRAW MAIN MENU

      // TITLE
      pros::screen::set_pen(COLOR_GREEN);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "MAIN MENU");

      // LOWER LEFT BUTTON [RED SELECT]
      DrawButtonBlock(/*BLOCK*/ COLOR_RED, blocks_x[0], blocks_y[3],
                      blocks_x[1], blocks_y[4],
                      /*TEXT*/ COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                      lower_block_1_text_x, lower_block_text_y, "RED");

      // LOWER CENTER BUTTON [BLUE SELECT]
      DrawButtonBlock(/*BLOCK*/ COLOR_BLUE, blocks_x[1], blocks_y[3],
                      blocks_x[2], blocks_y[4],
                      /*TEXT*/ COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                      lower_block_2_text_x, lower_block_text_y, "BLUE");

      // LOWER RIGHT BUTTON [SKILL SELECT]
      DrawButtonBlock(/*BLOCK*/ COLOR_GREEN, blocks_x[2], blocks_y[3],
                      blocks_x[3], blocks_y[4],
                      /*TEXT*/ COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                      lower_block_3_text_x - 20, lower_block_text_y, "SKILLS");

      // UPPER RIGHT BUTTON [DEBUGGING]
      DrawButtonBlock(/*BLOCK*/ COLOR_LIGHT_STEEL_BLUE, blocks_x[2] + 20,
                      blocks_y[0], blocks_x[3], blocks_y[1],
                      /*TEXT*/ COLOR_BLACK, pros::E_TEXT_LARGE_CENTER,
                      blocks_x[2] + 40, blocks_y[1] - BRAIN_SCREEN_HEIGHT / 6,
                      "DEBUG");
      break;

      //    ___ ___ ___
      //   | _ \ __|   \
      //   |   / _|| |) |
      //   |_|_\___|___/
      //
    case kSelectRed:
      // DRAW SELECT RED

      // TITLE
      pros::screen::set_pen(COLOR_RED);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "SELECT RED");

      // MENU BUTTON
      DrawButtonBlock(/*BLOCK*/ COLOR_YELLOW, blocks_x[0], blocks_y[0],
                      menu_block_x, menu_block_y,
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, menu_text_x,
                      menu_text_y, "MENU");

      // LOWER LEFT BUTTON [AUT1]
      DrawButtonBlock(
          /*BLOCK*/ COLOR_LIGHT_PINK, blocks_x[0], blocks_y[3], blocks_x[1],
          blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_1_text_x,
          lower_block_text_y, "AUT1");

      // LOWER CENTER BUTTON [AUT2]
      DrawButtonBlock(/*BLOCK*/ COLOR_CRIMSON, blocks_x[1], blocks_y[3],
                      blocks_x[2], blocks_y[4],
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_2_text_x,
                      lower_block_text_y, "AUT2");

      // LOWER RIGHT BUTTON [AUT3]
      DrawButtonBlock(/*BLOCK*/ COLOR_RED, blocks_x[2], blocks_y[3],
                      blocks_x[3], blocks_y[4],
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_3_text_x,
                      lower_block_text_y, "AUT3");
      break;

      //    ___ _   _   _ ___
      //   | _ ) | | | | | __|
      //   | _ \ |_| |_| | _|
      //   |___/____\___/|___|
      //
    case kSelectBlue:
      // DRAW SELECT BLUE

      // TITLE
      pros::screen::set_pen(COLOR_BLUE);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "SELECT BLUE");

      // MENU BUTTON
      DrawButtonBlock(/*BLOCK*/ COLOR_YELLOW, blocks_x[0], blocks_y[0],
                      menu_block_x, menu_block_y,
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, menu_text_x,
                      menu_text_y, "MENU");

      // LOWER LEFT BUTTON [AUT1]
      DrawButtonBlock(
          /*BLOCK*/ COLOR_SKY_BLUE, blocks_x[0], blocks_y[3], blocks_x[1],
          blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_1_text_x,
          lower_block_text_y, "AUT1");

      // LOWER CENTER BUTTON [AUT2]
      DrawButtonBlock(
          /*BLOCK*/ COLOR_STEEL_BLUE, blocks_x[1], blocks_y[3], blocks_x[2],
          blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_2_text_x,
          lower_block_text_y, "AUT2");

      // LOWER RIGHT BUTTON [AUT3]
      DrawButtonBlock(/*BLOCK*/ COLOR_BLUE, blocks_x[2], blocks_y[3],
                      blocks_x[3], blocks_y[4],
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_3_text_x,
                      lower_block_text_y, "AUT3");
      break;

      //    ___ _  _____ _    _    ___
      //   / __| |/ /_ _| |  | |  / __|
      //   \__ \ ' < | || |__| |__\__ \
      //   |___/_|\_\___|____|____|___/
      //
    case kSelectSkill:
      // DRAW SELECT SKILL

      // TITLE
      pros::screen::set_pen(COLOR_GREEN);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "SELECT SKILL");

      // MENU BUTTON
      DrawButtonBlock(/*BLOCK*/ COLOR_YELLOW, blocks_x[0], blocks_y[0],
                      menu_block_x, menu_block_y,
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, menu_text_x,
                      menu_text_y, "MENU");
      
      // LOWER LEFT BUTTON [SKILL]
      DrawButtonBlock(
          /*BLOCK*/ COLOR_SKY_BLUE, blocks_x[0], blocks_y[3], blocks_x[1],
          blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_1_text_x,
          lower_block_text_y, "SKILL");
      break;

      //    ___  ___ ___ _   _  ___  ___ ___ _  _  ___
      //   |   \| __| _ ) | | |/ __|/ __|_ _| \| |/ __|
      //   | |) | _|| _ \ |_| | (_ | (_ || || .` | (_ |
      //   |___/|___|___/\___/ \___|\___|___|_|\_|\___|
      //
    case kDebugging:
      // DRAW DEBUGGING

      // TITLE
      pros::screen::set_pen(COLOR_GRAY);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "DEBUGGING");

      // MENU BUTTON
      DrawButtonBlock(/*BLOCK*/ COLOR_YELLOW, blocks_x[0], blocks_y[0],
                      menu_block_x, menu_block_y,
                      /*TEXT*/ COLOR_BLACK, TEXT_LARGE, menu_text_x,
                      menu_text_y, "MENU");
      // LOWER LEFT BUTTON [ADJUSTABLE]
      DrawButtonBlock(
          /*BLOCK*/ COLOR_SKY_BLUE, blocks_x[0], blocks_y[3], blocks_x[1],
          blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_1_text_x,
          lower_block_text_y, "ADJUSTABLE");

      // LOWER CENTER BUTTON [GUI...]
      DrawButtonBlock(
          /*BLOCK*/ COLOR_LIGHT_STEEL_BLUE, blocks_x[1], blocks_y[3],
          blocks_x[2], blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_LARGE, lower_block_2_text_x,
          lower_block_text_y, "GUI...");

      // LOWER RIGHT BUTTON [MULTIPLE]
      DrawButtonBlock(/*BLOCK*/ COLOR_BLUE, blocks_x[2], blocks_y[3],
          blocks_x[3], blocks_y[4],
          /*TEXT*/ COLOR_BLACK, TEXT_MEDIUM, lower_block_3_text_x,
          lower_block_text_y, "MULTIPLE");

      break;

      //   __      ___   ___ _____ ___ _  _  ___
      //   \ \    / /_\ |_ _|_   _|_ _| \| |/ __|
      //    \ \/\/ / _ \ | |  | |  | || .` | (_ |
      //     \_/\_/_/ \_\___| |_| |___|_|\_|\___|
      //
    case kWaiting:
    default:
      int buffering_count = 0;
      std::string buffer[] = {"", ".", "..", "..."};
      pros::screen::set_pen(COLOR_WHITE_SMOKE);
      pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "WAITING");
      // TITLE
      while (is_waiting) {
        DrawButtonBlock(/*BLOCK*/ COLOR_BLACK, blocks_x[0], blocks_y[3],
                        blocks_x[3], blocks_y[4],
                        /*TEXT*/ COLOR_WHITE_SMOKE, pros::E_TEXT_LARGE_CENTER,
                        10, buffer[(buffering_count++) % 4]);
        pros::delay(200);
      }
      drawn = false;
      CurrentScreen = PreviousScreen;
      return;
  }
  drawn = true;
}

/// Method to handle all button presses and user interactions/selections
static void HandleButtonPress() {
  PreviousScreen = CurrentScreen;
  // First check buttons in each screen
  switch (CurrentScreen) {
      //    __  __   _   ___ _  _   __  __ ___ _  _ _   _
      //   |  \/  | /_\ |_ _| \| | |  \/  | __| \| | | | |
      //   | |\/| |/ _ \ | || .` | | |\/| | _|| .` | |_| |
      //   |_|  |_/_/ \_\___|_|\_| |_|  |_|___|_|\_|\___/
      //
    case kMainMenu:
      if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          // RED
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          CurrentScreen = kSelectRed;
        } else if (TouchStatus.x < blocks_x[2]) {
          // BLUE
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          CurrentScreen = kSelectBlue;
        } else {
          // GREEN
          DrawPressedBlock(blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4]);
          CurrentScreen = kSelectSkill;
        }
        pros::delay(200);
      } else if (TouchStatus.y < blocks_y[1]) {
        if (TouchStatus.x > blocks_x[2]) {
          DrawPressedBlock(blocks_x[2], blocks_y[0], blocks_x[3], blocks_y[1]);
          CurrentScreen = kDebugging;
          pros::delay(200);
        }
      }
      break;

      //    ___ ___ ___
      //   | _ \ __|   \
      //   |   / _|| |) |
      //   |_|_\___|___/
      //
    case kSelectRed:
      // In selected autonomous
      // function reader will be loaded
      // with the required sequence
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] &&
          TouchStatus.x < menu_block_x) {
        // MENU
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(200);
      } else if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          // AUT1
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::RedRingsRoutine);
          pros::delay(200);

        } else if (TouchStatus.x < blocks_x[2]) {
          // AUT2
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::RedRingsRoutine);
          pros::delay(200);

        } else {
          // AUT3
          DrawPressedBlock(blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::RedRingsRoutine);
          pros::delay(200);
        }
      }
      break;

      //    ___ _   _   _ ___
      //   | _ ) | | | | | __|
      //   | _ \ |_| |_| | _|
      //   |___/____\___/|___|
      //
    case kSelectBlue:
      // In selected autonomous function reader will be loaded with the required
      // sequence
      pros::screen::set_eraser(COLOR_GRAY);
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] &&
          TouchStatus.x < menu_block_x) {
        // MENU
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(200);
      } else if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          // AUT1
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::BlueRingsRoutine);
          pros::delay(200);

        } else if (TouchStatus.x < blocks_x[2]) {
          // AUT2
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::BlueRingsRoutine);
          pros::delay(200);

        } else {
          // AUT3
          DrawPressedBlock(blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::BlueRingsRoutine);
          pros::delay(200);
        }
      }
      break;

      //    ___ _  _____ _    _    ___
      //   / __| |/ /_ _| |  | |  / __|
      //   \__ \ ' < | || |__| |__\__ \
      //   |___/_|\_\___|____|____|___/
      //
    case kSelectSkill:
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] &&
          TouchStatus.x < menu_block_x) {
        // MENU
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(200);
      } else if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          // AUT1
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          aon::operator_control::Run(aon::operator_control::DEFAULT);
          pros::delay(200);
        }
      }
      break;

      //    ___  ___ ___ _   _  ___  ___ ___ _  _  ___
      //   |   \| __| _ ) | | |/ __|/ __|_ _| \| |/ __|
      //   | |) | _|| _ \ |_| | (_ | (_ || || .` | (_ |
      //   |___/|___|___/\___/ \___|\___|___|_|\_|\___|
      //
    case kDebugging:
      if (TouchStatus.y < menu_block_y && TouchStatus.y > blocks_y[0] &&
          TouchStatus.x < menu_block_x) {
        // MENU
        DrawPressedBlock(blocks_x[0], blocks_y[0], menu_block_x, menu_block_y);
        CurrentScreen = kMainMenu;
        pros::delay(200);
      } else if (TouchStatus.y > blocks_y[3]) {
        if (TouchStatus.x < blocks_x[1]) {
          // ADJUSTABLE
          DrawPressedBlock(blocks_x[0], blocks_y[3], blocks_x[1], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::testAdjustable);
          pros::delay(200);

        } else if (TouchStatus.x < blocks_x[2]) {
          // GUI...
          DrawPressedBlock(blocks_x[1], blocks_y[3], blocks_x[2], blocks_y[4]);
          // Change screen to waiting
          CurrentScreen = kWaiting;
          is_waiting = true;
          pros::Task test_task([=] {
            for (int i = 0; i < 10; i++) {
              std::cout << "I'm HERE " << i << std::endl;
              DrawButtonBlock(COLOR_BLACK, blocks_x[0], blocks_y[1],
                              blocks_x[1] - 20, blocks_y[2], COLOR_WHITE_SMOKE,
                              TEXT_LARGE, blocks_x[1] / 2,
                              blocks_y[1] + blocks_y[2] / 6, std::to_string(i));

              pros::delay(1000);
            }
            is_waiting = false;
          });
          pros::delay(200);

        } else {
          // MULTIPLE
          DrawPressedBlock(blocks_x[2], blocks_y[3], blocks_x[3], blocks_y[4]);
          AutonomousReader->AddFunction("autonomous", aon::testMultiple);
          pros::delay(200);
        }
      }
      break;

      //   __      ___   ___ _____ ___ _  _  ___
      //   \ \    / /_\ |_ _|_   _|_ _| \| |/ __|
      //    \ \/\/ / _ \ | |  | |  | || .` | (_ |
      //     \_/\_/_/ \_\___| |_| |___|_|\_|\___|
      //
    case kWaiting:
    default:
      break;
  }

  // If screen changes redraw...
  if (PreviousScreen != CurrentScreen) drawn = false;
}

/// Draw a block with text (x,y)
/// \param block_color Color that will fill the rectangle of the button [can be
/// found in pros/colors.h] \param block_start_x X coordinate of the left top
/// corner of the rectangle \param block_start_y Y coordinate of the left top
/// corner of the rectangle \param block_end_x X coordinate of the right bottom
/// corner of the rectangle \param block_end_y Y coordinate of the right bottom
/// corner of the rectangle \param text_color Color that will be used to write
/// the text characters [can be found in pros/colors.h] \param text_format Size
/// and orientation of the characters of text [can be found in pros/screen.h]
/// \param text_x X coordinate of the first character in the text of the button
/// \param text_y Y coordinate of the first character in the text of the button
/// \param text Characters to be displayed in the button
static void DrawButtonBlock(const std::uint32_t block_color, int block_start_x,
                            int block_start_y, int block_end_x, int block_end_y,
                            const std::uint32_t text_color,
                            pros::text_format_e_t text_format, int text_x,
                            int text_y, std::string text) {
  pros::screen::set_eraser(block_color);
  pros::screen::erase_rect(block_start_x, block_start_y, block_end_x,
                           block_end_y);
  pros::screen::set_pen(text_color);
  pros::screen::print(text_format, text_x, text_y, text.c_str());
}

/// Draw a block with text (line)
/// \param block_color Color that will fill the rectangle of the button [can be
/// found in pros/colors.h] \param block_start_x X coordinate of the left top
/// corner of the rectangle \param block_start_y Y coordinate of the left top
/// corner of the rectangle \param block_end_x X coordinate of the right bottom
/// corner of the rectangle \param block_end_y Y coordinate of the right bottom
/// corner of the rectangle \param text_color Color that will be used to write
/// the text characters [can be found in pros/colors.h] \param text_format Size
/// and orientation of the characters of text [can be found in pros/screen.h]
/// \param line Line number to insert the text of the button
/// \param text Characters to be displayed in the button
static void DrawButtonBlock(const std::uint32_t block_color, int block_start_x,
                            int block_start_y, int block_end_x, int block_end_y,
                            const std::uint32_t text_color,
                            pros::text_format_e_t text_format, int line,
                            std::string text) {
  pros::screen::set_eraser(block_color);
  pros::screen::erase_rect(block_start_x, block_start_y, block_end_x,
                           block_end_y);
  pros::screen::set_pen(text_color);
  pros::screen::print(text_format, line, text.c_str());
}

/// Draw a block that represents an undefined autonomous sequence
/// \param block_start_x X coordinate of the left top corner of the rectangle
/// \param block_start_y Y coordinate of the left top corner of the rectangle
/// \param block_end_x X coordinate of the right bottom corner of the rectangle
/// \param block_end_y Y coordinate of the right bottom corner of the rectangle
/// \param text_x X coordinate of the first character in the text of the button
/// \param text_y Y coordinate of the first character in the text of the button
static void DrawNABlock(int block_start_x, int block_start_y, int block_end_x,
                        int block_end_y, int text_x, int text_y) {
  DrawButtonBlock(COLOR_GRAY, block_start_x, block_start_y, block_end_x,
                  block_end_y, COLOR_BLACK, TEXT_LARGE, text_x, text_y, "N/A");
}

/// Draw a block that visualizes a press action
/// \param block_start_x X coordinate of the left top corner of the rectangle
/// \param block_start_y Y coordinate of the left top corner of the rectangle
/// \param block_end_x X coordinate of the right bottom corner of the rectangle
/// \param block_end_y Y coordinate of the right bottom corner of the rectangle
static void DrawPressedBlock(int block_start_x, int block_start_y,
                             int block_end_x, int block_end_y) {
  pros::screen::set_eraser(COLOR_GRAY);
  pros::screen::erase_rect(block_start_x, block_start_y, block_end_x,
                           block_end_y);
}

/// SEQUENCES FOR BASE GUI
/// Test sequence to be stored in reader
/// Will be run in the debug screen as part of a series of tests
static int TestSequence() {
  std::cout << "Executing function inside the reader!" << std::endl;
  int count = 0;
  for (int i = 0; i < 10; ++i) {
    std::cout << count++ << "\t";
  }
  std::cout << "\n";
  return 100000;
}
};  // namespace gui
};  // namespace aon

#endif  // AON_TOOLS_GUI_BASE_GUI_HPP_
