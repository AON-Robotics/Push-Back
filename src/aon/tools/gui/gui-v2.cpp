#include "../../../include/aon/tools/gui/gui-v2.hpp"
#include "../../../include/aon/tools/gui/gui-v2-debug.hpp"
#include "../../../include/aon/constants.hpp"
#include "../../../include/aon/tools/gui/ui/button.hpp"
#include "../../../include/aon/tools/gui/ui/gui-v2-layout.hpp"

namespace aon {

// Button layout constants are provided by Gui-V2-Layout.hpp / Gui-V2-Layout.cpp

// Select concrete GUI implementation based on TESTING_AUTONOMOUS flag.
// TESTING_AUTONOMOUS = true  -> GuiDebug (full debug features)
// TESTING_AUTONOMOUS = false -> Gui (competition mode, no debug menu)
#if TESTING_AUTONOMOUS
static GuiDebug gui_impl;
#else
static Gui gui_impl;
#endif
Gui& gui = gui_impl;

// Define the AutonomousReader unique_ptr
std::unique_ptr<FunctionReader<int>> AutonomousReader =
    std::make_unique<FunctionReader<int>>();

// ============================================================================
// Helper Methods
// ============================================================================

  int Gui::DisplayInitializationMessage() {
    // Clear the screen completely
    pros::screen::set_eraser(COLOR_BLACK);
    pros::screen::erase();

    // Typewriter-style primary message (shows debug text when built for testing)
  #if TESTING_AUTONOMOUS
    const char* msg = "Initializing Debug...";
  #else
    const char* msg = "Initializing AON...";
  #endif
    int len = 0; for (const char* p = msg; *p; ++p) ++len;

    // Estimate character dimensions (matches Button helpers)
    int charWidth = 18; // pixels for large text
    int charHeight = 20;

    int textWidth = len * charWidth;
    int startX = (BRAIN_SCREEN_WIDTH - textWidth) / 2;
    int startY = BRAIN_SCREEN_HEIGHT / 3; // higher on screen

    pros::screen::set_pen(COLOR_WHITE);
    for (int i = 1; i <= len; ++i) {
      // Clear the text area to avoid artifacts
      pros::screen::set_eraser(COLOR_BLACK);
      pros::screen::erase_rect(startX, startY, startX + textWidth, startY + charHeight + 4);

      // Print substring (typewriter effect)
      char buf[128];
      int copy = (i < (int)sizeof(buf)) ? i : ((int)sizeof(buf) - 1);
      for (int j = 0; j < copy; ++j) buf[j] = msg[j];
      buf[copy] = '\0';
      pros::screen::print(pros::E_TEXT_LARGE, startX, startY, "%s", buf);

      pros::delay(60);
    }

    // Secondary message displayed lower and centered with typewriter effect
  #if TESTING_AUTONOMOUS
    const char* secMsg = "Debug is ENABLED";
  #else
    const char* secMsg = "AON is ON";
  #endif
    int secLen = 0; for (const char* p = secMsg; *p; ++p) ++secLen;
    int secTextWidth = secLen * charWidth;
    int secX = (BRAIN_SCREEN_WIDTH - secTextWidth) / 2;
    int secY = startY + 40;
    pros::screen::set_pen(COLOR_WHITE);
    for (int i = 1; i <= secLen; ++i) {
      // Clear the secondary text area
      pros::screen::set_eraser(COLOR_BLACK);
      pros::screen::erase_rect(secX, secY, secX + secTextWidth, secY + charHeight + 4);

      // Print substring
      char buf[64];
      int copy = (i < (int)sizeof(buf)) ? i : ((int)sizeof(buf) - 1);
      for (int j = 0; j < copy; ++j) buf[j] = secMsg[j];
      buf[copy] = '\0';
      pros::screen::print(pros::E_TEXT_LARGE, secX, secY, "%s", buf);

      pros::delay(60);
    }
    if (TESTING_AUTONOMOUS) {
      aon::drawDebugCleaner();
    } else {
      return 0;
    }

    return 0;
  }

void Gui::ApplyPreselectedAuton() {
  // If user already directly set selectedAuton, don't override.
  if (selectedAuton.routine != nullptr && selectedAutonName != "None" &&
      (selectedRedAut == 0 && selectedBlueAut == 0 && selectedSkill == 0)) {
    return; // Nothing to map; routine already chosen
  }

  if (selectedRedAut > 0) {
    SelectAutonByList(Alliance::Red, selectedRedAut);
    return; // Red takes precedence
  }

  if (selectedBlueAut > 0) {
    SelectAutonByList(Alliance::Blue, selectedBlueAut);
    return; // Blue next
  }

  if (selectedSkill > 0) {
    SelectAutonByList(Alliance::Skills, selectedSkill);
  }
}

void Gui::SelectAutonByList(Alliance alliance, int index1Based) {
  if (index1Based < 1) index1Based = 1;
  if (index1Based > 3) index1Based = 3;

  const AutonOption* options = nullptr;
  switch (alliance) {
    case Alliance::Red:
      options = RedAutonOptions;
      selectedRedAut = index1Based;
      selectedBlueAut = 0;
      selectedSkill = 0;
      break;
    case Alliance::Blue:
      options = BlueAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = index1Based;
      selectedSkill = 0;
      break;
    case Alliance::Skills:
      options = SkillsAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = 0;
      selectedSkill = index1Based;
      break;
  }
  const AutonOption& choice = options[index1Based - 1];
  selectedAuton = choice;
  selectedAutonName = choice.name;
  
  // Register the selected autonomous routine to the AutonomousReader
  AutonomousReader->AddFunction("autonomous", choice.routine);

  if (CurrentScreen == MainMenu) {
    DisplayMainMenu();
  }
}



// ============================================================================
// Touch Handlers
// ============================================================================

void Gui::HandleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) {
  // Check if AUTONS button is pressed
  if (AutonsBtn.isHit(touchStatus.x, touchStatus.y)) {
    if (CurrentScreen != AutonMenu) {
      DisplayAutonMenu();
      CurrentScreen = AutonMenu;
    }
  }
}

void Gui::HandleAutonMenuTouch() {
  if (CurrentScreen != AutonMenu) return;

  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnGray.isHit(x, y)) {
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (BlueBtn.isHit(x, y)) {
    DisplayBlueAutonMenu();
    CurrentScreen = BlueAutons;
  } else if (RedBtn.isHit(x, y)) {
    DisplayRedAutonMenu();
    CurrentScreen = RedAutons;
  } else if (SkillsBtn.isHit(x, y)) {
    DisplaySkillsMenu();
    CurrentScreen = SkillAutons;
  }
}

void Gui::HandleRedAutonMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnRed.isHit(x, y)) {
    DisplayAutonMenu();
    CurrentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (Aut1Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Red, 1);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Red, 2);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Red, 3);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  }
}

void Gui::HandleBlueAutonMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnBlue.isHit(x, y)) {
    DisplayAutonMenu();
    CurrentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (Aut1Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Blue, 1);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Blue, 2);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Blue, 3);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  }
}

void Gui::HandleSkillsMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnGreen.isHit(x, y)) {
    DisplayAutonMenu();
    CurrentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (Aut1Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Skills, 1);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Skills, 2);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    SelectAutonByList(Alliance::Skills, 3);
    DisplayMainMenu();
    CurrentScreen = MainMenu;
  }
}

// ============================================================================
// GUI Loop
// ============================================================================

void Gui::RunGuiLoop() {
  while (true) {
    pros::screen_touch_status_s_t TouchStatus = pros::screen::touch_status();
    if (TouchStatus.touch_status > 0) {
      switch (CurrentScreen) {
        case MainMenu:
          HandleMainMenuTouch(TouchStatus);
          break;
        case AutonMenu:
          HandleAutonMenuTouch();
          break;
        case RedAutons:
          HandleRedAutonMenuTouch();
          break;
        case BlueAutons:
          HandleBlueAutonMenuTouch();
          break;
        case SkillAutons:
          HandleSkillsMenuTouch();
          break;
        default:
          break;
      }
    }

    pros::delay(100);
  }
}

// ============================================================================
// Initialization
// ============================================================================

void Gui::Initialize() {
  std::cout << "Start" << std::endl;

  pros::delay(5);

  CurrentScreen = MainMenu;
  
  DisplayInitializationMessage();
  
  // Keep initialization message visible briefly before showing main menu
  pros::delay(1000);

  DisplayMainMenu();

  ApplyPreselectedAuton();

  // Ensure gui has the default selected-invoker behavior available.
  // The base `Gui` implementation uses `selectedAutonRoutine`.

  // Launch GUI loop task
  if (!guiLoopTask) {
    guiLoopTask = std::make_unique<pros::Task>([this] {
      this->RunGuiLoop();
    });
  }
}

int Gui::InvokeSelectedAuton() {
  if (selectedAuton.routine != nullptr) return selectedAuton.routine();
  return 0;
}


void InitializeGui() {
    gui.Initialize();
  }
}  // namespace aon
