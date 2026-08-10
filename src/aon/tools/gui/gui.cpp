#include "aon/tools/gui/gui.hpp"
#include "aon/tools/gui/gui-debug.hpp"
#include "aon/tools/gui-image-generator/gui-images.hpp"
#include "aon/auton/fallback-status.hpp"
#include "aon/config/robot-config.hpp"
#include "aon/constants.hpp"
#include "aon/tools/gui/ui/gui-layout.hpp"

#include <cstdio>

namespace aon {
namespace {

struct SavedAutonSelection {
  Alliance alliance = Alliance::Red;
  int index = 0;
  bool hasSelection = false;
};

SavedAutonSelection savedAutonSelection;

constexpr const char* kAutonSelectionPath = "/usd/aon-auton-selection.txt";

bool sameSlotSummary(const shadow::SlotSummary& left,
                     const shadow::SlotSummary& right) {
  return left.result == right.result && left.valid == right.valid &&
         left.generation == right.generation &&
         left.durationMs == right.durationMs && left.startX == right.startX &&
         left.startY == right.startY &&
         left.startHeading == right.startHeading;
}

bool timeReached(std::uint32_t now, std::uint32_t deadline) {
  return shadow::confirmationExpired(now, deadline);
}

shadow::RobotIdentity activeShadowRobot() {
  return aon::config::activeRobotConfig().identity ==
                 aon::config::RobotIdentity::Small
             ? shadow::RobotIdentity::Small
             : shadow::RobotIdentity::Big;
}

bool shadowArmExists() {
  return shadow::service().status().mode == shadow::ServiceMode::Armed;
}

void clearShadowArmIfPresent() {
  if (shadowArmExists()) shadow::service().clearPlaybackArm();
}

shadow::ResultCode shadowPlaybackEligibility(
    const shadow::SlotSummary& summary, shadow::ServiceMode mode) {
  const auto& config = aon::config::activeRobotConfig();
  return shadow::playbackEligibility(
      config.shadowPlaybackAuthorized, activeShadowRobot(),
      pros::competition::is_disabled(), summary, mode);
}

void saveAutonSelection(Alliance alliance, int index) {
  savedAutonSelection = {alliance, index, true};

  FILE* file = std::fopen(kAutonSelectionPath, "w");
  if (file == nullptr) return;
  std::fprintf(file, "%d %d\n", static_cast<int>(alliance), index);
  std::fclose(file);
}

void loadAutonSelection() {
  if (savedAutonSelection.hasSelection) return;

  FILE* file = std::fopen(kAutonSelectionPath, "r");
  if (file == nullptr) return;

  int alliance = 0;
  int index = 0;
  const int parsed = std::fscanf(file, "%d %d", &alliance, &index);
  std::fclose(file);

  if (parsed != 2 || index < 1 || index > autonOptionsCount) return;
  if (alliance < static_cast<int>(Alliance::Red) ||
      alliance > static_cast<int>(Alliance::Skills)) {
    return;
  }

  savedAutonSelection = {static_cast<Alliance>(alliance), index, true};
}

}  // namespace

// Owning GUI instance — type selected at compile time by TESTING_AUTONOMOUS.
// A single std::unique_ptr<Gui> is used so no redundant reference alias is needed.
#if TESTING_AUTONOMOUS
std::unique_ptr<Gui> gui = std::make_unique<GuiDebug>();
#else
std::unique_ptr<Gui> gui = std::make_unique<Gui>();
#endif

// Define the autonomousReader unique_ptr
std::unique_ptr<FunctionReader<int>> autonomousReader =
    std::make_unique<FunctionReader<int>>();

// ============================================================================
// Helper Methods
// ============================================================================

int Gui::displayInitializationMessage() {
  // Clear the screen completely
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Typewriter-style primary message (shows debug text when enabled for
  // testing)
#if TESTING_AUTONOMOUS
  const char* msg = "Initializing Debug...";
#else
  const char* msg = "Initializing AON...";
#endif
  int len = 0;
  for (const char* p = msg; *p; ++p) ++len;

  // Estimate character dimensions (matches Button helpers)
  int charWidth = 18;  // pixels for large text
  int charHeight = 20;

  int textWidth = len * charWidth;
  int startX = (BRAIN_SCREEN_WIDTH - textWidth) / 2;
  int startY = BRAIN_SCREEN_HEIGHT / 3;  // higher on screen

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
  int secLen = 0;
  for (const char* p = secMsg; *p; ++p) ++secLen;
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

void Gui::applyPreselectedAuton() {
  // If user already directly set selectedAuton, don't override.
  if (selectedAuton.routine != nullptr && selectedAutonName != "None" &&
      (selectedRedAut == 0 && selectedBlueAut == 0 && selectedSkill == 0)) {
    return;  // Nothing to map; routine already chosen
  }

  loadAutonSelection();
  if (savedAutonSelection.hasSelection) {
    selectAutonByList(savedAutonSelection.alliance, savedAutonSelection.index);
    return;
  }

  if (selectedRedAut > 0) {
    selectAutonByList(Alliance::Red, selectedRedAut);
    return;  // Red takes precedence
  }

  if (selectedBlueAut > 0) {
    selectAutonByList(Alliance::Blue, selectedBlueAut);
    return;  // Blue next
  }

  if (selectedSkill > 0) {
    selectAutonByList(Alliance::Skills, selectedSkill);
  }
}

void Gui::selectAutonByList(Alliance alliance, int index1Based) {
  if (index1Based < 1) index1Based = 1;
  if (index1Based > 3) index1Based = 3;

  const AutonOption* options = nullptr;
  switch (alliance) {
    case Alliance::Red:
      options = redAutonOptions;
      selectedRedAut = index1Based;
      selectedBlueAut = 0;
      selectedSkill = 0;
      break;
    case Alliance::Blue:
      options = blueAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = index1Based;
      selectedSkill = 0;
      break;
    case Alliance::Skills:
      options = skillsAutonOptions;
      selectedRedAut = 0;
      selectedBlueAut = 0;
      selectedSkill = index1Based;
      break;
  }
  ALLIANCE = alliance;
  const AutonOption& choice = options[index1Based - 1];
  selectedAuton = choice;
  selectedAutonName = choice.name;
  selectedAutonInvoker = nullptr;
  aon::auton::selectRoutine(choice.name);

  saveAutonSelection(alliance, index1Based);
  autonomousReader->AddFunction("autonomous", choice.routine);

  if (currentScreen == MainMenu) {
    displayMainMenu();
  }
}

// ============================================================================
// Touch Handlers
// ============================================================================

void Gui::handleMainMenuTouch(
    const pros::screen_touch_status_s_t& touchStatus) {
  // Check if AUTONS button is pressed
  if (AutonsBtn.isHit(touchStatus.x, touchStatus.y)) {
    if (currentScreen != AutonMenu) {
      displayAutonMenu();
      currentScreen = AutonMenu;
    }
  } else if (ShadowBtn.isHit(touchStatus.x, touchStatus.y)) {
    shadowConfirmation = ShadowConfirmation::None;
    shadowHasActionResult = false;
    shadowDeleteSucceeded = false;
    refreshShadowSlots();
    shadowLastStatusChange = shadow::service().status().changedAt;
    displayShadowMenu();
    currentScreen = ShadowMenu;
  }
}

void Gui::handleAutonMenuTouch() {
  if (currentScreen != AutonMenu) return;

  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (backBtnGray.isHit(x, y)) {
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (blueBtn.isHit(x, y)) {
    displayBlueAutonMenu();
    currentScreen = BlueAutons;
  } else if (redBtn.isHit(x, y)) {
    displayRedAutonMenu();
    currentScreen = RedAutons;
  } else if (skillsBtn.isHit(x, y)) {
    displaySkillsMenu();
    currentScreen = SkillAutons;
  } else if (fallbackModeBtn.isHit(x, y)) {
    const auto status = aon::auton::fallbackStatus();
    const bool force = status.mode != aon::auton::MotionMode::ForcedEncoder;
    if (aon::auton::selectForcedEncoder(force)) displayAutonMenu();
  }
}

void Gui::handleRedAutonMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (backBtnRed.isHit(x, y)) {
    displayAutonMenu();
    currentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (aut1Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Red, 1);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Red, 2);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Red, 3);
    displayMainMenu();
    currentScreen = MainMenu;
  }
}

void Gui::handleBlueAutonMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnBlue.isHit(x, y)) {
    displayAutonMenu();
    currentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (aut1Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Blue, 1);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Blue, 2);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Blue, 3);
    displayMainMenu();
    currentScreen = MainMenu;
  }
}

void Gui::handleSkillsMenuTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x, y = touch.y;

  if (BackBtnGreen.isHit(x, y)) {
    displayAutonMenu();
    currentScreen = AutonMenu;
    return;
  }

  // Check auton selection buttons
  if (aut1Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Skills, 1);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut2Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Skills, 2);
    displayMainMenu();
    currentScreen = MainMenu;
  } else if (Aut3Btn.isHit(x, y)) {
    selectAutonByList(Alliance::Skills, 3);
    displayMainMenu();
    currentScreen = MainMenu;
  }
}

bool Gui::refreshShadowSlots() {
  bool changed = false;
  for (std::size_t index = 0; index < shadowSlots.size(); ++index) {
    const auto latest =
        shadow::service().slot(static_cast<std::uint8_t>(index + 1));
    if (!sameSlotSummary(latest, shadowSlots[index])) {
      shadowSlots[index] = latest;
      changed = true;
    }
  }
  shadowLastSlotPollAt = pros::millis();
  return changed;
}

void Gui::handleShadowMenuTouch() {
  if (currentScreen != ShadowMenu || shadowSaving) return;

  const auto touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;
  const int x = touch.x;
  const int y = touch.y;
  const auto status = shadow::service().status();
  const bool busy = status.mode == shadow::ServiceMode::Processing;
  const std::uint32_t now = pros::millis();

  // Processing may also be initiated by the recorder service task. Keep every
  // control inert until its SD operation finishes.
  if (busy) return;

  if (shadowBackBtn.isHit(x, y)) {
    clearShadowArmIfPresent();
    shadowConfirmation = ShadowConfirmation::None;
    displayMainMenu();
    currentScreen = MainMenu;
    return;
  }

  if (status.mode != shadow::ServiceMode::Recording) {
    std::uint8_t slot = 0;
    if (shadowSlot1Btn.isHit(x, y)) slot = 1;
    else if (shadowSlot2Btn.isHit(x, y)) slot = 2;
    else if (shadowSlot3Btn.isHit(x, y)) slot = 3;
    if (slot != 0) {
      if (slot != selectedShadowSlot) clearShadowArmIfPresent();
      selectedShadowSlot = slot;
      shadowConfirmation = ShadowConfirmation::None;
      shadowHasActionResult = false;
      shadowDeleteSucceeded = false;
      displayShadowMenu();
      return;
    }
  }

  if (shadowRecordBtn.isHit(x, y)) {
    if (status.mode == shadow::ServiceMode::Recording) {
      shadowSaving = true;
      shadowConfirmation = ShadowConfirmation::None;
      displayShadowMenu();
      const auto result = shadow::service().stopAndSave();
      shadowSaving = false;
      shadowActionResult = result;
      shadowHasActionResult = result != shadow::ResultCode::Ok;
      shadowDeleteSucceeded = false;
      refreshShadowSlots();
      shadowLastStatusChange = shadow::service().status().changedAt;
      displayShadowMenu();
      return;
    }
    const bool occupied = shadowSlots[selectedShadowSlot - 1].valid;
    const bool confirmed =
        shadowConfirmation == ShadowConfirmation::Overwrite &&
        !timeReached(now, shadowConfirmationExpiresAt);
    if (occupied && !confirmed) {
      shadowConfirmation = ShadowConfirmation::Overwrite;
      shadowConfirmationExpiresAt = now + 5000;
      shadowHasActionResult = false;
      shadowDeleteSucceeded = false;
      displayShadowMenu();
      return;
    }

    clearShadowArmIfPresent();
    const auto result =
        shadow::service().beginRecording(selectedShadowSlot, confirmed);
    shadowConfirmation = ShadowConfirmation::None;
    shadowActionResult = result;
    shadowHasActionResult = result != shadow::ResultCode::Ok;
    shadowDeleteSucceeded = false;
    shadowLastStatusChange = shadow::service().status().changedAt;
    displayShadowMenu();
    return;
  }

  if (shadowDeleteBtn.isHit(x, y) &&
      status.mode != shadow::ServiceMode::Recording) {
    clearShadowArmIfPresent();
    if (shadowSlots[selectedShadowSlot - 1].result ==
        shadow::ResultCode::EmptyRecording) {
      shadowActionResult = shadow::ResultCode::EmptyRecording;
      shadowHasActionResult = true;
      shadowDeleteSucceeded = false;
      displayShadowMenu();
      return;
    }
    const bool confirmed =
        shadowConfirmation == ShadowConfirmation::Delete &&
        !timeReached(now, shadowConfirmationExpiresAt);
    if (!confirmed) {
      shadowConfirmation = ShadowConfirmation::Delete;
      shadowConfirmationExpiresAt = now + 5000;
      shadowHasActionResult = false;
      shadowDeleteSucceeded = false;
      displayShadowMenu();
      return;
    }

    shadowSaving = true;
    shadowConfirmation = ShadowConfirmation::None;
    displayShadowMenu();
    const auto result = shadow::service().erase(selectedShadowSlot, true);
    shadowSaving = false;
    shadowActionResult = result;
    shadowHasActionResult = result != shadow::ResultCode::Ok;
    shadowDeleteSucceeded = result == shadow::ResultCode::Ok;
    refreshShadowSlots();
    displayShadowMenu();
    return;
  }

  if (shadowPlayBtn.isHit(x, y)) {
    if (status.mode == shadow::ServiceMode::Armed ||
        status.mode == shadow::ServiceMode::Playing) {
      return;
    }
    const auto eligibility = shadowPlaybackEligibility(
        shadowSlots[selectedShadowSlot - 1], status.mode);
    if (eligibility != shadow::ResultCode::Ok) {
      shadowConfirmation = ShadowConfirmation::None;
      shadowActionResult = eligibility;
      shadowHasActionResult = true;
      shadowDeleteSucceeded = false;
      displayShadowMenu();
      return;
    }

    const bool confirmed = shadowConfirmation == ShadowConfirmation::Play &&
                           !timeReached(now, shadowConfirmationExpiresAt);
    if (!confirmed) {
      shadowConfirmation = ShadowConfirmation::Play;
      shadowConfirmationExpiresAt = now + kShadowPlayConfirmationMs;
      shadowHasActionResult = false;
      shadowDeleteSucceeded = false;
      displayShadowMenu();
      return;
    }

    const auto result = shadow::service().armPlayback(
        selectedShadowSlot, true, pros::competition::is_disabled());
    shadowConfirmation = ShadowConfirmation::None;
    shadowActionResult = result;
    shadowHasActionResult = result != shadow::ResultCode::Ok;
    shadowDeleteSucceeded = false;
    if (result == shadow::ResultCode::Ok) {
      selectAutonByList(Alliance::Skills, 3);
      shadowLastStatusChange = shadow::service().status().changedAt;
    }
    displayShadowMenu();
  }
}

void Gui::updateShadowMenu() {
  if (currentScreen != ShadowMenu) return;

  const std::uint32_t now = pros::millis();
  const auto shadowStatus = shadow::service().status();
  bool redraw = shadowStatus.changedAt != shadowLastStatusChange;
  if (redraw) {
    shadowLastStatusChange = shadowStatus.changedAt;
  }
  if (!shadowSaving && shadowStatus.mode != shadow::ServiceMode::Recording &&
      shadowStatus.mode != shadow::ServiceMode::Processing &&
      now - shadowLastSlotPollAt >= 1000) {
    redraw = refreshShadowSlots() || redraw;
  }
  if (shadowConfirmation != ShadowConfirmation::None &&
      timeReached(now, shadowConfirmationExpiresAt)) {
    clearShadowArmIfPresent();
    shadowConfirmation = ShadowConfirmation::None;
    redraw = true;
  }
  if (shadow::playbackArmExpired(shadowStatus, now,
                                 kShadowPlayConfirmationMs)) {
    clearShadowArmIfPresent();
    redraw = true;
  }
  if (redraw) displayShadowMenu();
}

// ============================================================================
// GUI Loop
// ============================================================================

void Gui::mainLoop() {
  auto lastStatus = aon::auton::routineStatus();
  auto lastFallback = aon::auton::fallbackStatus();
  while (true) {
    pros::screen_touch_status_s_t TouchStatus = pros::screen::touch_status();
    if (TouchStatus.touch_status <= 0) touchLatched = false;
    if (TouchStatus.touch_status > 0 && !touchLatched) {
      touchLatched = true;
      switch (currentScreen) {
        case MainMenu:
          handleMainMenuTouch(TouchStatus);
          break;
        case AutonMenu:
          handleAutonMenuTouch();
          break;
        case RedAutons:
          handleRedAutonMenuTouch();
          break;
        case BlueAutons:
          handleBlueAutonMenuTouch();
          break;
        case SkillAutons:
          handleSkillsMenuTouch();
          break;
        case ShadowMenu:
          handleShadowMenuTouch();
          break;
        default:
          break;
      }
    }

    const auto status = aon::auton::routineStatus();
    const auto fallback = aon::auton::fallbackStatus();
    if (currentScreen == MainMenu &&
        (status.state != lastStatus.state || status.name != lastStatus.name ||
         fallback.changedAt != lastFallback.changedAt)) {
      displayMainMenu();
    }
    lastStatus = status;
    lastFallback = fallback;

    updateShadowMenu();

    pros::delay(100);
  }
}

// ============================================================================
// Initialization
// ============================================================================

void Gui::initialize() {
  std::cout << "Start" << std::endl;

  pros::delay(5);

  currentScreen = MainMenu;

  displayInitializationMessage();

  // Keep initialization message visible briefly before showing main menu
  pros::delay(1000);

  displayMainMenu();

  applyPreselectedAuton();

  this->mainLoop();
}

int Gui::invokeSelectedAuton() {
  if (selectedAuton.routine != nullptr) return selectedAuton.routine();
  return 0;
}

}  // namespace aon
