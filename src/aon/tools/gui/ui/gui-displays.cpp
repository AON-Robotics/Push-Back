#include "aon/tools/gui/gui.hpp"
#include "aon/tools/gui/gui-debug.hpp"
#include "aon/auton/fallback-status.hpp"
#include "aon/config/robot-config.hpp"
#include "aon/constants.hpp"
#include "aon/tools/gui/ui/gui-layout.hpp"
#include "aon/tools/gui-image-generator/gui-images.hpp"

#include <array>
#include <cstdio>

namespace aon {
namespace {

const char* shadowResultName(shadow::ResultCode result) {
  switch (result) {
    case shadow::ResultCode::Ok: return "OK";
    case shadow::ResultCode::NotRecording: return "NOT RECORDING";
    case shadow::ResultCode::AlreadyRecording: return "ALREADY RECORDING";
    case shadow::ResultCode::SampleTooSoon: return "SAMPLE TOO SOON";
    case shadow::ResultCode::DuplicateEvent: return "DUPLICATE EVENT";
    case shadow::ResultCode::CapacityReached: return "CAPACITY REACHED";
    case shadow::ResultCode::InvalidPose: return "INVALID POSE";
    case shadow::ResultCode::PoseJump: return "POSE JUMP";
    case shadow::ResultCode::EmptyRecording: return "EMPTY";
    case shadow::ResultCode::NoSd: return "NO SD";
    case shadow::ResultCode::ReadOnly: return "SD READ ONLY";
    case shadow::ResultCode::OpenFailed: return "OPEN FAILED";
    case shadow::ResultCode::ReadFailed: return "READ FAILED";
    case shadow::ResultCode::WriteFailed: return "WRITE FAILED";
    case shadow::ResultCode::FlushFailed: return "FLUSH FAILED";
    case shadow::ResultCode::CloseFailed: return "CLOSE FAILED";
    case shadow::ResultCode::DeleteFailed: return "DELETE FAILED";
    case shadow::ResultCode::WriteCleanupFailed:
      return "WRITE+CLEANUP FAILED";
    case shadow::ResultCode::FlushCleanupFailed:
      return "FLUSH+CLEANUP FAILED";
    case shadow::ResultCode::CloseCleanupFailed:
      return "CLOSE+CLEANUP FAILED";
    case shadow::ResultCode::CorruptFile: return "CORRUPT";
    case shadow::ResultCode::UnsupportedVersion: return "BAD VERSION";
    case shadow::ResultCode::WrongRobot: return "WRONG ROBOT";
    case shadow::ResultCode::InvalidSlot: return "INVALID SLOT";
    case shadow::ResultCode::PlayLocked: return "PLAY LOCKED";
    case shadow::ResultCode::UnsafeState: return "UNSAFE STATE";
    case shadow::ResultCode::Cancelled: return "CANCELLED";
    case shadow::ResultCode::OdometryFailure: return "ODOM FAILURE";
    case shadow::ResultCode::MotionFailure: return "MOTION FAILURE";
    case shadow::ResultCode::UnsupportedRobot: return "UNSUPPORTED ROBOT";
  }
  return "UNKNOWN";
}

const char* shadowSlotState(const shadow::SlotSummary& slot) {
  if (slot.valid) return "READY";
  if (slot.result == shadow::ResultCode::EmptyRecording) return "EMPTY";
  return "INVALID";
}

const char* shadowModeName(shadow::ServiceMode mode) {
  switch (mode) {
    case shadow::ServiceMode::Idle: return "IDLE";
    case shadow::ServiceMode::Recording: return "RECORDING";
    case shadow::ServiceMode::Processing: return "PROCESSING";
    case shadow::ServiceMode::Saved: return "SAVED";
    case shadow::ServiceMode::Invalid: return "INVALID";
    case shadow::ServiceMode::Armed: return "ARMED";
    case shadow::ServiceMode::Playing: return "PLAYING";
    case shadow::ServiceMode::Finished: return "FINISHED";
    case shadow::ServiceMode::Cancelled: return "CANCELLED";
  }
  return "UNKNOWN";
}

shadow::ResultCode shadowPlayEligibility(const shadow::SlotSummary& summary,
                                         shadow::ServiceMode mode) {
  const auto& config = aon::config::activeRobotConfig();
  const auto robot = config.identity == aon::config::RobotIdentity::Small
                         ? shadow::RobotIdentity::Small
                         : shadow::RobotIdentity::Big;
  return shadow::playbackEligibility(
      config.shadowPlaybackAuthorized, robot, summary, mode);
}

}  // namespace

void Gui::displayFallbackStatusLine() {
  const auto fallback = aon::auton::fallbackStatus();
  const char* mode = "TRACKING";
  pros::screen::set_pen(COLOR_GREEN);
  if (fallback.mode == aon::auton::MotionMode::ForcedEncoder) {
    mode = "FORCED ENCODER";
    pros::screen::set_pen(COLOR_ORANGE);
  } else if (fallback.mode == aon::auton::MotionMode::FaultedEncoder) {
    mode = "FAULT ENCODER";
    pros::screen::set_pen(COLOR_RED);
  }

  if (fallback.reason == aon::auton::MotionFailureReason::None) {
    pros::screen::print(pros::E_TEXT_SMALL, 5, 35, "ODOM: %s", mode);
  } else {
    pros::screen::print(pros::E_TEXT_SMALL, 5, 35, "ODOM: %s - %s", mode,
                        aon::auton::motionFailureName(fallback.reason));
  }
}

void Gui::displayAutonStatusLine() {
  const auto status = aon::auton::routineStatus();
  const char* label = "Selected";
  pros::screen::set_pen(COLOR_GREEN);

  switch (status.state) {
    case aon::auton::RoutineState::Idle:
      label = "No auton";
      pros::screen::set_pen(COLOR_WHITE);
      break;
    case aon::auton::RoutineState::Selected:
      break;
    case aon::auton::RoutineState::Running:
      label = "Running";
      pros::screen::set_pen(COLOR_ORANGE);
      break;
    case aon::auton::RoutineState::Completed:
      label = "Completed";
      pros::screen::set_pen(COLOR_CYAN);
      break;
    case aon::auton::RoutineState::Failed:
      label = "Failed";
      pros::screen::set_pen(COLOR_RED);
      break;
  }

  if (status.state == aon::auton::RoutineState::Idle) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "%s", label);
    return;
  }
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "%s: %s", label,
                      status.name.c_str());
}

void Gui::displayMainMenu() {
  // Ensure the screen is cleared at the start of each display function
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  aon::DrawAONLogo((BRAIN_SCREEN_WIDTH - 225) / 2, (BRAIN_SCREEN_HEIGHT - 225) / 4);

  displayAutonStatusLine();
  displayFallbackStatusLine();

  // Draw the "AUTONS" button using UI helper
  AutonsBtn.draw(pros::E_TEXT_LARGE);
  ShadowBtn.draw(pros::E_TEXT_LARGE);
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

  const auto fallback = aon::auton::fallbackStatus();
  ui::Button modeButton = fallbackModeBtn;
  modeButton.label =
      fallback.mode == aon::auton::MotionMode::ForcedEncoder
          ? "FORCE ENCODERS"
          : "AUTO FALLBACK";
  modeButton.bg =
      fallback.mode == aon::auton::MotionMode::ForcedEncoder
          ? COLOR_ORANGE
          : COLOR_DARK_GREEN;
  if (fallback.mode == aon::auton::MotionMode::Tracking &&
      !aon::config::activeRobotConfig()
           .lemlib.fallback.automaticFallbackAuthorized) {
    modeButton.label = "AUTO LOCKED";
    modeButton.bg = COLOR_DARK_GRAY;
  }
  if (fallback.mode == aon::auton::MotionMode::FaultedEncoder) {
    modeButton.label = "FAULT ENCODER";
    modeButton.bg = COLOR_RED;
  }
  modeButton.draw(pros::E_TEXT_SMALL);
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

void Gui::displayShadowMenu() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  std::array<ui::Button, shadow::kSlotCount> slots = {
      shadowSlot1Btn, shadowSlot2Btn, shadowSlot3Btn};
  for (std::size_t index = 0; index < slots.size(); ++index) {
    const auto& summary = shadowSlots[index];
    char label[24];
    std::snprintf(label, sizeof(label), "SLOT %u %s",
                  static_cast<unsigned>(index + 1),
                  shadowSlotState(summary));
    slots[index].label = label;
    slots[index].bg = summary.valid
                          ? COLOR_DARK_GREEN
                          : summary.result == shadow::ResultCode::EmptyRecording
                                ? COLOR_DARK_GRAY
                                : COLOR_DARK_RED;
    if (selectedShadowSlot == index + 1) slots[index].bg = COLOR_BLUE;
    slots[index].draw(pros::E_TEXT_SMALL);
  }

  const auto& selected = shadowSlots[selectedShadowSlot - 1];
  pros::screen::set_pen(selected.valid ? COLOR_GREEN : COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 68, "SLOT %u: %s",
                      static_cast<unsigned>(selectedShadowSlot),
                      shadowSlotState(selected));
  if (selected.valid) {
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 96,
                        "DURATION %.1fs  GENERATION %lu",
                        selected.durationMs / 1000.0,
                        static_cast<unsigned long>(selected.generation));
    pros::screen::print(pros::E_TEXT_SMALL, 10, 116,
                        "START X %.1f  Y %.1f  H %.1f", selected.startX,
                        selected.startY, selected.startHeading);
  } else {
    pros::screen::set_pen(selected.result == shadow::ResultCode::EmptyRecording
                              ? COLOR_GRAY
                              : COLOR_RED);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 100, "%s",
                        shadowResultName(selected.result));
  }

  const auto status = shadow::service().status();
  const auto playEligibility = shadowPlayEligibility(selected, status.mode);
  pros::screen::set_pen(shadowSaving ||
                                status.mode == shadow::ServiceMode::Processing
                            ? COLOR_ORANGE
                            : status.mode == shadow::ServiceMode::Invalid
                                  ? COLOR_RED
                                  : COLOR_CYAN);
  pros::screen::print(pros::E_TEXT_SMALL, 10, 142, "STATUS: %s",
                      shadowSaving
                          ? "PROCESSING"
                          : shadowDeleteSucceeded ? "DELETED"
                                                  : shadowModeName(status.mode));

  pros::screen::set_pen(COLOR_ORANGE);
  if (shadowConfirmation == ShadowConfirmation::Overwrite) {
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164,
                        "TAP OVERWRITE? AGAIN WITHIN 5s");
  } else if (shadowConfirmation == ShadowConfirmation::Delete) {
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164,
                        "TAP DELETE? AGAIN WITHIN 5s");
  } else if (shadowConfirmation == ShadowConfirmation::Play) {
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164,
                        "VERIFY START POSE; CONFIRM WITHIN 5s");
  } else if (shadowHasActionResult) {
    pros::screen::set_pen(shadowActionResult == shadow::ResultCode::PlayLocked
                              ? COLOR_ORANGE
                              : COLOR_RED);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164, "%s",
                        shadowResultName(shadowActionResult));
  } else if (shadowDeleteSucceeded) {
    pros::screen::set_pen(COLOR_GREEN);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164, "SLOT DELETED");
  } else if (status.mode == shadow::ServiceMode::Armed) {
    pros::screen::set_pen(COLOR_ORANGE);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164,
                        "START AUTON WITHIN 5s");
  } else if (status.result != shadow::ResultCode::Ok) {
    pros::screen::set_pen(COLOR_RED);
    pros::screen::print(pros::E_TEXT_SMALL, 10, 164, "%s",
                        shadowResultName(status.result));
  } else {
    pros::screen::set_pen(playEligibility == shadow::ResultCode::Ok
                              ? COLOR_GREEN
                              : COLOR_ORANGE);
    pros::screen::print(
        pros::E_TEXT_SMALL, 10, 164, "%s",
        playEligibility == shadow::ResultCode::Ok
            ? "READY: VERIFY START POSE BEFORE PLAY"
            : shadowResultName(playEligibility));
  }

  shadowBackBtn.draw(pros::E_TEXT_SMALL);

  ui::Button record = shadowRecordBtn;
  if (shadowSaving || status.mode == shadow::ServiceMode::Processing) {
    record.label = "PROCESSING";
    record.bg = COLOR_DARK_GRAY;
  } else if (status.mode == shadow::ServiceMode::Recording) {
    record.label = "STOP SAVE";
    record.bg = COLOR_ORANGE;
  } else if (shadowConfirmation == ShadowConfirmation::Overwrite) {
    record.label = "OVERWRITE?";
    record.bg = COLOR_ORANGE;
  }
  record.draw(pros::E_TEXT_SMALL);

  ui::Button remove = shadowDeleteBtn;
  if (shadowSaving || status.mode == shadow::ServiceMode::Processing ||
      status.mode == shadow::ServiceMode::Recording) {
    remove.bg = COLOR_DARK_GRAY;
  } else if (shadowConfirmation == ShadowConfirmation::Delete) {
    remove.label = "DELETE?";
    remove.bg = COLOR_ORANGE;
  }
  remove.draw(pros::E_TEXT_SMALL);
  ui::Button play = shadowPlayBtn;
  if (shadowConfirmation == ShadowConfirmation::Play) {
    play.label = "CONFIRM PLAY";
    play.bg = COLOR_ORANGE;
  } else if (status.mode == shadow::ServiceMode::Armed) {
    play.label = "ARMED";
    play.bg = COLOR_GREEN;
  } else if (status.mode == shadow::ServiceMode::Playing) {
    play.label = "PLAYING";
    play.bg = COLOR_ORANGE;
  } else if (status.mode == shadow::ServiceMode::Finished) {
    play.label = "FINISHED";
    play.bg = COLOR_GREEN;
  } else if (status.mode == shadow::ServiceMode::Cancelled) {
    play.label = "CANCELLED";
    play.bg = COLOR_DARK_GRAY;
  } else if (status.mode == shadow::ServiceMode::Invalid &&
             status.result != shadow::ResultCode::Ok) {
    play.label = shadowResultName(status.result);
    play.bg = COLOR_RED;
  } else if (playEligibility == shadow::ResultCode::Ok) {
    play.label = "PLAY";
    play.bg = COLOR_DARK_GREEN;
  } else {
    play.label = "PLAY LOCKED";
    play.bg = COLOR_DARK_GRAY;
  }
  play.draw(pros::E_TEXT_SMALL);
}

}  // namespace aon
