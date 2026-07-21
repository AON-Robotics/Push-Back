#include "aon/shadow/mechanisms.hpp"

#include "aon/constants.hpp"
#include "aon/globals.hpp"
#include "aon/lemlib/drive-io.hpp"

namespace aon::shadow {
namespace {

constexpr RobotIdentity kRobotIdentity =
#if USING_BIG_ROBOT
    RobotIdentity::Big;
#else
    RobotIdentity::Small;
#endif

void setPiston(Piston& piston, bool activated) {
  if (activated) {
    piston.activate();
  } else {
    piston.deactivate();
  }
}

}  // namespace

void captureDrive(int left, int right) {
  aon::lemlib_integration::setEffectiveDriveCommand(left, right);
}

void applyDriverIntakeIntent(IntakeIntent intent) {
#if USING_BIG_ROBOT
  if (intent != IntakeIntent::SortNormal &&
      intent != IntakeIntent::SortInverted) {
    intake.stopReleasing();
  }
  switch (intent) {
    case IntakeIntent::Idle:
      intake.stop();
      break;
    case IntakeIntent::Store:
      intake.store();
      break;
    case IntakeIntent::ScoreBottom:
      intake.score(Intake::BOTTOM);
      break;
    case IntakeIntent::ScoreMiddle:
      intake.score(Intake::MIDDLE);
      break;
    case IntakeIntent::ScoreTop:
      intake.score(Intake::TOP);
      break;
    case IntakeIntent::SortNormal:
      intake.setSortHeights(Intake::TOP);
      intake.startReleasing();
      break;
    case IntakeIntent::SortInverted:
      intake.setSortHeights(Intake::MIDDLE);
      intake.startReleasing();
      break;
    case IntakeIntent::Corridor:
    case IntakeIntent::Reject:
      break;
  }
#else
  switch (intent) {
    case IntakeIntent::Idle:
      intake.stop();
      break;
    case IntakeIntent::Store:
      intake.store();
      break;
    case IntakeIntent::Corridor:
      intake.corridor();
      break;
    case IntakeIntent::Reject:
      intake.reject();
      break;
    case IntakeIntent::ScoreBottom:
      intake.score(Intake::BOTTOM);
      break;
    case IntakeIntent::ScoreMiddle:
    case IntakeIntent::ScoreTop:
    case IntakeIntent::SortNormal:
    case IntakeIntent::SortInverted:
      break;
  }
#endif
}

ResultCode applyMechanism(const MechanismEvent& event) {
  const ResultCode validation = validateMechanism(kRobotIdentity, event);
  if (validation != ResultCode::Ok) return validation;

  const bool active = event.value == 1;
  switch (event.kind) {
    case MechanismKind::IntakeMode:
      applyDriverIntakeIntent(static_cast<IntakeIntent>(event.value));
      break;
    case MechanismKind::Cart:
      if (active) intake.dropCart();
      else intake.raiseCart();
      break;
    case MechanismKind::Brooks:
      setPiston(brooks, active);
      break;
#if USING_BIG_ROBOT
    case MechanismKind::Sem:
      setPiston(sem, active);
      break;
    case MechanismKind::ScorerHeight:
    case MechanismKind::Trapdoor:
    case MechanismKind::Lever:
    case MechanismKind::Arrow:
      return ResultCode::WrongRobot;
#else
    case MechanismKind::ScorerHeight:
      if (active) intake.raiseScorer();
      else intake.lowerScorer();
      break;
    case MechanismKind::Trapdoor:
      if (active) intake.openTrapdoor();
      else intake.closeTrapdoor();
      break;
    case MechanismKind::Lever:
      if (active) intake.extendLever();
      else intake.resetLever();
      break;
    case MechanismKind::Arrow:
      setPiston(arrow, active);
      break;
    case MechanismKind::Sem:
      return ResultCode::WrongRobot;
#endif
  }
  return ResultCode::Ok;
}

}  // namespace aon::shadow
