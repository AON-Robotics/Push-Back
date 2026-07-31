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

ResultCode applyIntakeAction(IntakeAdapterAction action) {
#if USING_BIG_ROBOT
  switch (action) {
    case IntakeAdapterAction::StopSortAndWait:
      return intake.stopReleasingAndWait() ? ResultCode::Ok
                                           : ResultCode::MotionFailure;
    case IntakeAdapterAction::Stop:
      intake.stop();
      break;
    case IntakeAdapterAction::Store:
      intake.store();
      break;
    case IntakeAdapterAction::ScoreBottom:
      intake.score(Intake::BOTTOM);
      break;
    case IntakeAdapterAction::ScoreMiddle:
      intake.score(Intake::MIDDLE);
      break;
    case IntakeAdapterAction::ScoreTop:
      intake.score(Intake::TOP);
      break;
    case IntakeAdapterAction::SortNormal:
      return intake.startReleasing(Intake::TOP) ? ResultCode::Ok
                                                 : ResultCode::MotionFailure;
    case IntakeAdapterAction::SortInverted:
      return intake.startReleasing(Intake::MIDDLE)
                 ? ResultCode::Ok
                 : ResultCode::MotionFailure;
    case IntakeAdapterAction::Corridor:
    case IntakeAdapterAction::Reject:
      break;
  }
#else
  switch (action) {
    case IntakeAdapterAction::Stop:
      intake.stop();
      break;
    case IntakeAdapterAction::Store:
      intake.store();
      break;
    case IntakeAdapterAction::Corridor:
      intake.corridor();
      break;
    case IntakeAdapterAction::Reject:
      intake.reject();
      break;
    case IntakeAdapterAction::ScoreBottom:
      intake.score(Intake::BOTTOM);
      break;
    case IntakeAdapterAction::StopSortAndWait:
    case IntakeAdapterAction::ScoreMiddle:
    case IntakeAdapterAction::ScoreTop:
    case IntakeAdapterAction::SortNormal:
    case IntakeAdapterAction::SortInverted:
      break;
  }
#endif
  return ResultCode::Ok;
}

}  // namespace

void captureDrive(int left, int right) {
  aon::lemlib_integration::setEffectiveDriveCommand(left, right);
}

ResultCode applyDriverIntakeIntent(IntakeIntent intent) {
  const IntakeAdapterPlan plan = intakeAdapterPlan(kRobotIdentity, intent);
  for (std::uint8_t index = 0; index < plan.count; ++index) {
    const ResultCode result = applyIntakeAction(plan.actions[index]);
    if (result != ResultCode::Ok) return result;
  }
  return ResultCode::Ok;
}

ResultCode applyMechanism(const MechanismEvent& event) {
  const MechanismAdapterPlan plan = planMechanismAdapter(kRobotIdentity, event);
  if (plan.result != ResultCode::Ok) return plan.result;

  const bool active = plan.value == 1;
  switch (plan.target) {
    case MechanismAdapterTarget::Intake:
      return applyDriverIntakeIntent(static_cast<IntakeIntent>(plan.value));
    case MechanismAdapterTarget::Cart:
      if (active) intake.dropCart();
      else intake.raiseCart();
      break;
    case MechanismAdapterTarget::Brooks:
      setPiston(brooks, active);
      break;
#if USING_BIG_ROBOT
    case MechanismAdapterTarget::Sem:
      setPiston(sem, active);
      break;
    case MechanismAdapterTarget::ScorerHeight:
    case MechanismAdapterTarget::Trapdoor:
    case MechanismAdapterTarget::Lever:
    case MechanismAdapterTarget::Arrow:
      return ResultCode::WrongRobot;
#else
    case MechanismAdapterTarget::ScorerHeight:
      if (active) intake.raiseScorer();
      else intake.lowerScorer();
      break;
    case MechanismAdapterTarget::Trapdoor:
      if (active) intake.openTrapdoor();
      else intake.closeTrapdoor();
      break;
    case MechanismAdapterTarget::Lever:
      if (active) intake.extendLever();
      else intake.resetLever();
      break;
    case MechanismAdapterTarget::Arrow:
      setPiston(arrow, active);
      break;
    case MechanismAdapterTarget::Sem:
      return ResultCode::WrongRobot;
#endif
    case MechanismAdapterTarget::None:
      return ResultCode::CorruptFile;
  }
  return ResultCode::Ok;
}

void stopAllMechanisms() {
#if USING_BIG_ROBOT
  intake.stopReleasingAndWait();
#endif
  intake.stopScan();
  intake.stop();
}

}  // namespace aon::shadow
