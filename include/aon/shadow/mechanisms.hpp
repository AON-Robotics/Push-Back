#pragma once

#include "aon/shadow/types.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace aon::shadow {

enum class IntakeIntent : std::int16_t {
  Idle,
  Store,
  Corridor,
  Reject,
  ScoreBottom,
  ScoreMiddle,
  ScoreTop,
  SortNormal,
  SortInverted,
};

constexpr IntakeIntent smallIntakeDecision(bool storeHeld, bool rejectHeld,
                                           bool scoreBottomHeld,
                                           bool storeIncludesElevator) {
  if (storeHeld) {
    return storeIncludesElevator ? IntakeIntent::Store
                                 : IntakeIntent::Corridor;
  }
  if (rejectHeld) return IntakeIntent::Reject;
  if (scoreBottomHeld) return IntakeIntent::ScoreBottom;
  return IntakeIntent::Idle;
}

constexpr IntakeIntent bigIntakeDecision(bool storeHeld,
                                         bool scoreBottomHeld,
                                         bool rightOneHeld,
                                         bool rightTwoHeld,
                                         bool sortEnabled) {
  if (rightOneHeld) {
    return sortEnabled ? IntakeIntent::SortNormal : IntakeIntent::ScoreTop;
  }
  if (rightTwoHeld) {
    return sortEnabled ? IntakeIntent::SortInverted
                       : IntakeIntent::ScoreMiddle;
  }
  if (storeHeld) return IntakeIntent::Store;
  if (scoreBottomHeld) return IntakeIntent::ScoreBottom;
  return IntakeIntent::Idle;
}

enum class IntakeAdapterAction : std::uint8_t {
  StopSortAndWait,
  Stop,
  Store,
  Corridor,
  Reject,
  ScoreBottom,
  ScoreMiddle,
  ScoreTop,
  SortNormal,
  SortInverted,
};

struct IntakeAdapterPlan {
  std::array<IntakeAdapterAction, 2> actions{};
  std::uint8_t count = 0;
};

constexpr IntakeAdapterPlan intakeAdapterPlan(RobotIdentity robot,
                                               IntakeIntent intent) {
  IntakeAdapterPlan plan{};
  if (robot == RobotIdentity::Big && intent != IntakeIntent::SortNormal &&
      intent != IntakeIntent::SortInverted &&
      intent != IntakeIntent::Corridor && intent != IntakeIntent::Reject) {
    plan.actions[plan.count++] = IntakeAdapterAction::StopSortAndWait;
  }

  switch (intent) {
    case IntakeIntent::Idle:
      plan.actions[plan.count++] = IntakeAdapterAction::Stop;
      break;
    case IntakeIntent::Store:
      plan.actions[plan.count++] = IntakeAdapterAction::Store;
      break;
    case IntakeIntent::Corridor:
      if (robot == RobotIdentity::Small) {
        plan.actions[plan.count++] = IntakeAdapterAction::Corridor;
      }
      break;
    case IntakeIntent::Reject:
      if (robot == RobotIdentity::Small) {
        plan.actions[plan.count++] = IntakeAdapterAction::Reject;
      }
      break;
    case IntakeIntent::ScoreBottom:
      plan.actions[plan.count++] = IntakeAdapterAction::ScoreBottom;
      break;
    case IntakeIntent::ScoreMiddle:
      if (robot == RobotIdentity::Big) {
        plan.actions[plan.count++] = IntakeAdapterAction::ScoreMiddle;
      }
      break;
    case IntakeIntent::ScoreTop:
      if (robot == RobotIdentity::Big) {
        plan.actions[plan.count++] = IntakeAdapterAction::ScoreTop;
      }
      break;
    case IntakeIntent::SortNormal:
      if (robot == RobotIdentity::Big) {
        plan.actions[plan.count++] = IntakeAdapterAction::SortNormal;
      }
      break;
    case IntakeIntent::SortInverted:
      if (robot == RobotIdentity::Big) {
        plan.actions[plan.count++] = IntakeAdapterAction::SortInverted;
      }
      break;
  }
  return plan;
}

struct MechanismCapturePlan {
  bool record = false;
  MechanismEvent event{};
};

constexpr MechanismCapturePlan planMechanismCapture(
    bool recording, std::uint32_t now, std::uint32_t startedAt,
    MechanismKind kind, std::int16_t value) {
  if (!recording) return {};
  return {true, {now - startedAt, kind, value}};
}

struct DriveIntent {
  int left;
  int right;
};

inline DriveIntent normalizedArcadeDrive(double forward, double turn) {
  double left = forward + turn;
  double right = forward - turn;
  const double largest = std::max(std::abs(left), std::abs(right));
  if (largest > 1.0) {
    left /= largest;
    right /= largest;
  }
  return {
      std::clamp(static_cast<int>(std::lround(left * 127.0)), -127, 127),
      std::clamp(static_cast<int>(std::lround(right * 127.0)), -127, 127),
  };
}

constexpr ResultCode validateMechanism(RobotIdentity robot,
                                       const MechanismEvent& event) {
  if (event.kind == MechanismKind::Sem && robot == RobotIdentity::Small) {
    return ResultCode::WrongRobot;
  }
  if (robot == RobotIdentity::Big &&
      (event.kind == MechanismKind::Arrow ||
       event.kind == MechanismKind::ScorerHeight ||
       event.kind == MechanismKind::Trapdoor ||
       event.kind == MechanismKind::Lever)) {
    return ResultCode::WrongRobot;
  }

  switch (event.kind) {
    case MechanismKind::IntakeMode: {
      if (event.value < static_cast<std::int16_t>(IntakeIntent::Idle) ||
          event.value >
              static_cast<std::int16_t>(IntakeIntent::SortInverted)) {
        return ResultCode::CorruptFile;
      }
      const auto intent = static_cast<IntakeIntent>(event.value);
      if (robot == RobotIdentity::Small &&
          (intent == IntakeIntent::ScoreMiddle ||
           intent == IntakeIntent::ScoreTop ||
           intent == IntakeIntent::SortNormal ||
           intent == IntakeIntent::SortInverted)) {
        return ResultCode::WrongRobot;
      }
      if (robot == RobotIdentity::Big &&
          (intent == IntakeIntent::Corridor ||
           intent == IntakeIntent::Reject)) {
        return ResultCode::WrongRobot;
      }
      return ResultCode::Ok;
    }
    case MechanismKind::ScorerHeight:
    case MechanismKind::Cart:
    case MechanismKind::Trapdoor:
    case MechanismKind::Lever:
    case MechanismKind::Brooks:
    case MechanismKind::Sem:
    case MechanismKind::Arrow:
      return event.value == 0 || event.value == 1 ? ResultCode::Ok
                                                  : ResultCode::CorruptFile;
  }
  return ResultCode::CorruptFile;
}

enum class MechanismAdapterTarget : std::uint8_t {
  None,
  Intake,
  ScorerHeight,
  Cart,
  Trapdoor,
  Lever,
  Brooks,
  Sem,
  Arrow,
};

struct MechanismAdapterPlan {
  ResultCode result = ResultCode::CorruptFile;
  MechanismAdapterTarget target = MechanismAdapterTarget::None;
  std::int16_t value = 0;
};

constexpr MechanismAdapterPlan planMechanismAdapter(
    RobotIdentity robot, const MechanismEvent& event) {
  const ResultCode result = validateMechanism(robot, event);
  if (result != ResultCode::Ok) return {result, MechanismAdapterTarget::None, 0};

  MechanismAdapterTarget target = MechanismAdapterTarget::None;
  switch (event.kind) {
    case MechanismKind::IntakeMode:
      target = MechanismAdapterTarget::Intake;
      break;
    case MechanismKind::ScorerHeight:
      target = MechanismAdapterTarget::ScorerHeight;
      break;
    case MechanismKind::Cart:
      target = MechanismAdapterTarget::Cart;
      break;
    case MechanismKind::Trapdoor:
      target = MechanismAdapterTarget::Trapdoor;
      break;
    case MechanismKind::Lever:
      target = MechanismAdapterTarget::Lever;
      break;
    case MechanismKind::Brooks:
      target = MechanismAdapterTarget::Brooks;
      break;
    case MechanismKind::Sem:
      target = MechanismAdapterTarget::Sem;
      break;
    case MechanismKind::Arrow:
      target = MechanismAdapterTarget::Arrow;
      break;
  }
  return {ResultCode::Ok, target, event.value};
}

void captureDrive(int left, int right);
void captureMechanism(MechanismKind kind, std::int16_t value);
ResultCode applyDriverIntakeIntent(IntakeIntent intent);
ResultCode applyMechanism(const MechanismEvent& event);
void stopAllMechanisms();

}  // namespace aon::shadow
