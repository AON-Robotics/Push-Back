#pragma once

#include "aon/shadow/types.hpp"

#include <algorithm>
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

void captureDrive(int left, int right);
void captureMechanism(MechanismKind kind, std::int16_t value);
void applyDriverIntakeIntent(IntakeIntent intent);
ResultCode applyMechanism(const MechanismEvent& event);

}  // namespace aon::shadow
