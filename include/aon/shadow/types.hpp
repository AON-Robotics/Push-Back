#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace aon::shadow {

constexpr std::uint32_t kSamplePeriodMs = 20;
constexpr std::uint32_t kMaximumDurationMs = 60000;
constexpr std::size_t kMaximumSamples = 3000;
constexpr std::size_t kMaximumEvents = 512;
constexpr std::size_t kSlotCount = 3;

enum class RobotIdentity : std::uint8_t { Small = 1, Big = 2 };
enum class Direction : std::int8_t { Reverse = -1, Stopped = 0, Forward = 1 };
enum class MechanismKind : std::uint8_t {
  IntakeMode, ScorerHeight, Cart, Trapdoor, Lever, Brooks, Sem, Arrow
};
enum class ResultCode : std::uint8_t {
  Ok, NotRecording, AlreadyRecording, SampleTooSoon, DuplicateEvent,
  CapacityReached, InvalidPose, PoseJump, EmptyRecording, NoSd, ReadOnly,
  OpenFailed, ReadFailed, WriteFailed, FlushFailed, CloseFailed, DeleteFailed,
  WriteCleanupFailed, FlushCleanupFailed, CloseCleanupFailed,
  CorruptFile, UnsupportedVersion, WrongRobot, InvalidSlot, PlayLocked,
  UnsafeState, Cancelled, OdometryFailure, MotionFailure, UnsupportedRobot
};

struct RawSample {
  std::uint32_t timeMs = 0;
  float x = 0, y = 0, heading = 0;
  std::int8_t leftX = 0, leftY = 0, rightX = 0, rightY = 0;
  std::int8_t leftCommand = 0, rightCommand = 0;
  Direction direction = Direction::Stopped;
  bool poseValid = false;
};

struct MechanismEvent {
  std::uint32_t timeMs = 0;
  MechanismKind kind = MechanismKind::IntakeMode;
  std::int16_t value = 0;
};

struct Capture {
  RobotIdentity robot = RobotIdentity::Small;
  std::array<RawSample, kMaximumSamples> samples{};
  std::array<MechanismEvent, kMaximumEvents> events{};
  std::size_t sampleCount = 0;
  std::size_t eventCount = 0;
  std::uint32_t durationMs = 0;
};

}  // namespace aon::shadow
