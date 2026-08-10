#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace aon::communication {

inline constexpr std::uint16_t kProtocolMagic = 0xA05A;
inline constexpr std::uint8_t kProtocolVersion = 1;
inline constexpr std::size_t kMaximumPayloadBytes = 128;
inline constexpr std::size_t kFrameHeaderBytes = 14;
inline constexpr std::size_t kFrameCrcBytes = 4;
inline constexpr std::size_t kMaximumFrameBytes =
    kFrameHeaderBytes + kMaximumPayloadBytes + kFrameCrcBytes;

enum class MessageType : std::uint8_t {
  Heartbeat = 1,
  PoseSnapshot = 2,
  LocalizationObservation = 3,
  ObstacleBatch = 4,
  RouteRequest = 5,
  RouteResponse = 6,
  RouteInvalidation = 7,
  Diagnostics = 8,
};

struct ProtocolMessage {
  MessageType type = MessageType::Heartbeat;
  std::uint32_t sequence = 0;
  std::uint32_t captureTimestampMs = 0;
  std::array<std::uint8_t, kMaximumPayloadBytes> payload{};
  std::size_t payloadSize = 0;
};

enum class EncodeStatus : std::uint8_t {
  Success,
  PayloadTooLarge,
  InvalidMessageType,
};

struct EncodedFrame {
  std::array<std::uint8_t, kMaximumFrameBytes> bytes{};
  std::size_t size = 0;
  EncodeStatus status = EncodeStatus::PayloadTooLarge;
};

enum class ParseResult : std::uint8_t {
  NeedMore,
  FrameReady,
  FramePending,
  CrcMismatch,
  Oversized,
  UnsupportedVersion,
  UnknownMessageType,
};

[[nodiscard]] std::uint32_t crc32(const std::uint8_t* data,
                                  std::size_t size) noexcept;
[[nodiscard]] EncodedFrame encodeFrame(
    const ProtocolMessage& message) noexcept;

class FrameParser {
 public:
  [[nodiscard]] ParseResult consume(std::uint8_t byte) noexcept;
  [[nodiscard]] bool take(ProtocolMessage& message) noexcept;
  void reset() noexcept;

 private:
  std::array<std::uint8_t, kMaximumFrameBytes> buffer_{};
  std::size_t size_ = 0;
  bool ready_ = false;
  ProtocolMessage message_{};
};

enum class SequenceResult : std::uint8_t {
  Accepted,
  Duplicate,
  OutOfOrder,
};

enum class LinkState : std::uint8_t { NeverSeen, Fresh, Stale };

class LinkHealthTracker {
 public:
  explicit LinkHealthTracker(std::uint32_t staleAfterMs) noexcept;

  [[nodiscard]] SequenceResult observe(std::uint32_t sequence,
                                       std::uint32_t nowMs) noexcept;
  [[nodiscard]] LinkState state(std::uint32_t nowMs) const noexcept;
  void reset() noexcept;

 private:
  std::uint32_t staleAfterMs_;
  bool seen_ = false;
  std::uint32_t lastSequence_ = 0;
  std::uint32_t lastAcceptedAtMs_ = 0;
};

}  // namespace aon::communication
