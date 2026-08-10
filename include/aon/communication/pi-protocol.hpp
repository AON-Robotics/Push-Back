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

enum class PayloadStatus : std::uint8_t {
  Success,
  WrongMessageType,
  InvalidLength,
  InvalidValue,
};

struct PoseSnapshotPayload {
  double xInches = 0.0;
  double yInches = 0.0;
  double headingRadians = 0.0;
  double positionVariance = 0.0;
  double headingVariance = 0.0;
  std::uint32_t estimateTimestampMs = 0;
};

enum class WallObservationAxis : std::uint8_t { X = 0, Y = 1 };

struct WallObservationPayload {
  WallObservationAxis axis = WallObservationAxis::X;
  double positionInches = 0.0;
  double variance = 0.0;
  std::uint16_t support = 0;
  std::uint32_t captureTimestampMs = 0;
};

inline constexpr std::size_t kMaximumObstaclesPerBatch = 4;
enum class ObstaclePayloadShape : std::uint8_t { Circle = 0, Rectangle = 1 };

struct ObstaclePayload {
  ObstaclePayloadShape shape = ObstaclePayloadShape::Circle;
  double xInches = 0.0;
  double yInches = 0.0;
  double radiusInches = 0.0;
  double halfWidthInches = 0.0;
  double halfHeightInches = 0.0;
  double headingRadians = 0.0;
  double confidence = 0.0;
};

struct ObstacleBatchPayload {
  std::array<ObstaclePayload, kMaximumObstaclesPerBatch> obstacles{};
  std::size_t count = 0;
};

inline constexpr std::size_t kMaximumRoutePointsPerChunk = 14;

struct RoutePointPayload {
  double xInches = 0.0;
  double yInches = 0.0;
};

struct RouteChunkPayload {
  std::uint32_t routeId = 0;
  std::uint8_t chunkIndex = 0;
  std::uint8_t chunkCount = 0;
  std::array<RoutePointPayload, kMaximumRoutePointsPerChunk> points{};
  std::size_t pointCount = 0;
};

[[nodiscard]] PayloadStatus encodePoseSnapshot(
    const PoseSnapshotPayload& payload, ProtocolMessage& message) noexcept;
[[nodiscard]] PayloadStatus decodePoseSnapshot(
    const ProtocolMessage& message, PoseSnapshotPayload& payload) noexcept;
[[nodiscard]] PayloadStatus encodeWallObservation(
    const WallObservationPayload& payload, ProtocolMessage& message) noexcept;
[[nodiscard]] PayloadStatus decodeWallObservation(
    const ProtocolMessage& message, WallObservationPayload& payload) noexcept;
[[nodiscard]] PayloadStatus encodeObstacleBatch(
    const ObstacleBatchPayload& payload, ProtocolMessage& message) noexcept;
[[nodiscard]] PayloadStatus decodeObstacleBatch(
    const ProtocolMessage& message, ObstacleBatchPayload& payload) noexcept;
[[nodiscard]] PayloadStatus encodeRouteChunk(
    const RouteChunkPayload& payload, ProtocolMessage& message) noexcept;
[[nodiscard]] PayloadStatus decodeRouteChunk(
    const ProtocolMessage& message, RouteChunkPayload& payload) noexcept;

}  // namespace aon::communication
