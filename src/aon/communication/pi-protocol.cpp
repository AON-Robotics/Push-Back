#include "aon/communication/pi-protocol.hpp"

namespace aon::communication {
namespace {

constexpr std::uint8_t lowByte(std::uint16_t value) noexcept {
  return static_cast<std::uint8_t>(value & 0xffU);
}

constexpr std::uint8_t highByte(std::uint16_t value) noexcept {
  return static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write16(std::uint8_t* output, std::uint16_t value) noexcept {
  output[0] = static_cast<std::uint8_t>(value & 0xffU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write32(std::uint8_t* output, std::uint32_t value) noexcept {
  for (std::size_t index = 0; index < 4; ++index) {
    output[index] =
        static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
  }
}

std::uint16_t read16(const std::uint8_t* input) noexcept {
  return static_cast<std::uint16_t>(input[0]) |
         static_cast<std::uint16_t>(input[1]) << 8U;
}

std::uint32_t read32(const std::uint8_t* input) noexcept {
  std::uint32_t value = 0;
  for (std::size_t index = 0; index < 4; ++index) {
    value |= static_cast<std::uint32_t>(input[index]) << (index * 8U);
  }
  return value;
}

bool validMessageType(std::uint8_t value) noexcept {
  switch (static_cast<MessageType>(value)) {
    case MessageType::Heartbeat:
    case MessageType::PoseSnapshot:
    case MessageType::LocalizationObservation:
    case MessageType::ObstacleBatch:
    case MessageType::RouteRequest:
    case MessageType::RouteResponse:
    case MessageType::RouteInvalidation:
    case MessageType::Diagnostics:
      return true;
  }
  return false;
}

}  // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) noexcept {
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask =
          0U - static_cast<std::uint32_t>(crc & 1U);
      crc = (crc >> 1U) ^ (0xedb88320U & mask);
    }
  }
  return ~crc;
}

EncodedFrame encodeFrame(const ProtocolMessage& message) noexcept {
  EncodedFrame frame;
  if (message.payloadSize > message.payload.size()) return frame;
  if (!validMessageType(static_cast<std::uint8_t>(message.type))) {
    frame.status = EncodeStatus::InvalidMessageType;
    return frame;
  }

  frame.bytes[0] = lowByte(kProtocolMagic);
  frame.bytes[1] = highByte(kProtocolMagic);
  frame.bytes[2] = kProtocolVersion;
  frame.bytes[3] = static_cast<std::uint8_t>(message.type);
  write16(&frame.bytes[4], static_cast<std::uint16_t>(message.payloadSize));
  write32(&frame.bytes[6], message.sequence);
  write32(&frame.bytes[10], message.captureTimestampMs);
  for (std::size_t index = 0; index < message.payloadSize; ++index) {
    frame.bytes[kFrameHeaderBytes + index] = message.payload[index];
  }
  const std::size_t crcIndex = kFrameHeaderBytes + message.payloadSize;
  write32(&frame.bytes[crcIndex],
          crc32(&frame.bytes[2], crcIndex - 2));
  frame.size = crcIndex + kFrameCrcBytes;
  frame.status = EncodeStatus::Success;
  return frame;
}

ParseResult FrameParser::consume(std::uint8_t byte) noexcept {
  if (ready_) return ParseResult::FramePending;

  if (size_ == 0) {
    if (byte != lowByte(kProtocolMagic)) return ParseResult::NeedMore;
    buffer_[size_++] = byte;
    return ParseResult::NeedMore;
  }
  if (size_ == 1) {
    if (byte == highByte(kProtocolMagic)) {
      buffer_[size_++] = byte;
    } else if (byte == lowByte(kProtocolMagic)) {
      buffer_[0] = byte;
    } else {
      size_ = 0;
    }
    return ParseResult::NeedMore;
  }
  if (size_ >= buffer_.size()) {
    reset();
    return ParseResult::Oversized;
  }
  buffer_[size_++] = byte;
  if (size_ == 3 && buffer_[2] != kProtocolVersion) {
    reset();
    return ParseResult::UnsupportedVersion;
  }
  if (size_ < kFrameHeaderBytes) return ParseResult::NeedMore;

  const std::size_t payloadSize = read16(&buffer_[4]);
  if (payloadSize > kMaximumPayloadBytes) {
    reset();
    return ParseResult::Oversized;
  }
  const std::size_t expectedSize =
      kFrameHeaderBytes + payloadSize + kFrameCrcBytes;
  if (size_ < expectedSize) return ParseResult::NeedMore;
  if (size_ > expectedSize) {
    reset();
    return ParseResult::Oversized;
  }
  const std::size_t crcIndex = kFrameHeaderBytes + payloadSize;
  const std::uint32_t expectedCrc = read32(&buffer_[crcIndex]);
  const std::uint32_t actualCrc = crc32(&buffer_[2], crcIndex - 2);
  if (expectedCrc != actualCrc) {
    reset();
    return ParseResult::CrcMismatch;
  }
  if (!validMessageType(buffer_[3])) {
    reset();
    return ParseResult::UnknownMessageType;
  }

  message_ = {};
  message_.type = static_cast<MessageType>(buffer_[3]);
  message_.payloadSize = payloadSize;
  message_.sequence = read32(&buffer_[6]);
  message_.captureTimestampMs = read32(&buffer_[10]);
  for (std::size_t index = 0; index < payloadSize; ++index) {
    message_.payload[index] = buffer_[kFrameHeaderBytes + index];
  }
  ready_ = true;
  return ParseResult::FrameReady;
}

bool FrameParser::take(ProtocolMessage& message) noexcept {
  if (!ready_) return false;
  message = message_;
  size_ = 0;
  ready_ = false;
  message_ = {};
  return true;
}

void FrameParser::reset() noexcept {
  size_ = 0;
  ready_ = false;
  message_ = {};
}

LinkHealthTracker::LinkHealthTracker(std::uint32_t staleAfterMs) noexcept
    : staleAfterMs_(staleAfterMs) {}

SequenceResult LinkHealthTracker::observe(std::uint32_t sequence,
                                          std::uint32_t nowMs) noexcept {
  if (seen_) {
    if (sequence == lastSequence_) return SequenceResult::Duplicate;
    if (static_cast<std::int32_t>(sequence - lastSequence_) <= 0) {
      return SequenceResult::OutOfOrder;
    }
  }
  seen_ = true;
  lastSequence_ = sequence;
  lastAcceptedAtMs_ = nowMs;
  return SequenceResult::Accepted;
}

LinkState LinkHealthTracker::state(std::uint32_t nowMs) const noexcept {
  if (!seen_) return LinkState::NeverSeen;
  return nowMs - lastAcceptedAtMs_ <= staleAfterMs_ ? LinkState::Fresh
                                                    : LinkState::Stale;
}

void LinkHealthTracker::reset() noexcept {
  seen_ = false;
  lastSequence_ = 0;
  lastAcceptedAtMs_ = 0;
}

}  // namespace aon::communication
