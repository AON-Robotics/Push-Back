#include "aon/communication/pi-protocol.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

void heartbeatRoundTripsThroughFragmentedInput() {
  using namespace aon::communication;

  ProtocolMessage heartbeat;
  heartbeat.type = MessageType::Heartbeat;
  heartbeat.sequence = 7;
  heartbeat.captureTimestampMs = 100;
  const EncodedFrame frame = encodeFrame(heartbeat);
  CHECK(frame.status == EncodeStatus::Success);

  FrameParser parser;
  ParseResult result = ParseResult::NeedMore;
  for (std::size_t index = 0; index < frame.size; ++index) {
    result = parser.consume(frame.bytes[index]);
  }
  CHECK(result == ParseResult::FrameReady);
  ProtocolMessage decoded;
  CHECK(parser.take(decoded));
  CHECK(decoded.type == MessageType::Heartbeat);
  CHECK(decoded.sequence == 7);
  CHECK(decoded.captureTimestampMs == 100);
  CHECK(decoded.payloadSize == 0);
}

void corruptionAndUnknownTypesFailClosedThenParserResynchronizes() {
  using namespace aon::communication;

  ProtocolMessage invalid;
  invalid.type = static_cast<MessageType>(0xff);
  CHECK(encodeFrame(invalid).status == EncodeStatus::InvalidMessageType);

  ProtocolMessage heartbeat;
  heartbeat.type = MessageType::Heartbeat;
  heartbeat.sequence = 9;
  EncodedFrame corrupt = encodeFrame(heartbeat);
  corrupt.bytes[6] ^= 0x40U;
  FrameParser parser;
  ParseResult result = ParseResult::NeedMore;
  for (std::size_t index = 0; index < corrupt.size; ++index) {
    result = parser.consume(corrupt.bytes[index]);
  }
  CHECK(result == ParseResult::CrcMismatch);

  EncodedFrame unknown = encodeFrame(heartbeat);
  unknown.bytes[3] = 0xfe;
  const std::size_t crcIndex = unknown.size - kFrameCrcBytes;
  const std::uint32_t checksum = crc32(&unknown.bytes[2], crcIndex - 2);
  for (std::size_t index = 0; index < 4; ++index) {
    unknown.bytes[crcIndex + index] = static_cast<std::uint8_t>(
        (checksum >> (index * 8U)) & 0xffU);
  }
  for (std::size_t index = 0; index < unknown.size; ++index) {
    result = parser.consume(unknown.bytes[index]);
  }
  CHECK(result == ParseResult::UnknownMessageType);

  CHECK(parser.consume(0x00) == ParseResult::NeedMore);
  CHECK(parser.consume(0x7f) == ParseResult::NeedMore);
  const EncodedFrame valid = encodeFrame(heartbeat);
  for (std::size_t index = 0; index < valid.size; ++index) {
    result = parser.consume(valid.bytes[index]);
  }
  CHECK(result == ParseResult::FrameReady);
  ProtocolMessage decoded;
  CHECK(parser.take(decoded));
  CHECK(decoded.sequence == 9);
}

void linkHealthRequiresFreshForwardSequenceProgress() {
  using namespace aon::communication;

  LinkHealthTracker link(250);
  CHECK(link.state(0) == LinkState::NeverSeen);
  CHECK(link.observe(10, 100) == SequenceResult::Accepted);
  CHECK(link.state(350) == LinkState::Fresh);
  CHECK(link.state(351) == LinkState::Stale);
  CHECK(link.observe(10, 360) == SequenceResult::Duplicate);
  CHECK(link.observe(9, 370) == SequenceResult::OutOfOrder);
  CHECK(link.state(370) == LinkState::Stale);
  CHECK(link.observe(11, 400) == SequenceResult::Accepted);
  CHECK(link.state(400) == LinkState::Fresh);
}

void typedPoseAndWallPayloadsRoundTripAndRejectNonFiniteValues() {
  using namespace aon::communication;

  ProtocolMessage poseMessage;
  const PoseSnapshotPayload pose{12.5, -8.25, 1.5, 0.4, 0.02, 1234};
  CHECK(encodePoseSnapshot(pose, poseMessage) == PayloadStatus::Success);
  CHECK(poseMessage.type == MessageType::PoseSnapshot);
  PoseSnapshotPayload decodedPose;
  CHECK(decodePoseSnapshot(poseMessage, decodedPose) ==
        PayloadStatus::Success);
  CHECK(std::abs(decodedPose.xInches - 12.5) < 1e-6);
  CHECK(std::abs(decodedPose.yInches + 8.25) < 1e-6);
  CHECK(decodedPose.estimateTimestampMs == 1234);

  ProtocolMessage wallMessage;
  const WallObservationPayload wall{WallObservationAxis::Y, 24.0, 0.5, 8,
                                    1300};
  CHECK(encodeWallObservation(wall, wallMessage) == PayloadStatus::Success);
  WallObservationPayload decodedWall;
  CHECK(decodeWallObservation(wallMessage, decodedWall) ==
        PayloadStatus::Success);
  CHECK(decodedWall.axis == WallObservationAxis::Y);
  CHECK(std::abs(decodedWall.positionInches - 24.0) < 1e-6);
  CHECK(decodedWall.support == 8);

  PoseSnapshotPayload invalid = pose;
  invalid.xInches = std::numeric_limits<double>::quiet_NaN();
  CHECK(encodePoseSnapshot(invalid, poseMessage) ==
        PayloadStatus::InvalidValue);
}

}  // namespace

int main() {
  heartbeatRoundTripsThroughFragmentedInput();
  corruptionAndUnknownTypesFailClosedThenParserResynchronizes();
  linkHealthRequiresFreshForwardSequenceProgress();
  typedPoseAndWallPayloadsRoundTripAndRejectNonFiniteValues();
  std::cout << "Pi protocol tests passed\n";
  return 0;
}
