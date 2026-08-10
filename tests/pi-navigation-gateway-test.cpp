#include <cstdlib>
#include <iostream>

#include "aon/communication/pi-navigation-gateway.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

aon::communication::GatewayResult deliver(
    aon::communication::PiNavigationGateway& gateway,
    const aon::communication::ProtocolMessage& message,
    std::uint32_t nowMs) {
  using namespace aon::communication;
  const EncodedFrame frame = encodeFrame(message);
  CHECK(frame.status == EncodeStatus::Success);
  GatewayResult result = GatewayResult::NeedMore;
  for (std::size_t index = 0; index < frame.size; ++index) {
    result = gateway.consume(frame.bytes[index], nowMs);
  }
  return result;
}

void validTypedMessagesReachOnlyBoundedHandoffs() {
  using namespace aon::communication;
  PiNavigationGateway gateway(250, {6.0, 500, 0.5});

  ProtocolMessage wallMessage;
  CHECK(encodeWallObservation({WallObservationAxis::Y, 20.0, 0.5, 8, 100},
                              wallMessage) == PayloadStatus::Success);
  wallMessage.sequence = 1;
  CHECK(deliver(gateway, wallMessage, 110) == GatewayResult::WallReady);
  WallObservationPayload wall;
  CHECK(gateway.takeWallObservation(wall));
  CHECK(wall.captureTimestampMs == 100);
  CHECK(!gateway.takeWallObservation(wall));

  ObstacleBatchPayload batch;
  batch.count = 1;
  batch.obstacles[0] = {ObstaclePayloadShape::Circle, 5.0, 6.0, 3.0,
                        0.0, 0.0, 0.0, 0.9};
  ProtocolMessage obstacleMessage;
  CHECK(encodeObstacleBatch(batch, obstacleMessage) == PayloadStatus::Success);
  obstacleMessage.sequence = 2;
  obstacleMessage.captureTimestampMs = 120;
  CHECK(deliver(gateway, obstacleMessage, 125) ==
        GatewayResult::ObstaclesUpdated);
  CHECK(gateway.obstacles().size() == 1);

  RouteChunkPayload route;
  route.routeId = 9;
  route.chunkCount = 1;
  route.pointCount = 2;
  route.points[0] = {0.0, 0.0};
  route.points[1] = {10.0, 10.0};
  ProtocolMessage routeMessage;
  CHECK(encodeRouteChunk(route, routeMessage) == PayloadStatus::Success);
  routeMessage.sequence = 3;
  CHECK(deliver(gateway, routeMessage, 130) == GatewayResult::RouteReady);
  aon::navigation::Path path;
  CHECK(gateway.takeRoute(path));
  CHECK(path.size == 2);
}

void replayMalformedPayloadAndStaleLinkFailClosed() {
  using namespace aon::communication;
  PiNavigationGateway gateway(50, {6.0, 500, 0.5});
  ProtocolMessage heartbeat;
  heartbeat.type = MessageType::Heartbeat;
  heartbeat.sequence = 10;
  CHECK(deliver(gateway, heartbeat, 100) == GatewayResult::MessageAccepted);
  CHECK(gateway.linkState(150) == LinkState::Fresh);
  CHECK(gateway.linkState(151) == LinkState::Stale);
  CHECK(deliver(gateway, heartbeat, 160) == GatewayResult::Duplicate);
  CHECK(gateway.linkState(160) == LinkState::Stale);

  ProtocolMessage malformed;
  malformed.type = MessageType::ObstacleBatch;
  malformed.sequence = 11;
  malformed.payloadSize = 1;
  malformed.payload[0] = 1;
  CHECK(deliver(gateway, malformed, 170) == GatewayResult::InvalidPayload);
  CHECK(gateway.obstacles().size() == 0);
}

}  // namespace

int main() {
  validTypedMessagesReachOnlyBoundedHandoffs();
  replayMalformedPayloadAndStaleLinkFailClosed();
  std::cout << "Pi navigation gateway tests passed\n";
  return 0;
}
