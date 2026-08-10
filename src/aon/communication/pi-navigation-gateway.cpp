#include "aon/communication/pi-navigation-gateway.hpp"

namespace aon::communication {

PiNavigationGateway::PiNavigationGateway(
    std::uint32_t staleAfterMs,
    navigation::DynamicObstacleConfig obstacleConfig) noexcept
    : link_(staleAfterMs), obstacles_(obstacleConfig) {}

GatewayResult PiNavigationGateway::consume(std::uint8_t byte,
                                           std::uint32_t nowMs) noexcept {
  const ParseResult parseResult = parser_.consume(byte);
  if (parseResult == ParseResult::NeedMore ||
      parseResult == ParseResult::FramePending) {
    return GatewayResult::NeedMore;
  }
  if (parseResult != ParseResult::FrameReady) return GatewayResult::ParseError;

  ProtocolMessage message;
  if (!parser_.take(message)) return GatewayResult::ParseError;

  WallObservationPayload wall;
  ObstacleBatchPayload batch;
  RouteChunkPayload chunk;
  switch (message.type) {
    case MessageType::Heartbeat:
      if (message.payloadSize != 0) return GatewayResult::InvalidPayload;
      break;
    case MessageType::LocalizationObservation:
      if (decodeWallObservation(message, wall) != PayloadStatus::Success) {
        return GatewayResult::InvalidPayload;
      }
      break;
    case MessageType::ObstacleBatch:
      if (decodeObstacleBatch(message, batch) != PayloadStatus::Success) {
        return GatewayResult::InvalidPayload;
      }
      break;
    case MessageType::RouteResponse:
      if (decodeRouteChunk(message, chunk) != PayloadStatus::Success) {
        return GatewayResult::InvalidPayload;
      }
      break;
    case MessageType::PoseSnapshot:
    case MessageType::RouteRequest:
    case MessageType::RouteInvalidation:
    case MessageType::Diagnostics:
      return GatewayResult::UnsupportedMessage;
  }

  const SequenceResult sequence = link_.observe(message.sequence, nowMs);
  if (sequence == SequenceResult::Duplicate) return GatewayResult::Duplicate;
  if (sequence == SequenceResult::OutOfOrder) {
    return GatewayResult::OutOfOrder;
  }

  if (message.type == MessageType::Heartbeat) {
    return GatewayResult::MessageAccepted;
  }
  if (message.type == MessageType::LocalizationObservation) {
    pendingWall_ = wall;
    wallReady_ = true;
    return GatewayResult::WallReady;
  }
  if (message.type == MessageType::ObstacleBatch) {
    navigation::DynamicObstacleMap candidate = obstacles_;
    for (std::size_t index = 0; index < batch.count; ++index) {
      const ObstaclePayload& input = batch.obstacles[index];
      navigation::ObstacleDetection detection;
      detection.shape = input.shape == ObstaclePayloadShape::Circle
                            ? navigation::ObstacleShape::Circle
                            : navigation::ObstacleShape::Rectangle;
      detection.center = {input.xInches, input.yInches};
      detection.radiusInches = input.radiusInches;
      detection.halfWidthInches = input.halfWidthInches;
      detection.halfHeightInches = input.halfHeightInches;
      detection.headingRadians = input.headingRadians;
      detection.confidence = input.confidence;
      detection.timestampMs = message.captureTimestampMs;
      const navigation::ObstacleUpdateResult update = candidate.update(detection);
      if (update == navigation::ObstacleUpdateResult::Invalid ||
          update == navigation::ObstacleUpdateResult::OutOfOrder ||
          update == navigation::ObstacleUpdateResult::CapacityRejected) {
        return GatewayResult::InvalidPayload;
      }
    }
    obstacles_ = candidate;
    return GatewayResult::ObstaclesUpdated;
  }

  const navigation::RouteAssemblyResult assembly = routeAssembler_.ingest(chunk);
  if (assembly == navigation::RouteAssemblyResult::Complete) {
    return GatewayResult::RouteReady;
  }
  if (assembly == navigation::RouteAssemblyResult::Started ||
      assembly == navigation::RouteAssemblyResult::Appended) {
    return GatewayResult::RoutePartial;
  }
  return GatewayResult::InvalidPayload;
}

bool PiNavigationGateway::takeWallObservation(
    WallObservationPayload& observation) noexcept {
  if (!wallReady_) return false;
  observation = pendingWall_;
  pendingWall_ = {};
  wallReady_ = false;
  return true;
}

bool PiNavigationGateway::takeRoute(navigation::Path& route) noexcept {
  return routeAssembler_.take(route);
}

const navigation::DynamicObstacleMap& PiNavigationGateway::obstacles()
    const noexcept {
  return obstacles_;
}

void PiNavigationGateway::expireObstacles(std::uint32_t nowMs) noexcept {
  obstacles_.expire(nowMs);
}

LinkState PiNavigationGateway::linkState(std::uint32_t nowMs) const noexcept {
  return link_.state(nowMs);
}

void PiNavigationGateway::reset() noexcept {
  parser_.reset();
  link_.reset();
  obstacles_.clear();
  routeAssembler_.reset();
  pendingWall_ = {};
  wallReady_ = false;
}

}  // namespace aon::communication
