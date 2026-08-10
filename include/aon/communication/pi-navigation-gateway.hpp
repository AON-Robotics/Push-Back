#pragma once

#include <cstdint>

#include "aon/communication/pi-protocol.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/navigation/pi-route-assembler.hpp"

namespace aon::communication {

enum class GatewayResult : std::uint8_t {
  NeedMore,
  MessageAccepted,
  WallReady,
  ObstaclesUpdated,
  RoutePartial,
  RouteReady,
  Duplicate,
  OutOfOrder,
  InvalidPayload,
  UnsupportedMessage,
  ParseError,
};

class PiNavigationGateway {
 public:
  PiNavigationGateway(
      std::uint32_t staleAfterMs,
      navigation::DynamicObstacleConfig obstacleConfig) noexcept;

  [[nodiscard]] GatewayResult consume(std::uint8_t byte,
                                      std::uint32_t nowMs) noexcept;
  [[nodiscard]] bool takeWallObservation(
      WallObservationPayload& observation) noexcept;
  [[nodiscard]] bool takeRoute(navigation::Path& route) noexcept;
  [[nodiscard]] const navigation::DynamicObstacleMap& obstacles()
      const noexcept;
  void expireObstacles(std::uint32_t nowMs) noexcept;
  [[nodiscard]] LinkState linkState(std::uint32_t nowMs) const noexcept;
  void reset() noexcept;

 private:
  FrameParser parser_;
  LinkHealthTracker link_;
  navigation::DynamicObstacleMap obstacles_;
  navigation::PiRouteAssembler routeAssembler_;
  WallObservationPayload pendingWall_{};
  bool wallReady_ = false;
};

}  // namespace aon::communication
