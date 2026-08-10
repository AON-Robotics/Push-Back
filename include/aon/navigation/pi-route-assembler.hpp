#pragma once

#include <cstdint>

#include "aon/communication/pi-protocol.hpp"
#include "aon/navigation/path-planner.hpp"

namespace aon::navigation {

enum class RouteAssemblyResult : std::uint8_t {
  Started,
  Appended,
  Complete,
  Invalid,
  MissingStart,
  RouteMismatch,
  MetadataMismatch,
  DuplicateOrOutOfOrder,
  CapacityExceeded,
};

class PiRouteAssembler {
 public:
  [[nodiscard]] RouteAssemblyResult ingest(
      const communication::RouteChunkPayload& chunk) noexcept;
  [[nodiscard]] bool take(Path& route) noexcept;
  void reset() noexcept;

 private:
  Path building_{};
  Path readyRoute_{};
  std::uint32_t routeId_ = 0;
  std::uint8_t chunkCount_ = 0;
  std::uint8_t nextChunk_ = 0;
  bool active_ = false;
  bool ready_ = false;
};

}  // namespace aon::navigation
