#include "aon/navigation/pi-route-assembler.hpp"

#include <cmath>

namespace aon::navigation {
namespace {

bool validChunk(const communication::RouteChunkPayload& chunk) noexcept {
  if (chunk.routeId == 0 || chunk.chunkCount == 0 ||
      chunk.chunkIndex >= chunk.chunkCount || chunk.pointCount == 0 ||
      chunk.pointCount > communication::kMaximumRoutePointsPerChunk) {
    return false;
  }
  for (std::size_t index = 0; index < chunk.pointCount; ++index) {
    if (!std::isfinite(chunk.points[index].xInches) ||
        !std::isfinite(chunk.points[index].yInches)) {
      return false;
    }
  }
  return true;
}

}  // namespace

RouteAssemblyResult PiRouteAssembler::ingest(
    const communication::RouteChunkPayload& chunk) noexcept {
  if (!validChunk(chunk)) return RouteAssemblyResult::Invalid;

  if (!active_) {
    if (chunk.chunkIndex != 0) return RouteAssemblyResult::MissingStart;
    building_ = {};
    ready_ = false;
    routeId_ = chunk.routeId;
    chunkCount_ = chunk.chunkCount;
    nextChunk_ = 0;
    active_ = true;
  } else {
    if (chunk.routeId != routeId_) return RouteAssemblyResult::RouteMismatch;
    if (chunk.chunkCount != chunkCount_) {
      return RouteAssemblyResult::MetadataMismatch;
    }
    if (chunk.chunkIndex != nextChunk_) {
      return RouteAssemblyResult::DuplicateOrOutOfOrder;
    }
  }

  if (building_.size + chunk.pointCount > Path::kMaximumPoints) {
    reset();
    return RouteAssemblyResult::CapacityExceeded;
  }
  for (std::size_t index = 0; index < chunk.pointCount; ++index) {
    building_.points[building_.size++] =
        {chunk.points[index].xInches, chunk.points[index].yInches};
  }
  ++nextChunk_;
  if (nextChunk_ == chunkCount_) {
    readyRoute_ = building_;
    ready_ = true;
    active_ = false;
    return RouteAssemblyResult::Complete;
  }
  return chunk.chunkIndex == 0 ? RouteAssemblyResult::Started
                               : RouteAssemblyResult::Appended;
}

bool PiRouteAssembler::take(Path& route) noexcept {
  if (!ready_) return false;
  route = readyRoute_;
  ready_ = false;
  readyRoute_ = {};
  return true;
}

void PiRouteAssembler::reset() noexcept {
  building_ = {};
  readyRoute_ = {};
  routeId_ = 0;
  chunkCount_ = 0;
  nextChunk_ = 0;
  active_ = false;
  ready_ = false;
}

}  // namespace aon::navigation
