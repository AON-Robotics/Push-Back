#include <cstdlib>
#include <iostream>

#include "aon/navigation/pi-route-assembler.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

aon::communication::RouteChunkPayload chunk(std::uint32_t routeId,
                                             std::uint8_t index,
                                             std::uint8_t count,
                                             std::size_t points) {
  aon::communication::RouteChunkPayload result;
  result.routeId = routeId;
  result.chunkIndex = index;
  result.chunkCount = count;
  result.pointCount = points;
  for (std::size_t point = 0; point < points; ++point) {
    result.points[point] = {static_cast<double>(index * 10 + point),
                            static_cast<double>(point)};
  }
  return result;
}

void completeOrderedRouteBecomesVisibleAtomically() {
  using namespace aon::navigation;
  PiRouteAssembler assembler;
  Path route;
  CHECK(assembler.ingest(chunk(42, 0, 2, 2)) == RouteAssemblyResult::Started);
  CHECK(!assembler.take(route));
  CHECK(assembler.ingest(chunk(42, 1, 2, 2)) == RouteAssemblyResult::Complete);
  CHECK(assembler.take(route));
  CHECK(route.size == 4);
  CHECK(route.points[0].xInches == 0.0);
  CHECK(route.points[2].xInches == 10.0);
  CHECK(!assembler.take(route));
}

void partialMixedAndDuplicateRoutesFailClosed() {
  using namespace aon::navigation;
  PiRouteAssembler assembler;
  CHECK(assembler.ingest(chunk(10, 1, 2, 1)) ==
        RouteAssemblyResult::MissingStart);
  CHECK(assembler.ingest(chunk(10, 0, 3, 1)) == RouteAssemblyResult::Started);
  CHECK(assembler.ingest(chunk(11, 1, 3, 1)) ==
        RouteAssemblyResult::RouteMismatch);
  CHECK(assembler.ingest(chunk(10, 0, 3, 1)) ==
        RouteAssemblyResult::DuplicateOrOutOfOrder);
  CHECK(assembler.ingest(chunk(10, 1, 2, 1)) ==
        RouteAssemblyResult::MetadataMismatch);
}

void overflowingRoutesAreDiscarded() {
  using namespace aon::navigation;
  PiRouteAssembler assembler;
  CHECK(assembler.ingest(chunk(7, 0, 3, 14)) == RouteAssemblyResult::Started);
  CHECK(assembler.ingest(chunk(7, 1, 3, 14)) ==
        RouteAssemblyResult::Appended);
  CHECK(assembler.ingest(chunk(7, 2, 3, 14)) ==
        RouteAssemblyResult::CapacityExceeded);
  Path route;
  CHECK(!assembler.take(route));
  CHECK(assembler.ingest(chunk(8, 0, 1, 2)) == RouteAssemblyResult::Complete);
  CHECK(assembler.take(route));
  CHECK(route.size == 2);
}

}  // namespace

int main() {
  completeOrderedRouteBecomesVisibleAtomically();
  partialMixedAndDuplicateRoutesFailClosed();
  overflowingRoutesAreDiscarded();
  std::cout << "Pi route assembler tests passed\n";
  return 0;
}
