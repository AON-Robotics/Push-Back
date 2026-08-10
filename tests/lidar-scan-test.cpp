#include "aon/field/push-back-field.hpp"
#include "aon/lidar/scan-processor.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

void knownWallScanProducesAnAbsoluteAxisCorrection() {
  using namespace aon::lidar;

  LidarScan scan;
  scan.captureTimestampMs = 100;
  for (double angle : {-0.2, -0.1, 0.0, 0.1, 0.2}) {
    scan.samples[scan.sampleCount++] =
        {angle, 72.0 / std::cos(angle), true};
  }

  ScanProcessor processor({2.0, 120.0, 3.0, 4, 3.0, 3, 0.25});
  const ScanResult result = processor.process(
      scan, {{0.0, 0.0, 0.0}, {0.0, 2.0, 0.0}},
      aon::field::pushBackField());
  CHECK(result.status == ScanStatus::Accepted);
  CHECK(result.wallObservationCount == 1);
  CHECK(result.wallObservations[0].axis == WallAxis::Y);
  CHECK(std::abs(result.wallObservations[0].positionInches) < 1e-6);
  CHECK(result.wallObservations[0].support == 5);
  CHECK(result.wallObservations[0].variance > 0.0);
  CHECK(result.obstacleCount == 0);
}

void unmatchedClustersBecomeObstaclesButOutOfFieldReturnsAreRejected() {
  using namespace aon::lidar;

  ScanProcessor processor({2.0, 120.0, 3.0, 4, 3.0, 3, 0.25});
  LidarScan objectScan;
  objectScan.captureTimestampMs = 200;
  for (double angle : {-0.05, 0.0, 0.05}) {
    objectScan.samples[objectScan.sampleCount++] = {angle, 20.0, true};
  }
  const ScanResult object = processor.process(
      objectScan, {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      aon::field::pushBackField());
  CHECK(object.status == ScanStatus::Accepted);
  CHECK(object.obstacleCount == 1);
  CHECK(std::abs(object.obstacles[0].center.xInches) < 1e-9);
  CHECK(object.obstacles[0].center.yInches > 19.9);

  LidarScan outsideScan;
  outsideScan.captureTimestampMs = 220;
  for (double angle : {-0.01, 0.0, 0.01}) {
    outsideScan.samples[outsideScan.sampleCount++] = {angle, 100.0, true};
  }
  const ScanResult outside = processor.process(
      outsideScan, {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      aon::field::pushBackField());
  CHECK(outside.obstacleCount == 0);
  CHECK(outside.rejectedSampleCount == 3);
}

void wallMatchingDoesNotExtendPerimeterSegmentsToInfinity() {
  using namespace aon::lidar;

  LidarScan scan;
  scan.captureTimestampMs = 300;
  for (double y : {80.0, 81.0, 82.0, 83.0}) {
    scan.samples[scan.sampleCount++] =
        {std::atan2(72.0, y), std::hypot(72.0, y), true};
  }
  ScanProcessor processor({2.0, 120.0, 3.0, 4, 3.0, 3, 0.25});
  const ScanResult result = processor.process(
      scan, {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
      aon::field::pushBackField());
  CHECK(result.wallObservationCount == 0);
  CHECK(result.obstacleCount == 0);
  CHECK(result.rejectedSampleCount == 4);
}

}  // namespace

int main() {
  knownWallScanProducesAnAbsoluteAxisCorrection();
  unmatchedClustersBecomeObstaclesButOutOfFieldReturnsAreRejected();
  wallMatchingDoesNotExtendPerimeterSegmentsToInfinity();
  std::cout << "LiDAR scan tests passed\n";
  return 0;
}
