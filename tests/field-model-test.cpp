#include "aon/field/field-map.hpp"
#include "aon/field/push-back-field.hpp"

#include <cmath>
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

void standardCompetitionBoundaryIsValidated() {
  using namespace aon::field;

  const FieldMap& field = pushBackField();
  CHECK(field.validate() == FieldMapIssue::None);
  CHECK(field.wallCount() == 4);
  CHECK(field.contains({0.0, 0.0}));
  CHECK(field.contains({71.0, -71.0}));
  CHECK(!field.contains({72.1, 0.0}));
  CHECK(!field.contains({0.0, -72.1}));
}

void clearanceAndWallDistanceProtectTheRobotFootprint() {
  using namespace aon::field;

  const FieldMap& field = pushBackField();
  CHECK(field.contains({60.0, 0.0}, 12.0));
  CHECK(!field.contains({60.1, 0.0}, 12.0));
  CHECK(std::abs(field.distanceToNearestWall({0.0, 0.0}) - 72.0) < 1e-9);
  CHECK(std::abs(field.distanceToNearestWall({70.0, 5.0}) - 2.0) < 1e-9);
  CHECK(field.segmentHasClearance({{0.0, 0.0}, {50.0, 50.0}}, 10.0));
  CHECK(!field.segmentHasClearance({{0.0, 0.0}, {70.0, 0.0}}, 3.0));
}

void malformedFieldGeometryIsRejected() {
  using namespace aon::field;

  std::array<Segment, FieldMap::kMaximumWalls> walls{};
  FieldMap inverted({1.0, -1.0, -1.0, 1.0}, walls, 0);
  CHECK(inverted.validate() == FieldMapIssue::InvertedBounds);

  const double invalid = std::numeric_limits<double>::quiet_NaN();
  FieldMap nonFinite({-1.0, invalid, -1.0, 1.0}, walls, 0);
  CHECK(nonFinite.validate() == FieldMapIssue::NonFiniteBounds);

  FieldMap tooMany({-1.0, 1.0, -1.0, 1.0}, walls,
                   FieldMap::kMaximumWalls + 1);
  CHECK(tooMany.validate() == FieldMapIssue::TooManyWalls);

  walls[0].start.xInches = invalid;
  FieldMap invalidWall({-1.0, 1.0, -1.0, 1.0}, walls, 1);
  CHECK(invalidWall.validate() == FieldMapIssue::InvalidWall);
}

}  // namespace

int main() {
  standardCompetitionBoundaryIsValidated();
  clearanceAndWallDistanceProtectTheRobotFootprint();
  malformedFieldGeometryIsRejected();
  std::cout << "field model tests passed\n";
  return 0;
}
