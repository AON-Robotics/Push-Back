#include "aon/field/push-back-field.hpp"

#include <array>

namespace aon::field {

const FieldMap& pushBackField() noexcept {
  // Push Back Game Manual 4.0, Appendix A: the foam playing surface is
  // 12 ft x 12 ft. Existing JerryIO field coordinates use its center as the
  // origin, so the verified boundary is +/-72 inches on each axis.
  constexpr double halfFieldInches = 72.0;
  static const std::array<Segment, FieldMap::kMaximumWalls> walls{{
      {{-halfFieldInches, -halfFieldInches},
       {halfFieldInches, -halfFieldInches}},
      {{halfFieldInches, -halfFieldInches},
       {halfFieldInches, halfFieldInches}},
      {{halfFieldInches, halfFieldInches},
       {-halfFieldInches, halfFieldInches}},
      {{-halfFieldInches, halfFieldInches},
       {-halfFieldInches, -halfFieldInches}},
  }};
  static const FieldMap field(
      {-halfFieldInches, halfFieldInches, -halfFieldInches,
       halfFieldInches},
      walls, 4);
  return field;
}

}  // namespace aon::field
