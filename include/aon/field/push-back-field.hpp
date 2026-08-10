#pragma once

#include "aon/field/field-map.hpp"

namespace aon::field {

/// Returns the verified 12-foot-square competition boundary.
///
/// Game-specific goals and restricted regions are deliberately omitted until
/// the active season and their official dimensions are selected explicitly.
[[nodiscard]] const FieldMap& pushBackField() noexcept;

}  // namespace aon::field
