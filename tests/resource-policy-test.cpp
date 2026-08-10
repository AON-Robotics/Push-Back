#include "aon/drivetrain/drivetrain.hpp"
#include "aon/tools/gui/gui-debug.hpp"

#include <type_traits>

static_assert(std::has_virtual_destructor_v<aon::Gui>);
static_assert(std::has_virtual_destructor_v<aon::Drivetrain>);
static_assert(!std::is_copy_constructible_v<aon::Gui>);
static_assert(!std::is_move_constructible_v<aon::Gui>);
static_assert(!std::is_copy_constructible_v<aon::Drivetrain>);
static_assert(!std::is_move_constructible_v<aon::Drivetrain>);

int main() {}
