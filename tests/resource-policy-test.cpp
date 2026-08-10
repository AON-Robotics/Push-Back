#include "aon/drivetrain/drivetrain.hpp"
#include "aon/communication/pi-protocol.hpp"
#include "aon/navigation/dynamic-obstacles.hpp"
#include "aon/navigation/path-planner.hpp"
#include "aon/tools/gui/gui-debug.hpp"

#include <type_traits>

static_assert(std::has_virtual_destructor_v<aon::Gui>);
static_assert(std::has_virtual_destructor_v<aon::Drivetrain>);
static_assert(!std::is_copy_constructible_v<aon::Gui>);
static_assert(!std::is_move_constructible_v<aon::Gui>);
static_assert(!std::is_copy_constructible_v<aon::Drivetrain>);
static_assert(!std::is_move_constructible_v<aon::Drivetrain>);
static_assert(sizeof(aon::navigation::PathPlanner) <= 16U * 1024U);
static_assert(sizeof(aon::navigation::PathPlanner) >
              sizeof(aon::navigation::PathPlannerConfig));
static_assert(sizeof(aon::communication::FrameParser) <= 512U);
static_assert(sizeof(aon::navigation::DynamicObstacleMap) <= 4U * 1024U);

int main() {}
