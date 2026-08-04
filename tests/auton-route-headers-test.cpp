#include "aon/auton/lemlib-routes.hpp"
#include "aon/auton/native-routes.hpp"
#include "aon/auton/native-tests.hpp"
#include "aon/auton/routines.hpp"

#include <type_traits>

int main() {
  using IntRoutine = int (*)();
  using VoidRoutine = void (*)();

  static_assert(
      std::is_same_v<decltype(&aon::routines::RedRoutine1), IntRoutine>);
  static_assert(
      std::is_same_v<decltype(&aon::routines::SkillsRoutine3), IntRoutine>);
  static_assert(std::is_same_v<
                decltype(&aon::routines::safeBigBotRoutine), VoidRoutine>);
  static_assert(std::is_same_v<
                decltype(&aon::routines::smallBotRoutine), VoidRoutine>);
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunLemLibForwardValidation),
                IntRoutine>);
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunShadowPlayback), IntRoutine>);
  static_assert(std::is_same_v<
                decltype(&aon::routines::RunNativeTurnTest), IntRoutine>);
}
