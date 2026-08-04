#include <cstdint>

namespace aon::core {
class Hardware;
}

#include "aon/auton/mechanism-actions.hpp"

#include <type_traits>

int main() {
  using Hardware = aon::core::Hardware;
  using Action = void (*)(Hardware&);
  using TimedAction = void (*)(Hardware&, std::uint32_t);

  static_assert(std::is_same_v<
                decltype(static_cast<Action>(
                    &aon::auton::mechanisms::beginLoaderCollection)),
                Action>);
  static_assert(std::is_same_v<
                decltype(static_cast<Action>(
                    &aon::auton::mechanisms::finishLoaderCollection)),
                Action>);
  static_assert(std::is_same_v<
                decltype(static_cast<Action>(
                    &aon::auton::mechanisms::prepareLoaderCart)),
                Action>);
  static_assert(std::is_same_v<
                decltype(static_cast<Action>(
                    &aon::auton::mechanisms::resetLoaderCart)),
                Action>);
  static_assert(std::is_same_v<
                decltype(static_cast<Action>(
                    &aon::auton::mechanisms::prepareTopScorer)),
                Action>);
  static_assert(std::is_same_v<
                decltype(static_cast<TimedAction>(
                    &aon::auton::mechanisms::scoreTopBlocks)),
                TimedAction>);
  static_assert(std::is_same_v<
                decltype(static_cast<Action>(
                    &aon::auton::mechanisms::deployParkMechanism)),
                Action>);
  static_assert(std::is_same_v<
                decltype(static_cast<Action>(&aon::auton::mechanisms::stopAll)),
                Action>);
}
