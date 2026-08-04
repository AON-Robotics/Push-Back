#include "aon/tools/function-registry.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#define CHECK(expression)                                                   \
  do {                                                                      \
    if (!(expression)) {                                                    \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #expression     \
                << '\n';                                                    \
      std::exit(1);                                                         \
    }                                                                       \
  } while (false)

int main() {
  aon::tools::FunctionRegistry<int> registry;

  registry.add("zeta", [] { return 1; });
  registry.add("alpha", [] { return 2; });
  registry.add("zeta", [] { return 3; });

  CHECK(registry.names() == std::vector<std::string>({"alpha", "zeta"}));
  CHECK(aon::tools::FunctionRegistry<int>::executeOrDefault(
            registry.find("zeta")) == 3);
  CHECK(aon::tools::FunctionRegistry<int>::executeOrDefault(
            registry.find("missing")) == 0);

  int invocations = 0;
  registry.add("side-effect", [&] {
    ++invocations;
    return 4;
  });
  const auto callable = registry.find("side-effect");
  registry.add("side-effect", [] { return 5; });
  CHECK(aon::tools::FunctionRegistry<int>::executeOrDefault(callable) == 4);
  CHECK(invocations == 1);

  std::cout << "function registry tests passed\n";
}
