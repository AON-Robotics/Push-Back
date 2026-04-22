#include <cstdlib>

namespace std {

[[noreturn]] void __attribute__((weak)) __throw_bad_array_new_length() {
  std::abort();
}

}  // namespace std
