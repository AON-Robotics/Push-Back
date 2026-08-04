#pragma once

#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace aon::tools {

/**
 * @brief Stores named, zero-argument functions without synchronization.
 *
 * This value type owns its callables. Callers that share an instance across
 * tasks must protect every operation with the same lock. A lookup returns a
 * callable copy so execution can safely happen after that lock is released.
 *
 * @tparam T Return type of each stored callable. It must be default
 * constructible for missing-function execution.
 */
template <class T>
class FunctionRegistry {
 public:
  /** Adds or replaces a callable without duplicating its registered name. */
  void add(std::string name, std::function<T()> function) {
    if (functions_.find(name) == functions_.end()) names_.push_back(name);
    functions_[std::move(name)] = std::move(function);
  }

  /** Returns an owned callable copy, or an empty function when not found. */
  [[nodiscard]] std::function<T()> find(const std::string& name) const {
    const auto found = functions_.find(name);
    return found == functions_.end() ? std::function<T()>{} : found->second;
  }

  /** Returns all unique registered names in lexical order. */
  [[nodiscard]] std::vector<std::string> names() const {
    auto result = names_;
    std::sort(result.begin(), result.end());
    return result;
  }

  /** Executes a copied callable, returning T{} when it is empty. */
  [[nodiscard]] static T executeOrDefault(
      const std::function<T()>& function) {
    return function ? function() : T{};
  }

 private:
  std::map<std::string, std::function<T()>> functions_;
  std::vector<std::string> names_;
};

}  // namespace aon::tools
