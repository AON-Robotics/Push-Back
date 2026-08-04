//
// Created by Alberto Cruz on 12/23/2020.
//

#ifndef AON_TOOLS_FUNCTION_READER_HPP_
#define AON_TOOLS_FUNCTION_READER_HPP_
#include "aon/tools/function-registry.hpp"
#include "pros/rtos.hpp"

#include <functional>
#include <string>
#include <vector>

/**
 * FUNCTION_READER
 *
 * @brief Thread-safe registry used to select a named callable across PROS
 * tasks.
 *
 * The registry owns its callables. ExecuteFunction copies the selected
 * callable while holding the mutex, then releases the mutex before invoking
 * it. Registration can therefore proceed while a long-running routine is
 * executing. Missing names return a value-initialized T.
 *
 * @tparam T Default-constructible return type of stored functions.
 */
template <class T>
class FunctionReader {
 private:
  mutable pros::MutexVar<aon::tools::FunctionRegistry<T>> registry_;

 public:
  FunctionReader() = default;
  ~FunctionReader() = default;

  /// Add function to be stored and executed later on
  /// \param name Specific name function to be stored as
  /// \param func Callable (function pointer, lambda, or std::function)
  void AddFunction(std::string name, std::function<T()> func) {
    auto registry = registry_.lock();
    registry->add(std::move(name), std::move(func));
  }

  /// Find and execute function stored in the function map
  /// \param name Specific name of function that could be stored inside the
  /// reader
  [[nodiscard]] T ExecuteFunction(const std::string& name) const {
    std::function<T()> function;
    {
      auto registry = registry_.lock();
      function = registry->find(name);
    }
    return aon::tools::FunctionRegistry<T>::executeOrDefault(function);
  }

  /// Return full list of function names that are stored
  [[nodiscard]] std::vector<std::string> GetFunctionNames() const {
    auto registry = registry_.lock();
    return registry->names();
  }
};
#endif  // AON_TOOLS_FUNCTION_READER_HPP_
