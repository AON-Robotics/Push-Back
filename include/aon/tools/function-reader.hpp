//
// Created by Alberto Cruz on 12/23/2020.
//

#ifndef AON_TOOLS_FUNCTION_READER_HPP_
#define AON_TOOLS_FUNCTION_READER_HPP_
#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <vector>

/**
 * FUNCTION_READER
 *
 * Generates and stores functions in order for them to be executed in
 * multiple threads and to be identified in further telemetry procedures.
 *
 * @tparam T return type of stored function (only supports int, float, long,
 * double)
 *
 * Recommended to use floating point
 */
template <class T>
class FunctionReader {
 private:
  std::map<std::string, std::function<T()>> FunctionMap;
  std::vector<std::string> Keys;

 public:
  FunctionReader() {}
  ~FunctionReader() {}

  /// Add function to be stored and executed later on
  /// \param name Specific name function to be stored as
  /// \param func Callable (function pointer, lambda, or std::function)
  void AddFunction(std::string name, std::function<T()> func) {
    Keys.push_back(name);
    FunctionMap[name] = std::move(func);
  }

  /// Find and execute function stored in the function map
  /// \param name Specific name of function that could be stored inside the
  /// reader
  T ExecuteFunction(std::string name) {
    if (FunctionMap.find(name) != FunctionMap.end()) {
      return FunctionMap[name]();
    }
    T temp;
    return temp;
  }

  /// Return full list of function names that are stored
  std::vector<std::string> GetFunctionNames() {
    std::sort(Keys.begin(), Keys.end());
    return Keys;
  }
};
#endif  // AON_TOOLS_FUNCTION_READER_HPP_
