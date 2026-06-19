#pragma once

#include <cstdio>
#include <string>

namespace aon::logging {

inline void Initialize() {}

inline void Error(const std::string& message) {
  std::fprintf(stderr, "[ERROR] %s\n", message.c_str());
}

inline void Warn(const std::string& message) {
  std::fprintf(stderr, "[WARN] %s\n", message.c_str());
}

inline void Debug(const std::string& message) {
  std::fprintf(stdout, "[DEBUG] %s\n", message.c_str());
}

inline void Close() {
  std::fflush(stdout);
  std::fflush(stderr);
}

}  // namespace aon::logging
