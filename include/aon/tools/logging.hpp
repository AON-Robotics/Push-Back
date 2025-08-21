/**
 * \file logging.hpp
 *
 * \author Alberto I. Cruz Salamán
 * \brief Contains global logger object to report runtime error or to generate
 * warnings inside internal classes
 */

#ifndef AON_TOOLS_LOGGING_HPP_
#define AON_TOOLS_LOGGING_HPP_

#include <fstream>
#include <memory>
#include <string>
#include "../../okapi/api.hpp"

/// \namespace aon::logging
/// \brief Logging functions for general use and error handling
namespace aon::logging {

/// Warn logger
static std::unique_ptr<okapi::Logger> WarnLogger;

/// Error Logger
static std::unique_ptr<okapi::Logger> ErrorLogger;

/// Configuration  of global logger
/// Output at the moment limited to the pros terminal
inline void Initialize() {
  WarnLogger = std::make_unique<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      stdout,                        // Using global stdout
      okapi::Logger::LogLevel::warn  // Show errors and warnings
  );

  ErrorLogger = std::make_unique<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      stderr,                         // Using global stderr
      okapi::Logger::LogLevel::error  // Show errors and warnings
  );

  okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
      okapi::TimeUtilFactory::createDefault().getTimer(),  // It needs a Timer
      stdout,  // Using global stdout
      okapi::Logger::LogLevel::debug));
}

/// Log an error inside the project
/// \param message Message to be added with the error
static void Error(std::string message) {
  ErrorLogger->error([message]() { return message; });
  okapi::Logger::getDefaultLogger()->error([message]() { return message; });
}

/// Log a warning inside the project
/// \param message Message to be added with the warning
static void Warn(std::string message) {
  WarnLogger->warn([message]() { return message; });
  okapi::Logger::getDefaultLogger()->warn([message]() { return message; });
}

/// Log a debug message
/// \param message Message to be added to the debug log
static void Debug(std::string message) {
  okapi::Logger::getDefaultLogger()->debug([message]() { return message; });
}

static void Close() {
  WarnLogger->close();
  ErrorLogger->close();
}

}  // namespace aon::logging

#endif  // AON_TOOLS_LOGGING_HPP_
