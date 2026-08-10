#pragma once

#include <cstdio>
#include <string>

#include "aon/odometry/diagnostics.hpp"

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

inline void WriteLocalizationCsvHeader(std::FILE* output) noexcept {
  if (output == nullptr) return;
  std::fprintf(
      output,
      "time_ms,raw_x,raw_y,raw_theta,fused_x,fused_y,fused_theta,"
      "imu_theta,imu_valid,gps_x,gps_y,gps_theta,gps_valid,gps_reason,"
      "p_x,p_y,p_theta,dt_s,execution_us,deadline_misses\n");
}

inline void WriteLocalizationCsvRow(
    std::FILE* output,
    const localization::LocalizationDiagnostics& diagnostics) noexcept {
  if (output == nullptr) return;
  // Formatting is deliberately caller-driven; the real-time estimator never
  // performs terminal or SD-card I/O while holding its snapshot cadence.
  std::fprintf(
      output,
      "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f,"
      "%u,%u,%.9f,%.9f,%.9f,%.6f,%lu,%lu\n",
      static_cast<unsigned long>(diagnostics.timestampMs),
      diagnostics.rawPose.xInches, diagnostics.rawPose.yInches,
      localization::degrees(diagnostics.rawPose.headingRadians),
      diagnostics.fusedPose.xInches, diagnostics.fusedPose.yInches,
      localization::degrees(diagnostics.fusedPose.headingRadians),
      localization::degrees(diagnostics.imu.headingRadians),
      diagnostics.imu.valid ? 1U : 0U, diagnostics.gps.xInches,
      diagnostics.gps.yInches,
      localization::degrees(diagnostics.gps.headingRadians),
      diagnostics.gpsPositionAccepted ? 1U : 0U,
      static_cast<unsigned>(diagnostics.gpsRejectionReason),
      diagnostics.covariance.xVariance, diagnostics.covariance.yVariance,
      diagnostics.covariance.headingVariance, diagnostics.dtSeconds,
      static_cast<unsigned long>(diagnostics.executionMicroseconds),
      static_cast<unsigned long>(diagnostics.deadlineMisses));
}

inline void Close() {
  std::fflush(stdout);
  std::fflush(stderr);
}

}  // namespace aon::logging
