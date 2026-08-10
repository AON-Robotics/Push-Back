#pragma once

#include <cstddef>
#include <cstdint>

#include "aon/odometry/ekf.hpp"

namespace aon::localization {

enum class ConfidenceLevel : std::uint8_t {
  Normal,
  Degraded,
  StopRequired,
};

struct ConfidencePolicy {
  double normalPositionStdDevInches = 2.0;
  double stopPositionStdDevInches = 6.0;
  double normalHeadingStdDevRadians = radians(5.0);
  double stopHeadingStdDevRadians = radians(20.0);
};

[[nodiscard]] ConfidenceLevel classifyConfidence(
    CovarianceDiagonal covariance,
    const ConfidencePolicy& policy) noexcept;

struct RecoveryPolicy {
  std::size_t requiredConsistentObservations = 3;
  double maximumPositionSeparationInches = 2.0;
  double maximumHeadingSeparationRadians = radians(5.0);
  std::uint32_t maximumObservationWindowMs = 300;
};

struct RecoveryCandidate {
  double xInches = 0.0;
  double yInches = 0.0;
  double headingRadians = 0.0;
  std::uint32_t timestampMs = 0;
};

enum class RecoveryStatus : std::uint8_t {
  Invalid,
  OutOfOrder,
  Collecting,
  Consistent,
};

struct RecoveryDecision {
  RecoveryStatus status = RecoveryStatus::Invalid;
  bool allowCorrection = false;
  std::size_t consistentObservations = 0;
};

class RecoveryMonitor {
 public:
  explicit RecoveryMonitor(RecoveryPolicy policy) noexcept;

  [[nodiscard]] RecoveryDecision observe(
      RecoveryCandidate candidate) noexcept;
  void reset() noexcept;

 private:
  RecoveryPolicy policy_;
  RecoveryCandidate reference_{};
  std::size_t consistentObservations_ = 0;
  bool hasReference_ = false;
};

}  // namespace aon::localization
