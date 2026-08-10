#include "aon/localization/confidence.hpp"
#include "aon/time/monotonic.hpp"

#include <algorithm>
#include <cmath>

namespace aon::localization {
namespace {

bool validPolicy(const ConfidencePolicy& policy) noexcept {
  return std::isfinite(policy.normalPositionStdDevInches) &&
         std::isfinite(policy.stopPositionStdDevInches) &&
         std::isfinite(policy.normalHeadingStdDevRadians) &&
         std::isfinite(policy.stopHeadingStdDevRadians) &&
         policy.normalPositionStdDevInches >= 0.0 &&
         policy.stopPositionStdDevInches >=
             policy.normalPositionStdDevInches &&
         policy.normalHeadingStdDevRadians >= 0.0 &&
         policy.stopHeadingStdDevRadians >=
             policy.normalHeadingStdDevRadians;
}

bool validRecoveryPolicy(const RecoveryPolicy& policy) noexcept {
  return policy.requiredConsistentObservations >= 2 &&
         std::isfinite(policy.maximumPositionSeparationInches) &&
         policy.maximumPositionSeparationInches >= 0.0 &&
         std::isfinite(policy.maximumHeadingSeparationRadians) &&
         policy.maximumHeadingSeparationRadians >= 0.0 &&
         policy.maximumObservationWindowMs > 0;
}

bool finiteCandidate(const RecoveryCandidate& candidate) noexcept {
  return std::isfinite(candidate.xInches) &&
         std::isfinite(candidate.yInches) &&
         std::isfinite(candidate.headingRadians);
}

}  // namespace

ConfidenceLevel classifyConfidence(
    CovarianceDiagonal covariance,
    const ConfidencePolicy& policy) noexcept {
  if (!validPolicy(policy) || !std::isfinite(covariance.xVariance) ||
      !std::isfinite(covariance.yVariance) ||
      !std::isfinite(covariance.headingVariance) ||
      covariance.xVariance < 0.0 || covariance.yVariance < 0.0 ||
      covariance.headingVariance < 0.0) {
    return ConfidenceLevel::StopRequired;
  }
  const double positionStdDev =
      std::sqrt(std::max(covariance.xVariance, covariance.yVariance));
  const double headingStdDev = std::sqrt(covariance.headingVariance);
  if (positionStdDev > policy.stopPositionStdDevInches ||
      headingStdDev > policy.stopHeadingStdDevRadians) {
    return ConfidenceLevel::StopRequired;
  }
  if (positionStdDev > policy.normalPositionStdDevInches ||
      headingStdDev > policy.normalHeadingStdDevRadians) {
    return ConfidenceLevel::Degraded;
  }
  return ConfidenceLevel::Normal;
}

RecoveryMonitor::RecoveryMonitor(RecoveryPolicy policy) noexcept
    : policy_(policy) {}

RecoveryDecision RecoveryMonitor::observe(
    RecoveryCandidate candidate) noexcept {
  if (!validRecoveryPolicy(policy_) || !finiteCandidate(candidate)) {
    return {RecoveryStatus::Invalid, false, consistentObservations_};
  }
  if (hasReference_ && !time::strictlyAfter(candidate.timestampMs,
                                             reference_.timestampMs)) {
    return {RecoveryStatus::OutOfOrder, false, consistentObservations_};
  }
  if (!hasReference_) {
    reference_ = candidate;
    consistentObservations_ = 1;
    hasReference_ = true;
    return {RecoveryStatus::Collecting, false, consistentObservations_};
  }

  const double dx = candidate.xInches - reference_.xInches;
  const double dy = candidate.yInches - reference_.yInches;
  const bool inWindow = time::elapsed(candidate.timestampMs,
                                      reference_.timestampMs) <=
                        policy_.maximumObservationWindowMs;
  const bool agrees =
      std::hypot(dx, dy) <= policy_.maximumPositionSeparationInches &&
      std::abs(shortestAngleDelta(reference_.headingRadians,
                                  candidate.headingRadians)) <=
          policy_.maximumHeadingSeparationRadians;
  if (!inWindow || !agrees) {
    reference_ = candidate;
    consistentObservations_ = 1;
    return {RecoveryStatus::Collecting, false, consistentObservations_};
  }

  reference_ = candidate;
  ++consistentObservations_;
  const bool allow = consistentObservations_ >=
                     policy_.requiredConsistentObservations;
  return {allow ? RecoveryStatus::Consistent : RecoveryStatus::Collecting,
          allow, consistentObservations_};
}

void RecoveryMonitor::reset() noexcept {
  reference_ = {};
  consistentObservations_ = 0;
  hasReference_ = false;
}

}  // namespace aon::localization
