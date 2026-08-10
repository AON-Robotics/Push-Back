#include "aon/navigation/path-follower.hpp"
#include "aon/time/monotonic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aon::navigation {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapRadians(double angle) noexcept {
  if (!std::isfinite(angle)) return angle;
  angle = std::fmod(angle + kPi, 2.0 * kPi);
  if (angle < 0.0) angle += 2.0 * kPi;
  return angle - kPi;
}

double pointDistance(field::Point2 point,
                     const localization::EstimatorPose& pose) noexcept {
  return std::hypot(point.xInches - pose.xInches,
                    point.yInches - pose.yInches);
}

}  // namespace

PathFollower::PathFollower(PathFollowerConfig config) noexcept
    : config_(config) {}

bool PathFollower::validConfig() const noexcept {
  return std::isfinite(config_.linearGain) && config_.linearGain >= 0.0 &&
         std::isfinite(config_.angularGain) && config_.angularGain >= 0.0 &&
         std::isfinite(config_.maximumCommand) && config_.maximumCommand > 0.0 &&
         std::isfinite(config_.maximumCommandChangePerSecond) &&
         config_.maximumCommandChangePerSecond > 0.0 &&
         std::isfinite(config_.waypointToleranceInches) &&
         config_.waypointToleranceInches >= 0.0 &&
         std::isfinite(config_.finalHeadingToleranceRadians) &&
         config_.finalHeadingToleranceRadians >= 0.0 &&
         std::isfinite(config_.maximumPositionStandardDeviationInches) &&
         config_.maximumPositionStandardDeviationInches >= 0.0 &&
         std::isfinite(config_.maximumHeadingStandardDeviationRadians) &&
         config_.maximumHeadingStandardDeviationRadians >= 0.0 &&
         time::validInterval(config_.timeoutMs) &&
         time::validInterval(config_.blockedDwellMs) &&
         std::isfinite(config_.minimumProgressInches) &&
         config_.minimumProgressInches >= 0.0 &&
         std::isfinite(config_.minimumHeadingProgressRadians) &&
         config_.minimumHeadingProgressRadians >= 0.0;
}

FollowerStatus PathFollower::start(const Path& path,
                                   double finalHeadingRadians,
                                   bool alignFinalHeading,
                                   std::uint32_t nowMs) noexcept {
  reset();
  if (!validConfig() || path.size == 0 || path.size > path.points.size() ||
      !std::isfinite(finalHeadingRadians)) {
    status_ = FollowerStatus::InvalidPath;
    return status_;
  }
  for (std::size_t index = 0; index < path.size; ++index) {
    if (!field::isFinite(path.points[index])) {
      status_ = FollowerStatus::InvalidPath;
      return status_;
    }
  }
  path_ = path;
  finalHeadingRadians_ = wrapRadians(finalHeadingRadians);
  alignFinalHeading_ = alignFinalHeading;
  nextPointIndex_ = path.size > 1 ? 1 : 0;
  startedAtMs_ = nowMs;
  lastProgressAtMs_ = nowMs;
  bestDistanceInches_ = std::numeric_limits<double>::infinity();
  bestHeadingErrorRadians_ = std::numeric_limits<double>::infinity();
  status_ = FollowerStatus::Following;
  return status_;
}

bool PathFollower::safeEstimate(
    const FollowerEstimate& estimate) const noexcept {
  return estimate.valid && localization::isFinite(estimate.pose) &&
         std::isfinite(estimate.positionStandardDeviationInches) &&
         estimate.positionStandardDeviationInches >= 0.0 &&
         estimate.positionStandardDeviationInches <=
             config_.maximumPositionStandardDeviationInches &&
         std::isfinite(estimate.headingStandardDeviationRadians) &&
         estimate.headingStandardDeviationRadians >= 0.0 &&
         estimate.headingStandardDeviationRadians <=
             config_.maximumHeadingStandardDeviationRadians;
}

FollowerOutput PathFollower::stopped(FollowerStatus status) noexcept {
  status_ = status;
  previousLeft_ = 0.0;
  previousRight_ = 0.0;
  return {0.0, 0.0, status_, nextPointIndex_, 0.0, 0.0};
}

double PathFollower::slew(double requested, double previous,
                          double dtSeconds) const noexcept {
  const double maximumChange =
      config_.maximumCommandChangePerSecond * dtSeconds;
  return previous +
         std::clamp(requested - previous, -maximumChange, maximumChange);
}

bool PathFollower::recordProgress(double distanceErrorInches,
                                  double headingErrorRadians,
                                  std::uint32_t nowMs) noexcept {
  const double absoluteHeadingError = std::abs(headingErrorRadians);
  const double distanceImprovement =
      bestDistanceInches_ - distanceErrorInches;
  const double headingImprovement =
      bestHeadingErrorRadians_ - absoluteHeadingError;
  const bool distanceProgress = config_.minimumProgressInches > 0.0
                                    ? distanceImprovement >=
                                          config_.minimumProgressInches
                                    : distanceImprovement > 0.0;
  const bool headingProgress = config_.minimumHeadingProgressRadians > 0.0
                                   ? headingImprovement >=
                                         config_.minimumHeadingProgressRadians
                                   : headingImprovement > 0.0;
  bool progressed = false;
  if (!std::isfinite(bestDistanceInches_) ||
      distanceProgress) {
    bestDistanceInches_ = distanceErrorInches;
    progressed = true;
  }
  if (!std::isfinite(bestHeadingErrorRadians_) ||
      headingProgress) {
    bestHeadingErrorRadians_ = absoluteHeadingError;
    progressed = true;
  }
  if (progressed) {
    lastProgressAtMs_ = nowMs;
    return true;
  }
  return !time::elapsedAtLeast(nowMs, lastProgressAtMs_,
                               config_.blockedDwellMs);
}

FollowerOutput PathFollower::update(const FollowerEstimate& estimate,
                                    double dtSeconds,
                                    std::uint32_t nowMs) noexcept {
  if (status_ != FollowerStatus::Following) return stopped(status_);
  if (!safeEstimate(estimate)) return stopped(FollowerStatus::LocalizationUnsafe);
  if (!std::isfinite(dtSeconds) || dtSeconds <= 0.0 || dtSeconds > 0.25) {
    return stopped(FollowerStatus::LocalizationUnsafe);
  }
  if (nowMs - startedAtMs_ >= config_.timeoutMs) {
    return stopped(FollowerStatus::TimedOut);
  }

  double distanceError = pointDistance(path_.points[nextPointIndex_], estimate.pose);
  while (distanceError <= config_.waypointToleranceInches &&
         nextPointIndex_ + 1 < path_.size) {
    ++nextPointIndex_;
    distanceError = pointDistance(path_.points[nextPointIndex_], estimate.pose);
    bestDistanceInches_ = std::numeric_limits<double>::infinity();
    lastProgressAtMs_ = nowMs;
  }

  if (distanceError <= config_.waypointToleranceInches &&
      nextPointIndex_ + 1 == path_.size) {
    const double finalError =
        wrapRadians(finalHeadingRadians_ - estimate.pose.headingRadians);
    if (!alignFinalHeading_ ||
        std::abs(finalError) <= config_.finalHeadingToleranceRadians) {
      return stopped(FollowerStatus::Complete);
    }
    if (!finalAlignmentStarted_) {
      finalAlignmentStarted_ = true;
      bestDistanceInches_ = std::numeric_limits<double>::infinity();
      bestHeadingErrorRadians_ = std::numeric_limits<double>::infinity();
      lastProgressAtMs_ = nowMs;
    }
    if (!recordProgress(distanceError, finalError, nowMs)) {
      return stopped(FollowerStatus::Blocked);
    }
    const double angular = std::clamp(config_.angularGain * finalError,
                                      -config_.maximumCommand,
                                      config_.maximumCommand);
    const double left = slew(angular, previousLeft_, dtSeconds);
    const double right = slew(-angular, previousRight_, dtSeconds);
    previousLeft_ = left;
    previousRight_ = right;
    return {left, right, status_, nextPointIndex_, distanceError, finalError};
  }

  const field::Point2 target = path_.points[nextPointIndex_];
  const double dx = target.xInches - estimate.pose.xInches;
  const double dy = target.yInches - estimate.pose.yInches;
  const double targetHeading = std::atan2(dx, dy);
  const double headingError =
      wrapRadians(targetHeading - estimate.pose.headingRadians);
  finalAlignmentStarted_ = false;
  if (!recordProgress(distanceError, headingError, nowMs)) {
    return stopped(FollowerStatus::Blocked);
  }

  const double forwardAlignment = std::max(0.0, std::cos(headingError));
  const double linear = std::clamp(
      config_.linearGain * distanceError * forwardAlignment,
      0.0, config_.maximumCommand);
  const double angular = std::clamp(config_.angularGain * headingError,
                                    -config_.maximumCommand,
                                    config_.maximumCommand);
  const double requestedLeft =
      std::clamp(linear + angular, -config_.maximumCommand,
                 config_.maximumCommand);
  const double requestedRight =
      std::clamp(linear - angular, -config_.maximumCommand,
                 config_.maximumCommand);
  const double left = slew(requestedLeft, previousLeft_, dtSeconds);
  const double right = slew(requestedRight, previousRight_, dtSeconds);
  previousLeft_ = left;
  previousRight_ = right;
  return {left, right, status_, nextPointIndex_, distanceError, headingError};
}

void PathFollower::cancel() noexcept { status_ = FollowerStatus::Cancelled; }

void PathFollower::reset() noexcept {
  path_ = {};
  finalHeadingRadians_ = 0.0;
  alignFinalHeading_ = false;
  status_ = FollowerStatus::Idle;
  nextPointIndex_ = 0;
  startedAtMs_ = 0;
  lastProgressAtMs_ = 0;
  bestDistanceInches_ = 0.0;
  bestHeadingErrorRadians_ = 0.0;
  previousLeft_ = 0.0;
  previousRight_ = 0.0;
  finalAlignmentStarted_ = false;
}

FollowerStatus PathFollower::status() const noexcept { return status_; }

}  // namespace aon::navigation
