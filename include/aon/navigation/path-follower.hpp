#pragma once

#include <cstddef>
#include <cstdint>

#include "aon/navigation/path-planner.hpp"
#include "aon/odometry/pose-estimator.hpp"

namespace aon::navigation {

struct FollowerEstimate {
  localization::EstimatorPose pose{};
  double positionStandardDeviationInches = 0.0;
  double headingStandardDeviationRadians = 0.0;
  bool valid = false;
};

struct PathFollowerConfig {
  double linearGain = 0.0;
  double angularGain = 0.0;
  double maximumCommand = 0.0;
  double maximumCommandChangePerSecond = 0.0;
  double waypointToleranceInches = 0.0;
  double finalHeadingToleranceRadians = 0.0;
  double maximumPositionStandardDeviationInches = 0.0;
  double maximumHeadingStandardDeviationRadians = 0.0;
  std::uint32_t timeoutMs = 0;
  std::uint32_t blockedDwellMs = 0;
  double minimumProgressInches = 0.0;
  double minimumHeadingProgressRadians = 0.01;
};

enum class FollowerStatus : std::uint8_t {
  Idle,
  Following,
  Complete,
  Cancelled,
  TimedOut,
  Blocked,
  LocalizationUnsafe,
  InvalidPath,
};

struct FollowerOutput {
  double leftCommand = 0.0;
  double rightCommand = 0.0;
  FollowerStatus status = FollowerStatus::Idle;
  std::size_t nextPointIndex = 0;
  double distanceErrorInches = 0.0;
  double headingErrorRadians = 0.0;
};

class PathFollower {
 public:
  explicit PathFollower(PathFollowerConfig config) noexcept;

  /** @brief Starts a bounded path run using a modular millisecond timestamp. */
  [[nodiscard]] FollowerStatus start(const Path& path,
                                     double finalHeadingRadians,
                                     bool alignFinalHeading,
                                     std::uint32_t nowMs) noexcept;
  /** @brief Stops on timeout, stalled progress, or an unsafe pose estimate. */
  [[nodiscard]] FollowerOutput update(
      const FollowerEstimate& estimate, double dtSeconds,
      std::uint32_t nowMs) noexcept;
  void cancel() noexcept;
  void reset() noexcept;
  [[nodiscard]] FollowerStatus status() const noexcept;

 private:
  [[nodiscard]] bool validConfig() const noexcept;
  [[nodiscard]] bool safeEstimate(
      const FollowerEstimate& estimate) const noexcept;
  [[nodiscard]] FollowerOutput stopped(FollowerStatus status) noexcept;
  [[nodiscard]] double slew(double requested, double previous,
                            double dtSeconds) const noexcept;
  [[nodiscard]] bool recordProgress(double distanceErrorInches,
                                    double headingErrorRadians,
                                    std::uint32_t nowMs) noexcept;

  PathFollowerConfig config_;
  Path path_{};
  double finalHeadingRadians_ = 0.0;
  bool alignFinalHeading_ = false;
  FollowerStatus status_ = FollowerStatus::Idle;
  std::size_t nextPointIndex_ = 0;
  std::uint32_t startedAtMs_ = 0;
  std::uint32_t lastProgressAtMs_ = 0;
  double bestDistanceInches_ = 0.0;
  double bestHeadingErrorRadians_ = 0.0;
  double previousLeft_ = 0.0;
  double previousRight_ = 0.0;
  bool finalAlignmentStarted_ = false;
};

}  // namespace aon::navigation
