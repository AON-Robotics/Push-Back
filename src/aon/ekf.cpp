#include "aon/odometry/ekf.hpp"

#include <cmath>

namespace aon::localization {
namespace {

constexpr std::size_t kStateDimension = 3;

bool finitePose(EstimatorPose pose) noexcept {
  return std::isfinite(pose.xInches) && std::isfinite(pose.yInches) &&
         std::isfinite(pose.headingRadians);
}

bool finiteMotion(LocalMotion motion) noexcept {
  return std::isfinite(motion.rightInches) &&
         std::isfinite(motion.forwardInches) &&
         std::isfinite(motion.headingRadians);
}

bool validCovariance(const Matrix3& covariance,
                     double tolerance) noexcept {
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    if (!std::isfinite(covariance[row][row]) ||
        covariance[row][row] < -tolerance) {
      return false;
    }
    for (std::size_t column = 0; column < kStateDimension; ++column) {
      if (!std::isfinite(covariance[row][column])) {
        return false;
      }
    }
  }
  return true;
}

Matrix3 multiply(const Matrix3& left, const Matrix3& right) noexcept {
  Matrix3 result{};
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    for (std::size_t column = 0; column < kStateDimension; ++column) {
      for (std::size_t inner = 0; inner < kStateDimension; ++inner) {
        result[row][column] += left[row][inner] * right[inner][column];
      }
    }
  }
  return result;
}

Matrix3 transpose(const Matrix3& matrix) noexcept {
  Matrix3 result{};
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    for (std::size_t column = 0; column < kStateDimension; ++column) {
      result[row][column] = matrix[column][row];
    }
  }
  return result;
}

}  // namespace

Ekf::Ekf(EkfConfig config) noexcept : config_(config) { reset({0.0, 0.0, 0.0}); }

void Ekf::reset(EstimatorPose pose) noexcept {
  if (!finitePose(pose)) {
    pose = {0.0, 0.0, 0.0};
  }
  pose.headingRadians = wrapRadians(pose.headingRadians);
  state_ = pose;
  covariance_ = {};
  covariance_[0][0] = config_.initialPositionVariance;
  covariance_[1][1] = config_.initialPositionVariance;
  covariance_[2][2] = config_.initialHeadingVariance;
}

bool Ekf::predict(LocalMotion motion) noexcept {
  if (!finiteMotion(motion)) {
    return false;
  }

  const EstimatorPose candidateState = propagatePose(state_, motion);
  if (!finitePose(candidateState)) {
    return false;
  }

  const double deltaX = candidateState.xInches - state_.xInches;
  const double deltaY = candidateState.yInches - state_.yInches;
  Matrix3 jacobian{{{1.0, 0.0, deltaY},
                    {0.0, 1.0, -deltaX},
                    {0.0, 0.0, 1.0}}};

  Matrix3 candidateCovariance =
      multiply(multiply(jacobian, covariance_), transpose(jacobian));
  const double translation =
      std::hypot(motion.rightInches, motion.forwardInches);
  candidateCovariance[0][0] +=
      config_.stationaryPositionVariance +
      config_.positionVariancePerInch * translation;
  candidateCovariance[1][1] +=
      config_.stationaryPositionVariance +
      config_.positionVariancePerInch * translation;
  candidateCovariance[2][2] +=
      config_.stationaryHeadingVariance +
      config_.headingVariancePerRadian *
          std::abs(motion.headingRadians);

  if (!validCovariance(candidateCovariance,
                       config_.singularityTolerance)) {
    return false;
  }

  state_ = candidateState;
  covariance_ = candidateCovariance;
  return true;
}

EstimatorPose Ekf::pose() const noexcept { return state_; }

Matrix3 Ekf::covariance() const noexcept { return covariance_; }

CovarianceDiagonal Ekf::covarianceDiagonal() const noexcept {
  return {covariance_[0][0], covariance_[1][1], covariance_[2][2]};
}

}  // namespace aon::localization
