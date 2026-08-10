#include "aon/odometry/ekf.hpp"

#include <cmath>
#include <limits>

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

void stabilizeCovariance(Matrix3& covariance, double tolerance) noexcept {
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    if (covariance[row][row] < 0.0 &&
        covariance[row][row] >= -tolerance) {
      covariance[row][row] = 0.0;
    }
    for (std::size_t column = row + 1; column < kStateDimension; ++column) {
      const double average =
          (covariance[row][column] + covariance[column][row]) / 2.0;
      covariance[row][column] = average;
      covariance[column][row] = average;
    }
  }
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

  stabilizeCovariance(candidateCovariance, config_.singularityTolerance);
  if (!validCovariance(candidateCovariance,
                       config_.singularityTolerance)) {
    return false;
  }

  state_ = candidateState;
  covariance_ = candidateCovariance;
  return true;
}

bool Ekf::updateImuHeading(double headingRadians) noexcept {
  return updateHeading(headingRadians, config_.imuHeadingVariance,
                       std::numeric_limits<double>::infinity());
}

bool Ekf::updateGpsPosition(double xInches, double yInches,
                            double maximumNis) noexcept {
  const double measurementVariance = config_.gpsPositionVariance;
  if (!std::isfinite(xInches) || !std::isfinite(yInches) ||
      !std::isfinite(measurementVariance) || measurementVariance < 0.0 ||
      std::isnan(maximumNis) || maximumNis < 0.0) {
    return false;
  }

  const double innovationX = xInches - state_.xInches;
  const double innovationY = yInches - state_.yInches;
  const double innovation00 = covariance_[0][0] + measurementVariance;
  const double innovation01 = covariance_[0][1];
  const double innovation10 = covariance_[1][0];
  const double innovation11 = covariance_[1][1] + measurementVariance;
  const double determinant =
      innovation00 * innovation11 - innovation01 * innovation10;
  if (!std::isfinite(determinant) ||
      determinant <= config_.singularityTolerance) {
    return false;
  }

  const double inverse00 = innovation11 / determinant;
  const double inverse01 = -innovation01 / determinant;
  const double inverse10 = -innovation10 / determinant;
  const double inverse11 = innovation00 / determinant;
  const double nis =
      innovationX * (inverse00 * innovationX + inverse01 * innovationY) +
      innovationY * (inverse10 * innovationX + inverse11 * innovationY);
  if (!std::isfinite(nis) || nis > maximumNis) {
    return false;
  }

  std::array<std::array<double, 2>, kStateDimension> gain{};
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    gain[row][0] =
        covariance_[row][0] * inverse00 +
        covariance_[row][1] * inverse10;
    gain[row][1] =
        covariance_[row][0] * inverse01 +
        covariance_[row][1] * inverse11;
  }

  EstimatorPose candidateState{
      state_.xInches + gain[0][0] * innovationX +
          gain[0][1] * innovationY,
      state_.yInches + gain[1][0] * innovationX +
          gain[1][1] * innovationY,
      wrapRadians(state_.headingRadians + gain[2][0] * innovationX +
                  gain[2][1] * innovationY),
  };

  Matrix3 josephLeft{{{1.0 - gain[0][0], -gain[0][1], 0.0},
                      {-gain[1][0], 1.0 - gain[1][1], 0.0},
                      {-gain[2][0], -gain[2][1], 1.0}}};
  Matrix3 candidateCovariance = multiply(
      multiply(josephLeft, covariance_), transpose(josephLeft));
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    for (std::size_t column = 0; column < kStateDimension; ++column) {
      candidateCovariance[row][column] +=
          measurementVariance *
          (gain[row][0] * gain[column][0] +
           gain[row][1] * gain[column][1]);
    }
  }
  stabilizeCovariance(candidateCovariance, config_.singularityTolerance);

  if (!finitePose(candidateState) ||
      !validCovariance(candidateCovariance,
                       config_.singularityTolerance)) {
    return false;
  }

  state_ = candidateState;
  covariance_ = candidateCovariance;
  return true;
}

bool Ekf::updateGpsHeading(double headingRadians, double maximumNis,
                           bool enabled) noexcept {
  if (!enabled) {
    return false;
  }
  return updateHeading(headingRadians, config_.gpsHeadingVariance,
                       maximumNis);
}

bool Ekf::updateAxisPosition(PositionAxis axis, double positionInches,
                             double measurementVariance,
                             double maximumNis) noexcept {
  if (!std::isfinite(positionInches) ||
      !std::isfinite(measurementVariance) || measurementVariance < 0.0 ||
      std::isnan(maximumNis) || maximumNis < 0.0) {
    return false;
  }
  const std::size_t observed = axis == PositionAxis::X ? 0U : 1U;
  const double current = observed == 0 ? state_.xInches : state_.yInches;
  const double innovation = positionInches - current;
  const double innovationVariance =
      covariance_[observed][observed] + measurementVariance;
  if (!std::isfinite(innovationVariance) ||
      innovationVariance <= config_.singularityTolerance) {
    return false;
  }
  const double nis = innovation * innovation / innovationVariance;
  if (!std::isfinite(nis) || nis > maximumNis) return false;

  std::array<double, kStateDimension> gain{};
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    gain[row] = covariance_[row][observed] / innovationVariance;
  }
  EstimatorPose candidateState{
      state_.xInches + gain[0] * innovation,
      state_.yInches + gain[1] * innovation,
      wrapRadians(state_.headingRadians + gain[2] * innovation)};

  Matrix3 josephLeft{{{1.0, 0.0, 0.0},
                      {0.0, 1.0, 0.0},
                      {0.0, 0.0, 1.0}}};
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    josephLeft[row][observed] -= gain[row];
  }
  Matrix3 candidateCovariance = multiply(
      multiply(josephLeft, covariance_), transpose(josephLeft));
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    for (std::size_t column = 0; column < kStateDimension; ++column) {
      candidateCovariance[row][column] +=
          measurementVariance * gain[row] * gain[column];
    }
  }
  stabilizeCovariance(candidateCovariance, config_.singularityTolerance);
  if (!finitePose(candidateState) ||
      !validCovariance(candidateCovariance, config_.singularityTolerance)) {
    return false;
  }
  state_ = candidateState;
  covariance_ = candidateCovariance;
  return true;
}

bool Ekf::updateHeading(double headingRadians, double measurementVariance,
                        double maximumNis) noexcept {
  if (!std::isfinite(headingRadians) ||
      !std::isfinite(measurementVariance) || measurementVariance < 0.0) {
    return false;
  }

  const double innovationVariance =
      covariance_[2][2] + measurementVariance;
  if (!std::isfinite(innovationVariance) ||
      innovationVariance <= config_.singularityTolerance) {
    return false;
  }

  const double innovation =
      shortestAngleDelta(state_.headingRadians, headingRadians);
  const double nis = innovation * innovation / innovationVariance;
  if (std::isnan(maximumNis) || maximumNis < 0.0 ||
      !std::isfinite(nis) || nis > maximumNis) {
    return false;
  }
  std::array<double, kStateDimension> gain{};
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    gain[row] = covariance_[row][2] / innovationVariance;
  }

  EstimatorPose candidateState{
      state_.xInches + gain[0] * innovation,
      state_.yInches + gain[1] * innovation,
      wrapRadians(state_.headingRadians + gain[2] * innovation),
  };

  Matrix3 josephLeft{{{1.0, 0.0, -gain[0]},
                      {0.0, 1.0, -gain[1]},
                      {0.0, 0.0, 1.0 - gain[2]}}};
  Matrix3 candidateCovariance = multiply(
      multiply(josephLeft, covariance_), transpose(josephLeft));
  for (std::size_t row = 0; row < kStateDimension; ++row) {
    for (std::size_t column = 0; column < kStateDimension; ++column) {
      candidateCovariance[row][column] +=
          gain[row] * measurementVariance * gain[column];
    }
  }

  // Roundoff can make mirrored entries differ after many updates. Averaging
  // preserves the covariance meaning without concealing a genuinely bad input.
  stabilizeCovariance(candidateCovariance, config_.singularityTolerance);

  if (!finitePose(candidateState) ||
      !validCovariance(candidateCovariance,
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
