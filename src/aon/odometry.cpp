#include "aon/odometry/odometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "aon/constants.hpp"
#include "pros/error.h"

namespace aon {
namespace {

constexpr double kInchesPerMeter = 39.37007874015748;

Pose publicPose(localization::EstimatorPose pose) noexcept {
  return {pose.xInches, pose.yInches,
          localization::degrees(pose.headingRadians)};
}

double trackingDistance(std::int32_t centidegrees,
                        double distancePerCentidegree) noexcept {
  return static_cast<double>(centidegrees) * distancePerCentidegree;
}

bool validRotationReading(std::int32_t value) noexcept {
  return value != PROS_ERR;
}

}  // namespace

Odometry::Odometry(const config::LocalizationConfig& config,
                   std::int8_t leftPort, std::int8_t rightPort,
                   std::int8_t backPort, std::int8_t imuPort)
    : config_(config),
      distancePerCentidegree_(localization::kPi *
                             config.trackingWheelDiameterInches / 36000.0),
      encoderLeft_(std::abs(static_cast<int>(leftPort))),
      encoderRight_(std::abs(static_cast<int>(rightPort))),
      encoderBack_(std::abs(static_cast<int>(backPort))),
      imu_(std::abs(static_cast<int>(imuPort))),
      ekf_(config.ekf),
      gpsGate_(config.gps.validation) {
  encoderLeft_.set_reversed(leftPort < 0);
  encoderRight_.set_reversed(rightPort < 0);
  encoderBack_.set_reversed(backPort < 0);

  if (config_.gps.enabled && config_.gps.port >= 1 &&
      config_.gps.port <= 21) {
    gps_.emplace(static_cast<std::uint8_t>(config_.gps.port),
                 config_.gps.xOffsetMeters, config_.gps.yOffsetMeters);
  }
  resetPose(0.0, 0.0, 0.0);
}

Pose Odometry::getPose() {
  snapshotMutex_.take();
  const Pose pose = published_.fusedPose;
  snapshotMutex_.give();
  return pose;
}

Pose Odometry::rawOdometryPose() {
  snapshotMutex_.take();
  const Pose pose = published_.rawPose;
  snapshotMutex_.give();
  return pose;
}

double Odometry::getX() { return getPose().x; }

double Odometry::getY() { return getPose().y; }

double Odometry::getDegrees() { return getPose().theta; }

Vector Odometry::getPosition() {
  const Pose pose = getPose();
  return Vector().SetPosition(pose.x, pose.y);
}

localization::LocalizationDiagnostics Odometry::getDiagnostics() {
  snapshotMutex_.take();
  const localization::LocalizationDiagnostics diagnostics =
      published_.diagnostics;
  snapshotMutex_.give();
  return diagnostics;
}

void Odometry::resetPose(double x, double y, double thetaDegrees) {
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(thetaDegrees)) {
    return;
  }
  const std::int32_t leftReading = encoderLeft_.get_position();
  const std::int32_t rightReading = encoderRight_.get_position();
  const std::int32_t backReading = encoderBack_.get_position();
  const double imuRotationDegrees = imu_.get_rotation();
  const bool imuValid = std::isfinite(imuRotationDegrees) &&
                        imu_.get_status() == pros::ImuStatus::ready;

  const localization::EstimatorPose requested{
      x, y, localization::wrapRadians(localization::radians(thetaDegrees))};

  snapshotMutex_.take();
  ++generation_;
  wheelBaselines_ = {
      validRotationReading(leftReading)
          ? trackingDistance(leftReading, distancePerCentidegree_)
          : 0.0,
      validRotationReading(rightReading)
          ? trackingDistance(rightReading, distancePerCentidegree_)
          : 0.0,
      validRotationReading(backReading)
          ? trackingDistance(backReading, distancePerCentidegree_)
          : 0.0,
      validRotationReading(leftReading),
      validRotationReading(rightReading),
      validRotationReading(backReading),
  };
  imuFieldOffsetValid_ = imuValid;
  imuFieldOffsetRadians_ =
      imuValid ? requested.headingRadians -
                     localization::radians(imuRotationDegrees)
               : 0.0;
  rawPose_ = requested;
  ekf_.reset(requested);
  velocityEstimator_.reset();
  gpsGate_.reset();
  gpsFreshness_.reset();
  lastUpdateMs_ = pros::millis();

  localization::LocalizationDiagnostics diagnostics{};
  diagnostics.timestampMs = lastUpdateMs_;
  diagnostics.rawPose = requested;
  diagnostics.fusedPose = requested;
  diagnostics.covariance = ekf_.covarianceDiagonal();
  diagnostics.resetCount = published_.diagnostics.resetCount + 1U;
  published_ = {publicPose(requested), publicPose(requested), diagnostics};
  snapshotMutex_.give();
}

void Odometry::update() {
  const std::uint32_t executionStartUs = pros::micros();
  const std::uint32_t nowMs = pros::millis();

  // Capture reset-sensitive state before sampling hardware. A reset that
  // overlaps this update changes generation_ and discards the whole sample.
  snapshotMutex_.take();
  const std::uint32_t generation = generation_;
  localization::EstimatorPose candidateRawPose = rawPose_;
  localization::Ekf candidateEkf = ekf_;
  localization::GpsGate candidateGpsGate = gpsGate_;
  localization::GpsFreshnessTracker candidateGpsFreshness = gpsFreshness_;
  localization::VelocityEstimator candidateVelocityEstimator =
      velocityEstimator_;
  localization::WheelDistances candidateBaselines = wheelBaselines_;
  double candidateImuOffset = imuFieldOffsetRadians_;
  bool candidateImuOffsetValid = imuFieldOffsetValid_;
  localization::LocalizationDiagnostics diagnostics = published_.diagnostics;
  const std::uint32_t previousUpdateMs = lastUpdateMs_;
  snapshotMutex_.give();

  const std::int32_t leftReading = encoderLeft_.get_position();
  const std::int32_t rightReading = encoderRight_.get_position();
  const std::int32_t backReading = encoderBack_.get_position();
  const bool leftValid = validRotationReading(leftReading);
  const bool rightValid = validRotationReading(rightReading);
  const bool backValid = validRotationReading(backReading);
  const localization::WheelDistances currentWheels{
      leftValid ? trackingDistance(leftReading, distancePerCentidegree_) : 0.0,
      rightValid ? trackingDistance(rightReading, distancePerCentidegree_) : 0.0,
      backValid ? trackingDistance(backReading, distancePerCentidegree_) : 0.0,
      leftValid,
      rightValid,
      backValid,
  };

  const double imuRotationDegrees = imu_.get_rotation();
  const bool imuValid = std::isfinite(imuRotationDegrees) &&
                        imu_.get_status() == pros::ImuStatus::ready;

  localization::GpsMeasurement gpsMeasurement{};
  if (gps_) {
    const pros::gps_position_s_t position = gps_->get_position();
    const double errorMeters = gps_->get_error();
    const double headingDegrees = gps_->get_heading();
    gpsMeasurement = {
        position.x * kInchesPerMeter,
        position.y * kInchesPerMeter,
        localization::radians(headingDegrees +
                              config_.gps.headingOffsetDegrees),
        errorMeters * kInchesPerMeter,
        std::isfinite(position.x) && std::isfinite(position.y) &&
            std::isfinite(errorMeters),
        std::isfinite(headingDegrees),
        false,
        0U,
    };
    candidateGpsFreshness.observe(gpsMeasurement, nowMs);
  }

  const double dtSeconds = previousUpdateMs == 0U
                               ? static_cast<double>(config_.loopPeriodMs) /
                                     1000.0
                               : static_cast<double>(nowMs - previousUpdateMs) /
                                     1000.0;

  const localization::WheelDeltas wheelDeltas =
      localization::consumeWheelDistances(currentWheels, candidateBaselines);
  candidateBaselines = currentWheels;

  const localization::LocalMotion motion =
      localization::localMotion(wheelDeltas, config_.geometry);
  if (std::isfinite(motion.headingRadians)) {
    const localization::VelocityUpdateResult velocityResult =
        candidateVelocityEstimator.update(
            motion, candidateEkf.pose().headingRadians, dtSeconds);
    if (velocityResult == localization::VelocityUpdateResult::InvalidMotion ||
        velocityResult == localization::VelocityUpdateResult::InvalidConfig) {
      ++diagnostics.numericalRejections;
    }
    const localization::EstimatorPose propagated =
        localization::propagatePose(candidateRawPose, motion);
    if (std::isfinite(propagated.xInches)) {
      candidateRawPose = propagated;
    } else {
      ++diagnostics.numericalRejections;
    }
    if (!candidateEkf.predict(motion)) ++diagnostics.numericalRejections;
  }

  if (imuValid) {
    if (!candidateImuOffsetValid) {
      // A sensor that recovers after reset is aligned to the current estimate,
      // avoiding a discontinuity when the first valid sample arrives.
      candidateImuOffset =
          candidateEkf.pose().headingRadians -
          localization::radians(imuRotationDegrees);
      candidateImuOffsetValid = true;
    }
    const double fieldHeading = localization::wrapRadians(
        localization::radians(imuRotationDegrees) + candidateImuOffset);
    if (!candidateEkf.updateImuHeading(fieldHeading)) {
      ++diagnostics.numericalRejections;
    }
    diagnostics.imu = {fieldHeading, true};
  } else {
    diagnostics.imu = {0.0, false};
    ++diagnostics.imuSensorErrors;
  }

  diagnostics.gps = gpsMeasurement;
  diagnostics.gpsPositionAccepted = false;
  diagnostics.gpsHeadingAccepted = false;
  diagnostics.gpsRejectionReason = localization::GpsRejectionReason::None;
  if (gps_) {
    const localization::GpsGateResult gate =
        candidateGpsGate.evaluate(gpsMeasurement);
    diagnostics.gpsRejectionReason = gate.reason;
    if (gate.positionAccepted) {
      diagnostics.gpsPositionAccepted = candidateEkf.updateGpsPosition(
          gpsMeasurement.xInches, gpsMeasurement.yInches,
          config_.gps.validation.maximumPositionNis);
      if (!diagnostics.gpsPositionAccepted) {
        diagnostics.gpsRejectionReason =
            localization::GpsRejectionReason::InnovationRejected;
        ++diagnostics.numericalRejections;
      }
    }
    if (gate.headingAccepted && config_.gps.headingUpdateEnabled) {
      diagnostics.gpsHeadingAccepted = candidateEkf.updateGpsHeading(
          gpsMeasurement.headingRadians,
          config_.gps.validation.maximumHeadingNis, true);
      if (!diagnostics.gpsHeadingAccepted) {
        diagnostics.gpsRejectionReason =
            localization::GpsRejectionReason::InnovationRejected;
        ++diagnostics.numericalRejections;
      }
    }
    candidateGpsGate.commit(gpsMeasurement,
                            diagnostics.gpsPositionAccepted,
                            diagnostics.gpsHeadingAccepted);
    if (!gpsMeasurement.positionValid) ++diagnostics.gpsSensorErrors;
  }

  if (!leftValid || !rightValid || !backValid) {
    ++diagnostics.wheelSensorErrors;
  }
  diagnostics.rawPose = candidateRawPose;
  diagnostics.fusedPose = candidateEkf.pose();
  diagnostics.timestampMs = nowMs;
  diagnostics.wheelDistances = currentWheels;
  diagnostics.covariance = candidateEkf.covarianceDiagonal();
  diagnostics.velocity = candidateVelocityEstimator.velocity();
  diagnostics.dtSeconds = dtSeconds;
  diagnostics.wheelHeadingDeltaRadians = motion.headingRadians;
  diagnostics.executionMicroseconds = pros::micros() - executionStartUs;
  diagnostics.maximumExecutionMicroseconds =
      std::max(diagnostics.maximumExecutionMicroseconds,
               diagnostics.executionMicroseconds);

  snapshotMutex_.take();
  if (generation_ == generation) {
    rawPose_ = candidateRawPose;
    ekf_ = candidateEkf;
    gpsGate_ = candidateGpsGate;
    gpsFreshness_ = candidateGpsFreshness;
    velocityEstimator_ = candidateVelocityEstimator;
    wheelBaselines_ = candidateBaselines;
    imuFieldOffsetRadians_ = candidateImuOffset;
    imuFieldOffsetValid_ = candidateImuOffsetValid;
    lastUpdateMs_ = nowMs;
    published_ = {publicPose(candidateRawPose),
                  publicPose(candidateEkf.pose()), diagnostics};
  }
  snapshotMutex_.give();
}

bool Odometry::calibrateImu(std::uint32_t timeoutMs) {
  if (imu_.reset(false) == PROS_ERR) return false;
  const std::uint32_t start = pros::millis();
  while (imu_.get_status() == pros::ImuStatus::calibrating) {
    if (pros::millis() - start >= timeoutMs) return false;
    pros::delay(10U);
  }
  return imu_.get_status() == pros::ImuStatus::ready;
}

void Odometry::runLocalizationLoop() {
  runLocalizationLoop(nullptr);
}

void Odometry::runLocalizationLoop(void (*publisher)(const Pose&)) {
  resetPose(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y,
            INITIAL_ODOMETRY_THETA);
  std::uint32_t wake = pros::millis();
  while (true) {
    const std::uint32_t deadline = wake + config_.loopPeriodMs;
    update();
    publishCurrent(publisher);
    if (static_cast<std::int32_t>(pros::millis() - deadline) > 0) {
      recordDeadlineMiss();
    }
    pros::Task::delay_until(&wake, config_.loopPeriodMs);
  }
}

void Odometry::publishCurrent(void (*publisher)(const Pose&)) {
  if (publisher == nullptr) return;
  snapshotMutex_.take();
  const Pose publicPose = published_.fusedPose;
  // Serialize publication with resetPose so an old snapshot cannot reach the
  // path follower after a newer reset has already been published.
  publisher(publicPose);
  snapshotMutex_.give();
}

void Odometry::recordDeadlineMiss() {
  snapshotMutex_.take();
  ++published_.diagnostics.deadlineMisses;
  snapshotMutex_.give();
}

pros::Rotation& Odometry::leftTrackingSensor() noexcept {
  return encoderLeft_;
}

pros::Rotation& Odometry::rightTrackingSensor() noexcept {
  return encoderRight_;
}

pros::Rotation& Odometry::backTrackingSensor() noexcept {
  return encoderBack_;
}

pros::Imu& Odometry::imuSensor() noexcept { return imu_; }

}  // namespace aon
