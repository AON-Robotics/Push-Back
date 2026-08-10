#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "aon/odometry/ekf.hpp"
#include "aon/odometry/pose-estimator.hpp"
#include "aon/odometry/sensor-measurements.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

constexpr double kTolerance = 1e-9;

void checkNear(double actual, double expected,
               double tolerance = kTolerance) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

void angleConversionsAndWrappingAreConsistent() {
  using namespace aon::localization;

  checkNear(wrapRadians(radians(359.0)), radians(-1.0));
  checkNear(shortestAngleDelta(radians(359.0), radians(1.0)),
            radians(2.0));
  checkNear(shortestAngleDelta(radians(-179.0), radians(179.0)),
            radians(-2.0));
  checkNear(degrees(radians(90.0)), 90.0);
}

void sincIsStableNearZero() {
  using namespace aon::localization;

  checkNear(sinc(0.0), 1.0);
  checkNear(sinc(1e-9), 1.0, 1e-12);
}

void measurementTypesCarryValuesAndValidity() {
  using namespace aon::localization;

  const WheelDistances wheels{1.0, 2.0, 3.0, true, true, true};
  const ImuMeasurement imu{radians(10.0), true};
  const GpsMeasurement gps{12.0, 24.0, radians(90.0), 1.5,
                           true, true, true, 100U};

  CHECK(wheels.leftInches == 1.0);
  CHECK(wheels.backValid && imu.valid && gps.fresh);
  CHECK(gps.timestampMs == 100U);
}

void wheelRecoveryRebaselinesWithoutAccumulatedMotion() {
  using namespace aon::localization;

  WheelDistances baselines{10.0, 10.0, 2.0, true, true, true};
  WheelDistances deltas = consumeWheelDistances(
      {0.0, 11.0, 2.0, false, true, true}, baselines);
  CHECK(!deltas.leftValid);
  CHECK(deltas.rightValid);
  checkNear(deltas.rightInches, 1.0);
  CHECK(!baselines.leftValid);

  deltas = consumeWheelDistances(
      {12.0, 12.0, 2.0, true, true, true}, baselines);
  CHECK(!deltas.leftValid);
  CHECK(deltas.rightValid);
  checkNear(deltas.rightInches, 1.0);
  CHECK(baselines.leftValid);

  deltas = consumeWheelDistances(
      {13.0, 13.0, 2.0, true, true, true}, baselines);
  CHECK(deltas.leftValid && deltas.rightValid);
  checkNear(deltas.leftInches, 1.0);
  checkNear(deltas.rightInches, 1.0);
}

void checkMotion(const aon::localization::LocalMotion& actual,
                 double expectedRight, double expectedForward,
                 double expectedHeading) {
  checkNear(actual.rightInches, expectedRight);
  checkNear(actual.forwardInches, expectedForward);
  checkNear(actual.headingRadians, expectedHeading);
}

void checkPose(const aon::localization::EstimatorPose& actual,
               double expectedX, double expectedY, double expectedHeading) {
  checkNear(actual.xInches, expectedX);
  checkNear(actual.yInches, expectedY);
  checkNear(actual.headingRadians, expectedHeading);
}

void threeWheelMotionUsesSignedOffsets() {
  using namespace aon::localization;

  const TrackingGeometry geometry{-4.0, 4.0, -3.0};
  checkMotion(localMotion({0.0, 0.0, 0.0, true, true, true}, geometry),
              0.0, 0.0, 0.0);
  checkMotion(localMotion({12.0, 12.0, 0.0, true, true, true}, geometry),
              0.0, 12.0, 0.0);
  checkMotion(localMotion({-12.0, -12.0, 0.0, true, true, true}, geometry),
              0.0, -12.0, 0.0);

  const double quarterTurn = kPi / 2.0;
  checkMotion(localMotion({4.0 * quarterTurn, -4.0 * quarterTurn,
                           -3.0 * quarterTurn, true, true, true},
                          geometry),
              0.0, 0.0, quarterTurn);
  checkMotion(localMotion({0.0, 0.0, 5.0, true, true, true}, geometry),
              5.0, 0.0, 0.0);

  const LocalMotion withoutBack =
      localMotion({3.0, 3.0, 100.0, true, true, false}, geometry);
  checkMotion(withoutBack, 0.0, 3.0, 0.0);
  CHECK(!withoutBack.lateralValid);

  const LocalMotion invalidGeometry =
      localMotion({1.0, 1.0, 0.0, true, true, true},
                  {2.0, 2.0, 1.0});
  CHECK(!std::isfinite(invalidGeometry.headingRadians));
}

void posePropagationMatchesTheFieldConvention() {
  using namespace aon::localization;

  checkPose(propagatePose({0.0, 0.0, 0.0},
                          {0.0, 12.0, 0.0, true}),
            0.0, 12.0, 0.0);
  checkPose(propagatePose({0.0, 0.0, kPi / 2.0},
                          {0.0, 12.0, 0.0, true}),
            12.0, 0.0, kPi / 2.0);
  checkPose(propagatePose({0.0, 0.0, 0.0},
                          {6.0, 0.0, 0.0, true}),
            6.0, 0.0, 0.0);

  constexpr double radius = 10.0;
  const double quarterTurn = kPi / 2.0;
  checkPose(propagatePose({0.0, 0.0, 0.0},
                          {0.0, radius * quarterTurn, quarterTurn, true}),
            radius, radius, quarterTurn);

  const LocalMotion combined{2.0, 6.0, kPi / 3.0, true};
  const double halfTurn = combined.headingRadians / 2.0;
  const double scale = sinc(halfTurn);
  checkPose(propagatePose({0.0, 0.0, 0.0}, combined),
            scale * (combined.rightInches * std::cos(halfTurn) +
                     combined.forwardInches * std::sin(halfTurn)),
            scale * (-combined.rightInches * std::sin(halfTurn) +
                     combined.forwardInches * std::cos(halfTurn)),
            combined.headingRadians);
}

double square(double value) { return value * value; }

aon::localization::EkfConfig testEkfConfig() {
  using namespace aon::localization;
  return {4.0,
          square(radians(5.0)),
          1e-6,
          1e-8,
          0.01,
          0.01,
          square(radians(2.0)),
          4.0,
          square(radians(8.0)),
          1e-12};
}

void ekfPredictionPropagatesPoseAndCovariance() {
  using namespace aon::localization;

  const EkfConfig config = testEkfConfig();
  Ekf ekf(config);
  ekf.reset({2.0, 3.0, radians(30.0)});
  checkPose(ekf.pose(), 2.0, 3.0, radians(30.0));

  CovarianceDiagonal diagonal = ekf.covarianceDiagonal();
  checkNear(diagonal.xVariance, config.initialPositionVariance);
  checkNear(diagonal.yVariance, config.initialPositionVariance);
  checkNear(diagonal.headingVariance, config.initialHeadingVariance);

  CHECK(ekf.predict({0.0, 0.0, 0.0, true}));
  checkPose(ekf.pose(), 2.0, 3.0, radians(30.0));
  diagonal = ekf.covarianceDiagonal();
  checkNear(diagonal.xVariance,
            config.initialPositionVariance +
                config.stationaryPositionVariance);
  checkNear(diagonal.yVariance,
            config.initialPositionVariance +
                config.stationaryPositionVariance);
  checkNear(diagonal.headingVariance,
            config.initialHeadingVariance +
                config.stationaryHeadingVariance);

  ekf.reset({0.0, 0.0, 0.0});
  CHECK(ekf.predict({0.0, 12.0, 0.0, true}));
  checkPose(ekf.pose(), 0.0, 12.0, 0.0);

  const LocalMotion combined{2.0, 6.0, radians(20.0), true};
  const EstimatorPose expected = propagatePose(ekf.pose(), combined);
  CHECK(ekf.predict(combined));
  checkPose(ekf.pose(), expected.xInches, expected.yInches,
            expected.headingRadians);
}

void ekfPredictionStaysFiniteAndRejectsBadInput() {
  using namespace aon::localization;

  Ekf ekf(testEkfConfig());
  ekf.reset({0.0, 0.0, 0.0});
  for (int index = 0; index < 10000; ++index) {
    const double turn = index % 2 == 0 ? 0.002 : -0.002;
    CHECK(ekf.predict({0.001, 0.01, turn, true}));
  }

  const Matrix3 covariance = ekf.covariance();
  for (std::size_t row = 0; row < covariance.size(); ++row) {
    CHECK(std::isfinite(covariance[row][row]));
    CHECK(covariance[row][row] >= 0.0);
    for (std::size_t column = 0; column < covariance.size(); ++column) {
      CHECK(std::isfinite(covariance[row][column]));
      checkNear(covariance[row][column], covariance[column][row], 1e-8);
    }
  }

  const EstimatorPose beforePose = ekf.pose();
  const Matrix3 beforeCovariance = ekf.covariance();
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  CHECK(!ekf.predict({0.0, invalid, 0.0, false}));
  checkPose(ekf.pose(), beforePose.xInches, beforePose.yInches,
            beforePose.headingRadians);
  const Matrix3 afterCovariance = ekf.covariance();
  for (std::size_t row = 0; row < beforeCovariance.size(); ++row) {
    for (std::size_t column = 0; column < beforeCovariance.size(); ++column) {
      checkNear(afterCovariance[row][column], beforeCovariance[row][column]);
    }
  }
}

void imuUpdatesUseWrappedInnovationsAndConfiguredNoise() {
  using namespace aon::localization;

  Ekf ekf(testEkfConfig());
  ekf.reset({0.0, 0.0, radians(359.0)});
  const double before359 = ekf.pose().headingRadians;
  CHECK(ekf.updateImuHeading(radians(1.0)));
  const double correction359 =
      shortestAngleDelta(before359, ekf.pose().headingRadians);
  CHECK(correction359 > 0.0);
  CHECK(correction359 < radians(2.0));

  ekf.reset({0.0, 0.0, radians(-179.0)});
  const double beforeNegative = ekf.pose().headingRadians;
  CHECK(ekf.updateImuHeading(radians(179.0)));
  const double correctionNegative =
      shortestAngleDelta(beforeNegative, ekf.pose().headingRadians);
  CHECK(correctionNegative < 0.0);
  CHECK(correctionNegative > radians(-2.0));

  EkfConfig lowNoiseConfig = testEkfConfig();
  lowNoiseConfig.imuHeadingVariance = square(radians(0.1));
  EkfConfig highNoiseConfig = testEkfConfig();
  highNoiseConfig.imuHeadingVariance = square(radians(100.0));
  Ekf lowNoise(lowNoiseConfig);
  Ekf highNoise(highNoiseConfig);
  lowNoise.reset({0.0, 0.0, 0.0});
  highNoise.reset({0.0, 0.0, 0.0});
  CHECK(lowNoise.updateImuHeading(radians(30.0)));
  CHECK(highNoise.updateImuHeading(radians(30.0)));
  CHECK(std::abs(lowNoise.pose().headingRadians) >
        std::abs(highNoise.pose().headingRadians));
}

void imuJosephUpdateIsStableAndRejectsInvalidMeasurements() {
  using namespace aon::localization;

  Ekf ekf(testEkfConfig());
  ekf.reset({1.0, 2.0, radians(10.0)});
  const double initialVariance = ekf.covarianceDiagonal().headingVariance;
  for (int index = 0; index < 1000; ++index) {
    CHECK(ekf.updateImuHeading(radians(10.0)));
  }
  const CovarianceDiagonal diagonal = ekf.covarianceDiagonal();
  CHECK(std::isfinite(diagonal.headingVariance));
  CHECK(diagonal.headingVariance >= 0.0);
  CHECK(diagonal.headingVariance < initialVariance);

  const EstimatorPose beforePose = ekf.pose();
  const Matrix3 beforeCovariance = ekf.covariance();
  CHECK(!ekf.updateImuHeading(
      std::numeric_limits<double>::quiet_NaN()));
  checkPose(ekf.pose(), beforePose.xInches, beforePose.yInches,
            beforePose.headingRadians);
  CHECK(ekf.covariance() == beforeCovariance);

  EkfConfig singularConfig = testEkfConfig();
  singularConfig.initialHeadingVariance = 0.0;
  singularConfig.imuHeadingVariance = 0.0;
  Ekf singular(singularConfig);
  singular.reset({0.0, 0.0, 0.0});
  CHECK(!singular.updateImuHeading(0.0));
  checkPose(singular.pose(), 0.0, 0.0, 0.0);
}

aon::localization::GpsValidationConfig testGpsGateConfig() {
  using namespace aon::localization;
  return {-72.0, 72.0, -72.0, 72.0, 6.0, 18.0, radians(45.0),
          50U, 9.21, 6.63};
}

aon::localization::GpsMeasurement gpsSample(
    double x, double y, double heading, std::uint32_t timestamp) {
  return {x, y, heading, 1.0, true, true, true, timestamp};
}

void gpsGateRejectsBadSamplesWithTypedReasons() {
  using namespace aon::localization;

  const GpsValidationConfig config = testGpsGateConfig();
  {
    GpsGate gate(config);
    GpsMeasurement sample = gpsSample(0.0, 0.0, 0.0, 100U);
    sample.xInches = std::numeric_limits<double>::quiet_NaN();
    CHECK(gate.evaluate(sample).reason == GpsRejectionReason::NonFinite);
  }
  {
    GpsGate gate(config);
    GpsMeasurement sample = gpsSample(0.0, 0.0, 0.0, 100U);
    sample.fresh = false;
    CHECK(gate.evaluate(sample).reason == GpsRejectionReason::NotFresh);
  }
  {
    GpsGate gate(config);
    const GpsMeasurement first = gpsSample(0.0, 0.0, 0.0, 100U);
    CHECK(gate.evaluate(first).positionAccepted);
    gate.commit(first, true, true);
    CHECK(gate.evaluate(gpsSample(1.0, 1.0, 0.0, 120U)).reason ==
          GpsRejectionReason::StaleTimestamp);
  }
  {
    GpsGate gate(config);
    GpsMeasurement sample = gpsSample(0.0, 0.0, 0.0, 100U);
    sample.positionErrorInches = 7.0;
    CHECK(gate.evaluate(sample).reason ==
          GpsRejectionReason::ExcessiveReportedError);
  }
  {
    GpsGate gate(config);
    CHECK(gate.evaluate(gpsSample(80.0, 0.0, 0.0, 100U)).reason ==
          GpsRejectionReason::OutsideFieldBounds);
  }
  {
    GpsGate gate(config);
    const GpsMeasurement first = gpsSample(0.0, 0.0, 0.0, 100U);
    CHECK(gate.evaluate(first).positionAccepted);
    gate.commit(first, true, true);
    CHECK(gate.evaluate(gpsSample(30.0, 0.0, 0.0, 160U)).reason ==
          GpsRejectionReason::PositionJump);
  }
  {
    GpsGate gate(config);
    const GpsMeasurement first = gpsSample(0.0, 0.0, 0.0, 100U);
    CHECK(gate.evaluate(first).headingAccepted);
    gate.commit(first, true, true);
    const GpsGateResult result =
        gate.evaluate(gpsSample(1.0, 0.0, radians(90.0), 160U));
    CHECK(result.positionAccepted);
    CHECK(!result.headingAccepted);
    CHECK(result.reason == GpsRejectionReason::HeadingJump);
  }
}

void gpsGateRecoversAfterTemporaryLoss() {
  using namespace aon::localization;

  GpsGate gate(testGpsGateConfig());
  const GpsMeasurement first = gpsSample(0.0, 0.0, 0.0, 100U);
  CHECK(gate.evaluate(first).positionAccepted);
  gate.commit(first, true, true);
  GpsMeasurement missing = gpsSample(1.0, 0.0, 0.0, 160U);
  missing.positionValid = false;
  CHECK(gate.evaluate(missing).reason ==
        GpsRejectionReason::InvalidPosition);
  const GpsGateResult recovered =
      gate.evaluate(gpsSample(2.0, 0.0, 0.0, 220U));
  CHECK(recovered.positionAccepted);
  CHECK(recovered.headingAccepted);
}

void gpsGateOnlyCommitsFilterAcceptedSamples() {
  using namespace aon::localization;

  GpsGate gate(testGpsGateConfig());
  const GpsMeasurement origin = gpsSample(0.0, 0.0, 0.0, 100U);
  const GpsGateResult originResult = gate.evaluate(origin);
  CHECK(originResult.positionAccepted);
  gate.commit(origin, true, true);

  const GpsMeasurement rejected = gpsSample(15.0, 0.0, 0.0, 160U);
  CHECK(gate.evaluate(rejected).positionAccepted);
  gate.commit(rejected, false, false);

  const GpsMeasurement recovered = gpsSample(1.0, 0.0, 0.0, 220U);
  CHECK(gate.evaluate(recovered).positionAccepted);
}

void gpsFreshnessRequiresAChangedHardwareObservation() {
  using namespace aon::localization;

  GpsFreshnessTracker tracker;
  GpsMeasurement first = gpsSample(1.0, 2.0, radians(5.0), 0U);
  CHECK(tracker.observe(first, 100U));
  CHECK(first.fresh && first.timestampMs == 100U);

  GpsMeasurement duplicate = gpsSample(1.0, 2.0, radians(5.0), 0U);
  CHECK(!tracker.observe(duplicate, 110U));
  CHECK(!duplicate.fresh && duplicate.timestampMs == 100U);

  GpsMeasurement changed = gpsSample(1.01, 2.0, radians(5.0), 0U);
  CHECK(tracker.observe(changed, 120U));
  CHECK(changed.fresh && changed.timestampMs == 120U);
}

void gpsPositionCorrectionIsWeightedAndOutlierGated() {
  using namespace aon::localization;

  Ekf ekf(testEkfConfig());
  ekf.reset({10.0, -8.0, 0.0});
  const CovarianceDiagonal before = ekf.covarianceDiagonal();
  CHECK(ekf.updateGpsPosition(0.0, 0.0, 1000.0));
  const EstimatorPose corrected = ekf.pose();
  CHECK(corrected.xInches > 0.0 && corrected.xInches < 10.0);
  CHECK(corrected.yInches < 0.0 && corrected.yInches > -8.0);
  const CovarianceDiagonal after = ekf.covarianceDiagonal();
  CHECK(after.xVariance < before.xVariance);
  CHECK(after.yVariance < before.yVariance);

  ekf.reset({0.0, 0.0, 0.0});
  const EstimatorPose beforeOutlier = ekf.pose();
  CHECK(!ekf.updateGpsPosition(60.0, 60.0, 9.21));
  checkPose(ekf.pose(), beforeOutlier.xInches, beforeOutlier.yInches,
            beforeOutlier.headingRadians);

  EkfConfig singularConfig = testEkfConfig();
  singularConfig.initialPositionVariance = 0.0;
  singularConfig.gpsPositionVariance = 0.0;
  Ekf singular(singularConfig);
  CHECK(!singular.updateGpsPosition(0.0, 0.0, 9.21));
}

void gpsHeadingCorrectionWrapsAndCanRemainDisabled() {
  using namespace aon::localization;

  Ekf ekf(testEkfConfig());
  ekf.reset({0.0, 0.0, radians(359.0)});
  const double before = ekf.pose().headingRadians;
  CHECK(ekf.updateGpsHeading(radians(1.0), 6.63, true));
  CHECK(shortestAngleDelta(before, ekf.pose().headingRadians) > 0.0);

  ekf.reset({0.0, 0.0, radians(25.0)});
  CHECK(!ekf.updateGpsHeading(radians(90.0), 6.63, false));
  checkNear(ekf.pose().headingRadians, radians(25.0));

  const EstimatorPose beforeOutlier = ekf.pose();
  CHECK(!ekf.updateGpsHeading(radians(170.0), 6.63, true));
  checkPose(ekf.pose(), beforeOutlier.xInches, beforeOutlier.yInches,
            beforeOutlier.headingRadians);
}

}  // namespace

int main() {
  angleConversionsAndWrappingAreConsistent();
  sincIsStableNearZero();
  measurementTypesCarryValuesAndValidity();
  wheelRecoveryRebaselinesWithoutAccumulatedMotion();
  threeWheelMotionUsesSignedOffsets();
  posePropagationMatchesTheFieldConvention();
  ekfPredictionPropagatesPoseAndCovariance();
  ekfPredictionStaysFiniteAndRejectsBadInput();
  imuUpdatesUseWrappedInnovationsAndConfiguredNoise();
  imuJosephUpdateIsStableAndRejectsInvalidMeasurements();
  gpsGateRejectsBadSamplesWithTypedReasons();
  gpsGateRecoversAfterTemporaryLoss();
  gpsGateOnlyCommitsFilterAcceptedSamples();
  gpsFreshnessRequiresAChangedHardwareObservation();
  gpsPositionCorrectionIsWeightedAndOutlierGated();
  gpsHeadingCorrectionWrapsAndCanRemainDisabled();
  std::cout << "localization math tests passed\n";
  return 0;
}
