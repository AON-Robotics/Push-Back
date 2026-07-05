#pragma once

namespace aon::util {

/// Lightweight scalar Kalman filter used for noisy distance estimates.
///
/// This preserves the algorithm and defaults previously supplied by the
/// temporary Okapi compatibility layer.
class ScalarKalmanFilter {
 public:
  explicit ScalarKalmanFilter(double processNoise = 0.0001,
                              double measurementNoise = 0.01)
      : processNoise(processNoise), measurementNoise(measurementNoise) {}

  double filter(double measurement) {
    if (!initialized) {
      estimate = measurement;
      initialized = true;
      return estimate;
    }

    covariance += processNoise;
    const double gain = covariance / (covariance + measurementNoise);
    estimate += gain * (measurement - estimate);
    covariance *= 1.0 - gain;
    return estimate;
  }

 private:
  double processNoise;
  double measurementNoise;
  double estimate = 0.0;
  double covariance = 1.0;
  bool initialized = false;
};

}  // namespace aon::util
