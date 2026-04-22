#pragma once

#include <cstdint>
#include <cmath>
#include <limits>

/// Pose-only EKF for 2D localization.
/// Units:
///   - X, Y: inches
///   - theta: radians
/// Coordinate convention:
///   +X forward, -X backward
///   +Y right,   -Y left
///   +theta CCW
class EKF {
public:
  enum class Mode : uint8_t { Tank, HDrive };

  struct State {
    double x_in{0.0};
    double y_in{0.0};
    double theta_rad{0.0};
  };

  /// Snapshot of EKF internals for deterministic telemetry logging.
  struct DebugSnapshot {
    double x_pred_in{0.0};
    double y_pred_in{0.0};
    double theta_pred_rad{0.0};

    double Pxx_pred{0.0};
    double Pyy_pred{0.0};
    double Ptt_pred{0.0};

    double Pxx_est{0.0};
    double Pyy_est{0.0};
    double Ptt_est{0.0};

    double innovation_theta{std::numeric_limits<double>::quiet_NaN()};
    double innovation_x{std::numeric_limits<double>::quiet_NaN()};
    double innovation_y{std::numeric_limits<double>::quiet_NaN()};
    double NIS_theta{std::numeric_limits<double>::quiet_NaN()};

    double dx_update_in{0.0};
    double dy_update_in{0.0};
    double dtheta_update_rad{0.0};
  };

  /// Noise parameters for tuning.
  struct Noise {
    // Process noise (prediction uncertainty)
    double q_x{1e-3};
    double q_y{1e-3};
    double q_theta{1e-4};

    // Default measurement noise
    double r_gps_x{4.0};    // inches^2
    double r_gps_y{4.0};    // inches^2
    double r_theta{(2.0 * M_PI / 180.0) * (2.0 * M_PI / 180.0)}; // (2deg)^2 in rad^2
  };

  struct TankConfig {
    State x0{};
    Noise noise{};
  };

  struct HDriveConfig {
    State x0{};
    Noise noise{};
    // Lateral tracking wheel / sensor X offset from robot center, in inches.
    // Positive if the lateral wheel is forward (+X), negative if backward (-X).
    double lateral_wheel_x_offset_in{7.25};
  };

  /// Constructs a tank/unicycle EKF instance using the provided initial state and noise.
  explicit EKF(const TankConfig& cfg);

  /// Constructs an H-Drive EKF instance using the provided initial state, noise, and lateral wheel offset.
  explicit EKF(const HDriveConfig& cfg);

  /// Returns the drivetrain mode for this EKF instance.
  Mode mode() const { return mode_; }

  // ---- State access ----
  /// Returns the current EKF state estimate.
  State getState() const { return x_; }

  /// Sets the EKF state estimate and normalizes the heading.
  void  setState(const State& s);

  /// Returns the latest EKF debug snapshot (prediction + update telemetry).
  DebugSnapshot getDebugSnapshot() const { return debug_; }

  // Covariance P access (3x3)
  /// Copies the current covariance matrix P into outP.
  void getP(double outP[3][3]) const;

  /// Replaces the current covariance matrix P with inP.
  void setP(const double inP[3][3]);

  // ---- Prediction ----
  /// Runs the tank/unicycle prediction step using forward velocity and yaw rate over dt.
  void predictTank(double v_in_per_s, double omega_rad_per_s, double dt_s);

  /// Runs the holonomic prediction step using body-frame vx/vy and yaw rate over dt.
  void predictHolonomic(double vx_in_per_s, double vy_in_per_s, double omega_rad_per_s, double dt_s);

  /// Compensates lateral wheel velocity for X-offset-induced rotation contribution.
  double compensateLateralVy(double vy_measured_in_per_s, double omega_rad_per_s) const;

  // ---- Measurement updates ----
  /// Performs a GPS position update using x/y measurements and optional per-call noise.
  void updateGPS(double x_gps_in, double y_gps_in, double r_x_in2 = -1.0, double r_y_in2 = -1.0);

  /// Performs a heading update using an absolute theta measurement and optional per-call noise.
  void updateHeading(double theta_meas_rad, double r_theta_rad2 = -1.0);

  // ---- Noise configuration ----
  /// Sets the diagonal process noise parameters used by the prediction step.
  void setProcessNoise(double qx, double qy, double qtheta);

  /// Sets default measurement noise parameters used when per-call noise is not provided.
  void setDefaultMeasurementNoise(double r_gps_x, double r_gps_y, double r_theta);

  /// Enables or disables GPS-driven heading correction during the GPS update.
  void setGPSUpdatesTheta(bool enable);

  // ---- Utility ----
  /// Normalizes an angle to the range [-pi, pi].
  static double normalizeAngle(double a_rad);

  /// Computes wrapped angle difference (a - b) in the range [-pi, pi).
  static double angleDiff(double a_rad, double b_rad);

private:
  // Internal helpers
  /// Adds diagonal process noise to the covariance scaled by dt.
  void addProcessNoiseScaled(double dt_s);

  /// Propagates covariance using P = F P F^T.
  void propagateCovariance(const double F[3][3]);

  /// Enforces covariance symmetry by averaging P with its transpose.
  void symmetrizeP();

  /// Clamps covariance diagonal entries to be >= eps.
  void clampPDiag(double eps);

  /// Resets per-tick debug values at the start of a prediction step.
  void resetDebugForPredict();

  /// Updates post-estimate debug values from current state and covariance.
  void updateDebugPostEstimate();

  // EKF generic update for:
  //   z dimension 1: theta
  //   z dimension 2: gps xy
  /// Performs a generic 1D Joseph-form EKF update (used for heading).
  void update1D(double z, double h, const double H[1][3], double R);

  /// Performs a generic 2D Joseph-form EKF update (used for GPS xy).
  void update2D(const double z[2], const double h[2], const double H[2][3], const double R[2][2]);

  /// Multiplies two 3x3 matrices.
  static void mat3_mul(const double A[3][3], const double B[3][3], double C[3][3]);

  /// Transposes a 3x3 matrix.
  static void mat3_transpose(const double A[3][3], double AT[3][3]);

  /// Adds two 3x3 matrices.
  static void mat3_add(const double A[3][3], const double B[3][3], double C[3][3]);

  /// Subtracts two 3x3 matrices.
  static void mat3_sub(const double A[3][3], const double B[3][3], double C[3][3]);

  /// Writes the 3x3 identity matrix into I.
  static void mat3_identity(double I[3][3]);

  /// Multiplies a 3x3 matrix by a 3x1 vector.
  static void mat3_vec_mul(const double A[3][3], const double v[3], double out[3]);

  /// Inverts a 2x2 matrix; returns false if singular.
  static bool inv2x2(const double A[2][2], double invA[2][2], double* det_out = nullptr);

private:
  Mode mode_{Mode::Tank};
  Noise noise_{};
  State x_{};

  // Covariance P (3x3)
  double P_[3][3]{};

  // H-Drive specifics
  double lateral_x_offset_in_{0.0};

  // GPS-to-heading correction policy
  bool gps_updates_theta_{false};

  // EKF telemetry snapshot for external logging.
  DebugSnapshot debug_{};
};
