// EKF.cpp

#include "../../../include/EKF/EKF.hpp"
#include <limits>

namespace {
double quietNaN() {
  return std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

// ---------------- Constructors ----------------
EKF::EKF(const TankConfig& cfg)
: mode_(Mode::Tank), noise_(cfg.noise), x_(cfg.x0), lateral_x_offset_in_(0.0), gps_updates_theta_(false) {
  mat3_identity(P_);
  P_[0][0] = 25.0;   // 5 in std
  P_[1][1] = 25.0;   // 5 in std
  P_[2][2] = (10.0 * M_PI / 180.0) * (10.0 * M_PI / 180.0); // 10deg std
}

EKF::EKF(const HDriveConfig& cfg)
: mode_(Mode::HDrive), noise_(cfg.noise), x_(cfg.x0), lateral_x_offset_in_(cfg.lateral_wheel_x_offset_in), gps_updates_theta_(false) {
  mat3_identity(P_);
  P_[0][0] = 25.0;
  P_[1][1] = 25.0;
  P_[2][2] = (10.0 * M_PI / 180.0) * (10.0 * M_PI / 180.0);
}

// ---------------- State / Covariance ----------------
void EKF::setState(const State& s) {
  x_ = s;
  x_.theta_rad = normalizeAngle(x_.theta_rad);
}

void EKF::getP(double outP[3][3]) const {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      outP[r][c] = P_[r][c];
}

void EKF::setP(const double inP[3][3]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      P_[r][c] = inP[r][c];
  symmetrizeP();
  clampPDiag(1e-12);
}

// ---------------- Noise setters ----------------

void EKF::setProcessNoise(double qx, double qy, double qtheta) {
  noise_.q_x = qx;
  noise_.q_y = qy;
  noise_.q_theta = qtheta;
}

void EKF::setDefaultMeasurementNoise(double r_gps_x, double r_gps_y, double r_theta) {
  noise_.r_gps_x = r_gps_x;
  noise_.r_gps_y = r_gps_y;
  noise_.r_theta = r_theta;
}

void EKF::setGPSUpdatesTheta(bool enable) {
  gps_updates_theta_ = enable;
}

// ---------------- Utility ----------------

double EKF::normalizeAngle(double a_rad) {
  while (a_rad > M_PI)  a_rad -= 2.0 * M_PI;
  while (a_rad < -M_PI) a_rad += 2.0 * M_PI;
  return a_rad;
}

double EKF::angleDiff(double a_rad, double b_rad) {
  return normalizeAngle(a_rad - b_rad);
}

// ---------------- Prediction ----------------
double EKF::compensateLateralVy(double vy_measured_in_per_s, double omega_rad_per_s) const {
  // vy_center = vy_measured - omega * x_offset
  // x_offset is inches, omega rad/s => inches/s contribution
  return vy_measured_in_per_s - omega_rad_per_s * lateral_x_offset_in_;
}

void EKF::predictTank(double v_in_per_s, double omega_rad_per_s, double dt_s) {
  resetDebugForPredict();

  // Unicycle/tank model:
  // X += v*cos(theta)*dt
  // Y += v*sin(theta)*dt   
  // theta += omega*dt
  const double th = x_.theta_rad;
  const double c = std::cos(th);
  const double s = std::sin(th);

  x_.x_in += (v_in_per_s * c) * dt_s;
  x_.y_in += (v_in_per_s * s) * dt_s;
  x_.theta_rad = normalizeAngle(x_.theta_rad + omega_rad_per_s * dt_s);

  // Jacobian F = d f / d x
  double F[3][3];
  mat3_identity(F);

  // dX/dtheta = -v*sin(theta)*dt
  // dY/dtheta =  v*cos(theta)*dt
  F[0][2] = -v_in_per_s * s * dt_s;
  F[1][2] =  v_in_per_s * c * dt_s;

  propagateCovariance(F);
  addProcessNoiseScaled(dt_s);

  debug_.x_pred_in = x_.x_in;
  debug_.y_pred_in = x_.y_in;
  debug_.theta_pred_rad = x_.theta_rad;
  debug_.Pxx_pred = P_[0][0];
  debug_.Pyy_pred = P_[1][1];
  debug_.Ptt_pred = P_[2][2];
  updateDebugPostEstimate();
}

void EKF::predictHolonomic(double vx_in_per_s, double vy_in_per_s, double omega_rad_per_s, double dt_s) {
  resetDebugForPredict();

  // Holonomic model (H-Drive) with body-frame velocities:
  // X += ( vx*cos(theta) - vy*sin(theta) ) * dt
  // Y += ( vx*sin(theta) + vy*cos(theta) ) * dt
  // theta += omega*dt
  //
  // vy is +right in this convention. This formula is consistent as long as theta is defined CCW from +X.
  const double th = x_.theta_rad;
  const double c = std::cos(th);
  const double s = std::sin(th);

  const double dX = (vx_in_per_s * c - vy_in_per_s * s) * dt_s;
  const double dY = (vx_in_per_s * s + vy_in_per_s * c) * dt_s;

  x_.x_in += dX;
  x_.y_in += dY;
  x_.theta_rad = normalizeAngle(x_.theta_rad + omega_rad_per_s * dt_s);

  // Jacobian F = d f / d x
  double F[3][3];
  mat3_identity(F);

  // dX/dtheta = (-vx*sin(theta) - vy*cos(theta)) * dt
  // dY/dtheta = ( vx*cos(theta) - vy*sin(theta)) * dt
  F[0][2] = (-vx_in_per_s * s - vy_in_per_s * c) * dt_s;
  F[1][2] = ( vx_in_per_s * c - vy_in_per_s * s) * dt_s;

  propagateCovariance(F);
  addProcessNoiseScaled(dt_s);

  debug_.x_pred_in = x_.x_in;
  debug_.y_pred_in = x_.y_in;
  debug_.theta_pred_rad = x_.theta_rad;
  debug_.Pxx_pred = P_[0][0];
  debug_.Pyy_pred = P_[1][1];
  debug_.Ptt_pred = P_[2][2];
  updateDebugPostEstimate();
}

void EKF::propagateCovariance(const double F[3][3]) {
  // P = F P F^T
  double FP[3][3];
  double FT[3][3];
  double FPFt[3][3];

  mat3_mul(F, P_, FP);
  mat3_transpose(F, FT);
  mat3_mul(FP, FT, FPFt);

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      P_[r][c] = FPFt[r][c];

  symmetrizeP();
  clampPDiag(1e-12);
}

void EKF::addProcessNoiseScaled(double dt_s) {
  // Simple diagonal Q scaled by dt.
  // You can change scaling to dt^2 if you prefer, dt scaling is a pragmatic VEX choice.
  P_[0][0] += noise_.q_x * dt_s;
  P_[1][1] += noise_.q_y * dt_s;
  P_[2][2] += noise_.q_theta * dt_s;

  clampPDiag(1e-12);
}

// ---------------- Measurement Updates ----------------
void EKF::updateGPS(double x_gps_in, double y_gps_in, double r_x_in2, double r_y_in2) {
  const double Rxx = (r_x_in2 > 0.0) ? r_x_in2 : noise_.r_gps_x;
  const double Ryy = (r_y_in2 > 0.0) ? r_y_in2 : noise_.r_gps_y;

  const double z[2] = { x_gps_in, y_gps_in };
  const double h[2] = { x_.x_in,  x_.y_in  };
  debug_.innovation_x = z[0] - h[0];
  debug_.innovation_y = z[1] - h[1];

  // H = [ [1 0 0],
  //       [0 1 0] ]
  const double H[2][3] = {
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0}
  };

  const double R[2][2] = {
    {Rxx, 0.0},
    {0.0, Ryy}
  };

  update2D(z, h, H, R);
  updateDebugPostEstimate();
}

void EKF::updateHeading(double theta_meas_rad, double r_theta_rad2) {
  const double R = (r_theta_rad2 > 0.0) ? r_theta_rad2 : noise_.r_theta;

  const double z = normalizeAngle(theta_meas_rad);
  const double h = x_.theta_rad;
  const double innovation = angleDiff(z, h);
  const double S = P_[2][2] + R;
  debug_.innovation_theta = innovation;
  debug_.NIS_theta = (S > 1e-12) ? ((innovation * innovation) / S) : quietNaN();

  const double H[1][3] = { 0.0, 0.0, 1.0 };

  // residual must be angle-wrapped
  update1D(z, h, H, R);
  updateDebugPostEstimate();
}

// Generic 1D update (theta)
void EKF::update1D(double z, double h, const double H[1][3], double R) {
  // r = z - h (wrapped for angles)
  double r = angleDiff(z, h);

  // S = H P H^T + R  => scalar
  double S = 0.0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      S += H[0][i] * P_[i][j] * H[0][j];
    }
  }
  S += R;

  if (S < 1e-12) return;

  // K = P H^T S^-1  => 3x1
  double K[3];
  for (int i = 0; i < 3; ++i) {
    double PHt = 0.0;
    for (int j = 0; j < 3; ++j) {
      PHt += P_[i][j] * H[0][j];
    }
    K[i] = PHt / S;
  }

  // x = x + K r
  x_.x_in     += K[0] * r;
  x_.y_in     += K[1] * r;
  x_.theta_rad = normalizeAngle(x_.theta_rad + K[2] * r);

  // Joseph form covariance update:
  // P = (I - K H) P (I - K H)^T + K R K^T
  double I[3][3]; mat3_identity(I);

  double KH[3][3] = {};
  for (int r_i = 0; r_i < 3; ++r_i) {
    for (int c_i = 0; c_i < 3; ++c_i) {
      KH[r_i][c_i] = K[r_i] * H[0][c_i];
    }
  }

  double IminusKH[3][3];
  mat3_sub(I, KH, IminusKH);

  double tmp[3][3];
  double IminusKH_T[3][3];
  mat3_transpose(IminusKH, IminusKH_T);

  mat3_mul(IminusKH, P_, tmp);
  double newP[3][3];
  mat3_mul(tmp, IminusKH_T, newP);

  // Add K R K^T
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      newP[i][j] += K[i] * R * K[j];
    }
  }

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      P_[i][j] = newP[i][j];

  symmetrizeP();
  clampPDiag(1e-12);
}

// Generic 2D update (GPS)
void EKF::update2D(const double z[2], const double h[2], const double H[2][3], const double R[2][2]) {
  // residual r = z - h
  double r[2] = { z[0] - h[0], z[1] - h[1] };

  // S = H P H^T + R   (2x2)
  double S[2][2] = { {0,0}, {0,0} };

  for (int a = 0; a < 2; ++a) {
    for (int b = 0; b < 2; ++b) {
      double sum = 0.0;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          sum += H[a][i] * P_[i][j] * H[b][j];
        }
      }
      S[a][b] = sum + R[a][b];
    }
  }

  // inv(S)
  double invS[2][2];
  double detS = 0.0;
  if (!inv2x2(S, invS, &detS)) {
    // If S is singular (shouldn't happen with sane R), skip update safely.
    return;
  }
  if (std::fabs(detS) < 1e-12) return;

  // K = P H^T inv(S)  => (3x2)
  // First compute PHt = P H^T  => (3x2)
  double PHt[3][2] = {};
  for (int i = 0; i < 3; ++i) {
    for (int a = 0; a < 2; ++a) {
      double sum = 0.0;
      for (int j = 0; j < 3; ++j) {
        sum += P_[i][j] * H[a][j];
      }
      PHt[i][a] = sum;
    }
  }

  // K = PHt * invS
  double K[3][2] = {};
  for (int i = 0; i < 3; ++i) {
    for (int a = 0; a < 2; ++a) {
      K[i][a] = PHt[i][0] * invS[0][a] + PHt[i][1] * invS[1][a];
    }
  }

  if (!gps_updates_theta_) {
    K[2][0] = 0.0;
    K[2][1] = 0.0;
  }

  // x = x + K r
  x_.x_in += K[0][0] * r[0] + K[0][1] * r[1];
  x_.y_in += K[1][0] * r[0] + K[1][1] * r[1];
  x_.theta_rad = normalizeAngle(x_.theta_rad + (K[2][0] * r[0] + K[2][1] * r[1]));

  // Joseph form:
  // P = (I - K H) P (I - K H)^T + K R K^T
  double I[3][3]; mat3_identity(I);

  // KH = K(3x2) * H(2x3) = 3x3
  double KH[3][3] = {};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      KH[i][j] = K[i][0] * H[0][j] + K[i][1] * H[1][j];
    }
  }

  double IminusKH[3][3];
  mat3_sub(I, KH, IminusKH);

  double tmp[3][3];
  double IminusKH_T[3][3];
  mat3_transpose(IminusKH, IminusKH_T);

  mat3_mul(IminusKH, P_, tmp);
  double newP[3][3];
  mat3_mul(tmp, IminusKH_T, newP);

  // Add K R K^T  (3x2)*(2x2)*(2x3)
  // Compute KR = K * R  => 3x2
  double KRm[3][2] = {};
  for (int i = 0; i < 3; ++i) {
    KRm[i][0] = K[i][0] * R[0][0] + K[i][1] * R[1][0];
    KRm[i][1] = K[i][0] * R[0][1] + K[i][1] * R[1][1];
  }
  // newP += KR * K^T
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      newP[i][j] += KRm[i][0] * K[j][0] + KRm[i][1] * K[j][1];
    }
  }

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      P_[i][j] = newP[i][j];

  symmetrizeP();
  clampPDiag(1e-12);
}

// ---------------- Small matrix helpers ----------------

void EKF::symmetrizeP() {
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      const double a = 0.5 * (P_[i][j] + P_[j][i]);
      P_[i][j] = a;
      P_[j][i] = a;
    }
  }
}

void EKF::clampPDiag(double eps) {
  for (int i = 0; i < 3; ++i) {
    if (P_[i][i] < eps) P_[i][i] = eps;
  }
}

void EKF::mat3_identity(double I[3][3]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      I[r][c] = (r == c) ? 1.0 : 0.0;
}

void EKF::mat3_transpose(const double A[3][3], double AT[3][3]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      AT[c][r] = A[r][c];
}

void EKF::mat3_mul(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) sum += A[r][k] * B[k][c];
      C[r][c] = sum;
    }
  }
}

void EKF::mat3_add(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      C[r][c] = A[r][c] + B[r][c];
}

void EKF::mat3_sub(const double A[3][3], const double B[3][3], double C[3][3]) {
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      C[r][c] = A[r][c] - B[r][c];
}

void EKF::mat3_vec_mul(const double A[3][3], const double v[3], double out[3]) {
  for (int r = 0; r < 3; ++r) {
    out[r] = A[r][0] * v[0] + A[r][1] * v[1] + A[r][2] * v[2];
  }
}

bool EKF::inv2x2(const double A[2][2], double invA[2][2], double* det_out) {
  const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  if (det_out) *det_out = det;
  if (std::fabs(det) < 1e-12) return false;

  const double invdet = 1.0 / det;
  invA[0][0] =  A[1][1] * invdet;
  invA[0][1] = -A[0][1] * invdet;
  invA[1][0] = -A[1][0] * invdet;
  invA[1][1] =  A[0][0] * invdet;
  return true;
}

void EKF::resetDebugForPredict() {
  debug_.innovation_theta = quietNaN();
  debug_.innovation_x = quietNaN();
  debug_.innovation_y = quietNaN();
  debug_.NIS_theta = quietNaN();
  debug_.dx_update_in = 0.0;
  debug_.dy_update_in = 0.0;
  debug_.dtheta_update_rad = 0.0;
}

void EKF::updateDebugPostEstimate() {
  debug_.Pxx_est = P_[0][0];
  debug_.Pyy_est = P_[1][1];
  debug_.Ptt_est = P_[2][2];

  debug_.dx_update_in = x_.x_in - debug_.x_pred_in;
  debug_.dy_update_in = x_.y_in - debug_.y_pred_in;
  debug_.dtheta_update_rad = angleDiff(x_.theta_rad, debug_.theta_pred_rad);
}
