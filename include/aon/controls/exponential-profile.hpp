#pragma once

#include <cmath>
#include <algorithm>
#include "../tools/logging.hpp"

namespace aon {
class ExponentialProfile {
 private:
  double D, v_0, T;

 public:
  ExponentialProfile() {
    D = 0;
    v_0 = 0;
    T = 0;
  }

  void SetParams(double D_, double T_) {
    if (!std::isnormal(D_)) {
      aon::logging::Error("[❌] D_ is not normal.");
      return;
    }
    if (!std::isnormal(T_)) {
      aon::logging::Error("[❌] T_ is not normal.");
      return;
    }

    D = D_;
    T = T_;
    v_0 = 5 * std::abs(D) / T_;
  }

  double GetT() { return T; }
  double GetV_0() { return v_0; }
  double GetD() { return D; }

  double SpeedProfile(double t) {
    if (D == 0 || T == 0) return 0;
    const double sign_D = (D > 0) ? 1 : (D == 0) ? 0 : -1;
    const double sign_t = (t > 0) ? 1 : (t == 0) ? 0 : -1;

    return sign_D * sign_t * v_0 *
           (sign_t + std::exp(-v_0 / std::abs(D) * std::fmod(std::abs(t), T)) -
            std::floor(std::abs(T) / T) - 1);
  }

  double PositionProfile(double t) {
    if (D == 0 || T == 0) return 0;
    const double sign_t = (t > 0) ? 1 : (t == 0) ? 0 : -1;

    return sign_t * D *
           (1 + std::floor(std::abs(t) / T) -
            std::exp(-v_0 / std::abs(D) * std::fmod(std::abs(t), T)));
  }

  double PositionProfileInverse(double d) {
    if (D == 0 || T == 0) return 0;

    const double sign_d = (d > 0) ? 1 : (d == 0) ? 0 : -1;

    return sign_d * T *
           (std::floor(std::abs(d) / D) -
            std::log(1 - std::fmod(std::abs(d), D) / D) / 2.17147241);
  }
};

}  // namespace aon
