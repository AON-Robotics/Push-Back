#include "../../../include/aon/math/scaling/pilons-scaler.hpp"

namespace aon {

double PilonsScaler::transform(double value) {
  value = ::std::clamp(value, -127.0, 127.0); // Normalizing input

  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(value) - 127.0) / 10.0);

  double result = (a + b * (1 - a)) * value / 127.0;

  return ::std::clamp(result, -1.0, 1.0); // Normalizing output
}

} // namespace aon