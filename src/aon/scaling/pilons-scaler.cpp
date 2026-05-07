#include <cmath>
#include "../../../include/aon/math/scaling/pilons-scaler.hpp"

namespace aon {

double PilonsScaler::operator()(double value) {
  const double a = ::std::exp(-::std::fabs(t) / 10.0);
  const double b = ::std::exp((::std::fabs(value) - 127.0) / 10.0);

  return (a + b * (1 - a)) * value / 127.0;
}

} // namespace aon