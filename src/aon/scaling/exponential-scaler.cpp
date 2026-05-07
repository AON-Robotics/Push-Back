#include <cmath>
#include "../../../include/aon/math/scaling/exponential-scaler.hpp"

namespace aon {

double ExponentialScaler::operator()(double value) {
  return ::std::exp((::std::fabs(value) - 127.0) * t / 1000.0) / 127.0;
}

} // namespace aon