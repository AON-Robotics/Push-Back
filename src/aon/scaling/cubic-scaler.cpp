#include "../../../include/aon/math/scaling/cubic-scaler.hpp"

namespace aon {

double CubicScaler::transform(double value) {
    value = ::std::clamp(value, -127.0, 127.0); // Normalizing input

    double x = value / 127.0;
    double result = (a * (x * x * x)) + ((1.0 - a) * x);

    return ::std::clamp(result, -1.0, 1.0); // Normalizing output
}

} // namespace aon