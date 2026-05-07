#include "../../../include/aon/math/scaling/cubic-scaler.hpp"

namespace aon {

double CubicScaler::operator()(double value) {
    double x = value / 127.0;
    return (a * (x * x * x)) + ((1.0 - a) * x);
}

} // namespace aon