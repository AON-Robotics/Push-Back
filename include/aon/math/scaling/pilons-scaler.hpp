#pragma once

#include "./scaler.hpp"
#include <cmath>
#include <algorithm>

namespace aon {

    class PilonsScaler : public Scaler {
        private:
        
        /// @brief The "curviness" of the curve, for a detailed explanation check out the link below
        /// @see https://www.desmos.com/calculator/kq9hgbxbwp
        double t;

        public:

        /// @brief Creates a `PilonsScaler` instance
        /// @param t The "curviness" of the curve, for a detailed explanation check out the link below
        /// @see https://www.desmos.com/calculator/kq9hgbxbwp
        PilonsScaler(double t = 10) : t(t){}

        /// @brief Applies pilons input scaling strategy to a floating-point input.
        /// @param value The input value to scale in the range: [-127, 127].
        /// @return The scaled value in the range: [-1, 1].
        double transform(double value) override;
    };

} // namespace aon