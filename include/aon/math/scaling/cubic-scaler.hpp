#pragma once

#include "./scaler.hpp"
#include <algorithm>

namespace aon {

    class CubicScaler : public Scaler {
        private:
        
        /// @brief The factor by which to multiply the cubic function in the range [-1, 1]
        /// @note `a` should be kept within its specified range, otherwise strange curve behavior occurs
        /// @see https://www.desmos.com/calculator/czsxed3byv
        double a;

        public:
        /// @brief Creates a `CubicScaler` instance
        /// @param a The factor by which to multiply the cubic function in the range [-1, 1]
        /// @note `a` should be kept within its specified range, otherwise strange curve behavior occurs
        /// @see https://www.desmos.com/calculator/czsxed3byv
        CubicScaler(double a = 1) : a(a){}

        /// @brief Applies the scaling strategy to a floating-point input.
        /// @param value The input value to scale in the range: [-127, 127].
        /// @return The scaled value in the range: [-1, 1].
        double transform(double value) override;
    };

} // namespace aon