#pragma once

namespace aon {

    /// @brief This abstract class is meant to be overridden by subclasses which 
    /// implement its `transform` method to transform analog inputs however the developer wishes. 
    /// The standard is that inputs come in [-127, 127] and outputs go in [-1, 1], 
    /// but this is not a hard set rule, although it is advised.
    class Scaler {
        public:

        /// @brief Applies the scaling strategy to a floating-point input.
        /// @param value The input value to transform.
        /// @return The transformed (scaled) value.
        virtual double transform(double value) = 0;
    };

} // namespace aon