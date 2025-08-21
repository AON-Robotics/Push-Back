/**
 * \file general.hpp
 * @author Marcos R. Pesante Colón (m4rc05.dev@gmail.com)
 * \brief Contains general tools that help make code shorter or more readable.
 * \version 0.1
 * \date 01-03-2022
 */
#ifndef AON_TOOLS_GENERAL_HPP_
#define AON_TOOLS_GENERAL_HPP_

#include <cmath>
#include <cfloat>

namespace aon {

/**
 * \brief Constrains value. Replaces value with 0 if outside of range.
 *
 * \tparam U Should be type that has lt (<), gt (>), and eq (==) operations.
 * \param value Value we want to constrain
 * \param min Output will not be 0 if value is smaller than this value
 * \param max Output will not be 0 if value is greater than this value
 * \return U Processed output value
 */
template <class U>
inline U threshold(U value, U min, U max) {
  if (min <= value && value <= max) return 0;
  return value;
}

/**
 * \brief Checks if values have an absolute difference less than 1.9E-7
 *
 * \details Implemented using FLT_EPSILON as the small number to compare.
 *
 * \param x First number.
 * \param y Second number.
 * \return true When values are close.
 * \return false When values are not close.
 */
inline bool is_close(double x, double y) { return abs(x - y) <= FLT_EPSILON; }

/// @brief Returns the nearest of two `values` with respect to a given `num`
/// @param num The reference number.
/// @param values The pair of values to compare
/// @return The value of the pair that is closest to `num`
inline double nearest(const double &num, const std::pair<double, double> &values){
  return std::abs(values.first - num) < std::abs(values.second - num) ? values.first : values.second;
}

};  // namespace aon

#endif  // AON_TOOLS_GENERAL_HPP_
