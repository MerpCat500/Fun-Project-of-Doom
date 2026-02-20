/**
 * @file HelperMath.hpp
 * @author Andrew Hilton (2131H)
 * @brief Helper Math Functions, such as chord length calculations
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cmath>

#include "2131H/Utilities/Angle.hpp"

inline float chordLength(float radius, Angle<Radians> angle)
{
  return 2.0f * radius * std::sin(angle.getValue() / 2.0f);
}

/**
 * @brief Returns the sign of a number.
 * @details Returns 1 if the number is positive, -1 if negative, else it returns the value (in cases
 * of -nan, +nan, -0, +0).
 *
 * @tparam T Return Type and Type of the input
 * @param x Value to check sign of
 * @return T Returns 1 if positive, -1 if negative, or the input value itself for cases like -0 or
 * +0
 */
template <typename T>
inline T sign(T x)
{
  if (x > 0)  // if Value is positive
    return 1;
  else if (x < 0)  // if Value is negative
    return -1;
  else         // Value is NaN, inf, or other edgecase
    return x;  // return edgecase
}

inline float calculateTicksPerInch(float wheelDiameter, float wheelRPM)
{
  // 1800.0f is the number of ticks per revolution for the VEX V5 motor with an internal encoder
  return (1800.0f / wheelRPM) / (wheelDiameter * pi);
}