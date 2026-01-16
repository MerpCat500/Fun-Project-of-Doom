/**
 * @file Pose.hpp
 * @author Andrew Hilton (2131N)
 * @brief Pose class representing a robot position and orientation
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <type_traits>

#include "2131N/Utilities/Angle.hpp"
#include "2131N/Utilities/Point.hpp"

template <typename>
inline constexpr bool pose_always_false_v = false;

struct Pose
{
  Point position;
  Angle<Radians> heading;

  Pose& operator+=(const Pose& other)
  {
    position += other.position;
    heading = heading + other.heading;
    return *this;
  }

  Pose& operator-=(const Pose& other)
  {
    position -= other.position;
    heading = heading - other.heading;
    return *this;
  }

  Pose& operator*=(const double scalar)
  {
    position *= scalar;
    heading = heading * scalar;
    return *this;
  }

  Pose& operator/=(const double scalar)
  {
    position /= scalar;
    heading = Angle<Radians>::fromRadians(heading.getValue() / scalar);
    return *this;
  }

  static Pose zero() { return Pose{Point::zero(), Angle<Radians>::fromRadians(0.0f)}; }

  Point getPoint() const { return position; }

  template <typename Unit>
  constexpr Angle<Unit> getAngle() const
  {
    if constexpr (std::is_same_v<Unit, Radians>) { return heading; }
    else if constexpr (std::is_same_v<Unit, Degrees>) { return Angle<Degrees>::toDegrees(heading); }
    else { static_assert(pose_always_false_v<Unit>, "Unsupported angle unit"); }
  }
};

inline Pose operator+(Pose lhs, const Pose& rhs)
{
  lhs += rhs;
  return lhs;
}

inline Pose operator-(Pose lhs, const Pose& rhs)
{
  lhs -= rhs;
  return lhs;
}

inline Pose operator*(Pose lhs, const double scalar)
{
  lhs *= scalar;
  return lhs;
}

inline Pose operator/(Pose lhs, const double scalar)
{
  lhs /= scalar;
  return lhs;
}