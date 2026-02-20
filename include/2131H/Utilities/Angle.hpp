/**
 * @file Angle.hpp
 * @author Andrew Hilton (2131H)
 * @brief Angle Utility Class
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cmath>
#include <type_traits>
struct Degrees
{
};
struct Radians
{
};

constexpr float pi = 3.14159265358979323846f;

template <typename From, typename To>
struct AngleConverter;

template <typename Unit>
class Angle
{
 private:
  float value;

 public:
  constexpr Angle(float value) : value(value) {}

  template <typename OtherUnit, typename = std::enable_if_t<!std::is_same_v<OtherUnit, Unit>>>
  constexpr Angle(const Angle<OtherUnit>& other)
      : value(AngleConverter<OtherUnit, Unit>::convert(other.getValue()))
  {
  }

  constexpr float getValue() const { return value; }

  static constexpr Angle<Degrees> fromDegrees(float degrees) { return Angle<Degrees>(degrees); }
  static constexpr Angle<Radians> fromRadians(float radians) { return Angle<Radians>(radians); }

  static constexpr Angle<Radians> toRadians(Angle<Degrees> DegreeMeasure)
  {
    return fromRadians(DegreeMeasure.getValue() * pi / 180.0);
  }

  static constexpr Angle<Degrees> toDegrees(Angle<Radians> RadianMeasure)
  {
    return fromDegrees(RadianMeasure.getValue() * 180.0 / pi);
  }
};

template <typename U>
constexpr Angle<U> operator+(Angle<U> a, Angle<U> b)
{
  return Angle<U>(a.getValue() + b.getValue());
}

template <typename U>
constexpr Angle<U> operator-(Angle<U> a, Angle<U> b)
{
  return Angle<U>(a.getValue() - b.getValue());
}

template <typename U>
constexpr Angle<U> operator*(Angle<U> a, float scalar)
{
  return Angle<U>(a.getValue() * scalar);
}

constexpr Angle<Radians> operator""_rad(long double value)
{
  return Angle<Radians>(static_cast<float>(value));
}

constexpr Angle<Degrees> operator""_deg(long double value)
{
  return Angle<Degrees>(static_cast<float>(value));
}

template <typename U>
struct AngleConverter<U, U>
{
  static constexpr float convert(float v) { return v; }
};

template <>
struct AngleConverter<Degrees, Radians>
{
  static constexpr float convert(float deg) { return deg * pi / 180.0f; }
};

template <>
struct AngleConverter<Radians, Degrees>
{
  static constexpr float convert(float rad) { return rad * 180.0f / pi; }
};

template <typename UL, typename UR>
constexpr Angle<UL> operator+(Angle<UL> lhs, Angle<UR> rhs)
{
  return Angle<UL>(lhs.getValue() + AngleConverter<UR, UL>::convert(rhs.getValue()));
}

template <typename UL, typename UR>
constexpr Angle<UL> operator-(Angle<UL> lhs, Angle<UR> rhs)
{
  return Angle<UL>(lhs.getValue() - AngleConverter<UR, UL>::convert(rhs.getValue()));
}
