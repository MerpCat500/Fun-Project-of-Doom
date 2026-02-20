/**
 * @file Point.hpp
 * @author Andrew Hilton (2131N)
 * @brief Point class (2D coordinate)
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cmath>

#include "2131N/Utilities/Angle.hpp"

struct Point
{
  float x{0.0f};
  float y{0.0f};

  Point& operator*=(float scalar)
  {
    x *= scalar;
    y *= scalar;
    return *this;
  }

  Point& operator/=(float scalar)
  {
    x /= scalar;
    y /= scalar;
    return *this;
  }

  Point& operator+=(const Point& other)
  {
    x += other.x;
    y += other.y;
    return *this;
  }

  Point& operator-=(const Point& other)
  {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  Point operator/(float scalar) const { return Point{x / scalar, y / scalar}; }
  Point operator*(float scalar) const { return Point{x * scalar, y * scalar}; }
  Point operator-(const Point& other) const { return Point{x - other.x, y - other.y}; }
  Point operator+(const Point& other) const { return Point{x + other.x, y + other.y}; }

  static constexpr Point zero() { return Point{0.0f, 0.0f}; }
  static constexpr Point fromPolar(float magnitude, const Angle<Radians>& direction)
  {
    return Point{
        magnitude * std::cos(direction.getValue()), magnitude * std::sin(direction.getValue())};
  }

  static float distance(const Point& a, const Point& b)
  {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return std::hypot(dx, dy);
  }

  float distance(const Point& other) const { return Point::distance(*this, other); }

  float magnitude() const { return std::hypot(x, y); }
  Angle<Radians> direction() const { return Angle<Radians>::fromRadians(std::atan2(y, x)); }
};