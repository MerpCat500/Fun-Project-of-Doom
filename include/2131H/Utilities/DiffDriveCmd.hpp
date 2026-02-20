/**
 * @file DiffDriveCmd.hpp
 * @author Andrew Hilton (2131H)
 * @brief Differential Drive Command Structs
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "2131H/Utilities/Angle.hpp"

struct WheelVelocities
{
  float leftVelocity;   // in inches per second
  float rightVelocity;  // in inches per second
};

template <typename Units>
struct VelocityPair
{
  float linearVelocity;          // in inches per second
  Angle<Units> angularVelocity;  // in angle per second
};

inline constexpr WheelVelocities PairToWheel(VelocityPair<Degrees> velPair, const float trackWidth)
{
  float leftVel =
      velPair.linearVelocity -
      (trackWidth / 2.0f) * Angle<Radians>::toRadians(velPair.angularVelocity).getValue();
  float rightVel =
      velPair.linearVelocity +
      (trackWidth / 2.0f) * Angle<Radians>::toRadians(velPair.angularVelocity).getValue();

  return WheelVelocities{leftVel, rightVel};
}

inline constexpr WheelVelocities PairToWheel(VelocityPair<Radians> velPair, const float trackwidth)
{
  float leftVel = velPair.linearVelocity - (trackwidth / 2.0f) * velPair.angularVelocity.getValue();
  float rightVel =
      velPair.linearVelocity + (trackwidth / 2.0f) * velPair.angularVelocity.getValue();

  return WheelVelocities{leftVel, rightVel};
}
