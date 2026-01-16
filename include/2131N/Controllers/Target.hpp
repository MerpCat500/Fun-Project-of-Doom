/**
 * @file Target.hpp
 * @author Andrew Hilton (2131N)
 * @brief Target robot state
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdint>
#include <optional>

#include "2131N/Utilities/Angle.hpp"
#include "2131N/Utilities/Point.hpp"
#include "2131N/Utilities/Pose.hpp"

enum class Direction
{
  FRONT,
  BACK,
  AUTOMATIC
};

struct Target
{
  std::optional<Point> targetPoint = std::nullopt;  // In inches
  std::optional<Angle<Radians>> targetHeading =
      std::nullopt;  // In radians

  float finalVelocityPct = 0.0f;  // In percent of max velocity
  float maxVelocityPct = 1.0f;    // In percent of max velocity
  bool asAsync = false;  // Whether to execute the motion asynchronously

  std::optional<std::uint32_t> timeout = std::nullopt;  // In milliseconds

  Direction targetWithFront = Direction::AUTOMATIC;

  Target() = default;

  Target(
      Pose target_pose,
      float finalVelocityPct = 0.0f,
      float maxVelocityPct = 1.0f,
      bool asAsync = false,
      std::optional<std::uint32_t> timeout = std::nullopt,
      Direction targetWithFront = Direction::AUTOMATIC)
      : targetPoint(target_pose.getPoint()),
        targetHeading(target_pose.getAngle<Radians>()),
        finalVelocityPct(std::abs(finalVelocityPct)),
        maxVelocityPct(std::abs(maxVelocityPct)),
        asAsync(asAsync),
        timeout(timeout),
        targetWithFront(targetWithFront)
  {
  }

  Target(
      Point target_point,
      float finalVelocityPct = 0.0f,
      float maxVelocityPct = 1.0f,
      bool asAsync = false,
      std::optional<std::uint32_t> timeout = std::nullopt,
      Direction targetWithFront = Direction::AUTOMATIC)
      : targetPoint(target_point),
        targetHeading(std::nullopt),
        finalVelocityPct(std::abs(finalVelocityPct)),
        maxVelocityPct(std::abs(maxVelocityPct)),
        asAsync(asAsync),
        timeout(timeout),
        targetWithFront(targetWithFront)
  {
  }

  template <typename Unit>
  Target(
      Angle<Unit> target_heading,
      float finalVelocityPct = 0.0f,
      float maxVelocityPct = 1.0f,
      bool asAsync = false,
      std::optional<std::uint32_t> timeout = std::nullopt,
      Direction targetWithFront = Direction::AUTOMATIC)
      : targetPoint(std::nullopt),
        targetHeading(AngleConverter<Unit, Radians>::convert(
            target_heading.getValue())),
        finalVelocityPct(std::abs(finalVelocityPct)),
        maxVelocityPct(std::abs(maxVelocityPct)),
        asAsync(asAsync),
        timeout(timeout),
        targetWithFront(targetWithFront)
  {
  }
};