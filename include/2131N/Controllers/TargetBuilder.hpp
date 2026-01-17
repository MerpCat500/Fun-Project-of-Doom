/**
 * @file TargetBuilder.hpp
 * @author Andrew Hilton (2131H)
 * @brief Builder for building Targets and feeding them to a controller
 * @version 0.1
 * @date 2026-01-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "2131N/Chassis/DifferentialChassis.hpp"
#include "2131N/Controllers/AbstractController.hpp"
#include "2131N/Controllers/Target.hpp"
#include "2131N/Utilities/Angle.hpp"

template <std::derived_from<AbstractController> ControllerType>
class TargetBuilder
{
 private:
  ControllerType pController;
  std::shared_ptr<DifferentialChassis> pChassis;
  Target target;

 public:
  TargetBuilder(
      std::shared_ptr<DifferentialChassis> chassis,
      ControllerType& controller)
      : pController(controller), pChassis(chassis)
  {
  }

  TargetBuilder& pose(Pose pose)
  {
    target.targetPoint = pose.getPoint();
    target.targetHeading = pose.getAngle<Radians>();

    return *this;
  }

  TargetBuilder& point(Point point)
  {
    target.targetPoint = point;
    return *this;
  }

  template <typename Unit>
  TargetBuilder& heading(Angle<Unit> heading)
  {
    target.targetHeading = AngleConverter<Unit, Radians>::convert(heading);

    return *this;
  }

  TargetBuilder& clampVelocities(
      float finalVelocityPct, float maxVelocityPct)
  {
    target.finalVelocityPct = std::abs(finalVelocityPct);
    target.maxVelocityPct = std::abs(maxVelocityPct);

    return *this;
  }

  TargetBuilder& finalVelocity(float finalVelocityPct)
  {
    target.finalVelocityPct = std::abs(finalVelocityPct);
    return *this;
  }

  TargetBuilder& maxVelocity(float maxVelocityPct)
  {
    target.maxVelocityPct = std::abs(maxVelocityPct);
    return *this;
  }

  TargetBuilder& async(bool asAsync = true)
  {
    target.asAsync = asAsync;
    return *this;
  }

  TargetBuilder& timeout(std::uint32_t timeoutMs)
  {
    target.timeout = timeoutMs;
    return *this;
  }

  TargetBuilder& targetWith(Direction direction)
  {
    target.targetWithFront = direction;
    return *this;
  }

  ControllerType build()
  {
    pController.setTarget(target);
    return pController;
  }
};
