/**
 * @file SettleExitConditions.hpp
 * @author Andrew Hilton (2131H)
 * @brief Settle Exit Conditions for robot pathing
 * @version 0.1
 * @date 2025-12-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "2131H/Controllers/ExitConditions/AbstractExitConditions.hpp"
#include "2131H/Utilities/Angle.hpp"
#include "pros/rtos.hpp"

template <typename Unit>
class SettleExitConditions : public AbstractExitCondition
{
  const Angle<Unit> angularTolerance;  // Angular Tolerance for the exit condition
  const float linearTolerance;         // Linear Tolerance for the exit condition, in inches

  const float angularExitTime;  // Time that the robot must be within the angular tolerance before
                                // the exit condition is satisfied, in milliseconds
  const float linearExitTime;  // Time that the robot must be within the linear tolerance before the
                               // exit condition is satisfied, in milliseconds

  uint32_t linearTime =
      0;  // Time that the robot has been within the linear tolerance, in milliseconds
  uint32_t angularTime =
      0;              // Time that the robot has been within the angular tolerance, in milliseconds
  uint32_t lastTime;  // Last time that the exit condition was checked, in milliseconds

 public:
  /**
   * @brief Construct a new Settle Exit Conditions object
   *
   * @param angularTolerance Angular tolerance for the exit condition
   * @param linearTolerance Linear tolerance for the exit condition, in inches
   * @param angularExitTime Time that the robot must be within the angular tolerance before the exit
   * condition is satisfied, in milliseconds
   * @param linearExitTime Time that the robot must be within the linear tolerance before the exit
   * condition is satisfied, in milliseconds
   */
  SettleExitConditions(
      Angle<Unit> angularTolerance,
      float linearTolerance,
      float angularExitTime,
      float linearExitTime)
      : angularTolerance(angularTolerance),
        linearTolerance(linearTolerance),
        angularExitTime(angularExitTime),
        linearExitTime(linearExitTime)
  {
    lastTime = pros::millis();  // Initialize lastTime to the current time when the exit condition
                                // is created
  }

  bool isSatisfied(Target target, std::shared_ptr<DifferentialChassis> pChassis) override
  {
    Pose position =
        pChassis->getLocalizer()->getPose();  // Get Position of the robot from the localizer

    // Check if the robot has been within the linear and angular tolerances for the required amount
    // of time
    if (target.targetPoint.has_value())
    {
      // Calculate the difference between the target point and the current position
      Point deltaPoint = target.targetPoint.value() - position.getPoint();

      // Calculate the linear error as the unsigned distance between the current position and the
      // target point
      float linearError = hypot(deltaPoint.x, deltaPoint.y);

      // if the linear error is within the linear tolerance, increment the linear time, otherwise
      // reset it
      if (linearTolerance > linearError) { linearTime += pros::millis() - lastTime; }
      else { linearTime = 0; }
    }
    // If there is no target point, we can consider the linear exit condition to be satisfied
    else { linearTime = linearExitTime; }

    // If there is a target heading, calculate the angular error and check if it is within the
    // angular tolerance
    if (target.targetHeading.has_value())
    {
      // Calculate the difference between the target heading and the current heading
      Angle<Radians> angularError = target.targetHeading.value() - position.heading;

      // Normalize the angular error to be within the range of -pi to pi
      while (std::fabs(angularError.getValue()) > pi)
        angularError = angularError - Angle<Radians>(
                                          2.0f * pi * angularError.getValue() /
                                          std::fabs(angularError.getValue()));

      // if the |angular error| is within the angular tolerance, increment the angular time,
      // otherwise reset it
      if (angularTolerance.getValue() > std::abs(angularError.getValue()))
      {
        angularTime += pros::millis() - lastTime;
      }
      else { angularTime = 0; }
    }
    // If there is no target heading, we can consider the angular exit condition to be satisfied
    else { angularTime = angularExitTime; }

    // Update the last time that the exit condition was checked
    lastTime = pros::millis();

    // Return true if both the linear and angular exit conditions have been satisfied for the
    // required amount of time
    return (linearExitTime < linearTime && angularExitTime < angularTime);
  };

  void reset() override
  {
    linearTime = 0;   // Reset the linear time to 0
    angularTime = 0;  // Reset the angular time to 0
    lastTime =
        pros::millis();  // Reset the last time to the current time when the exit condition is reset
  };

  static std::shared_ptr<SettleExitConditions<Unit>> build(
      Angle<Unit> angularTolerance,
      float linearTolerance,
      float angularExitTime,
      float linearExitTime)
  {
    return std::make_shared<SettleExitConditions<Unit>>(
        angularTolerance, linearTolerance, angularExitTime, linearExitTime);
  }
};