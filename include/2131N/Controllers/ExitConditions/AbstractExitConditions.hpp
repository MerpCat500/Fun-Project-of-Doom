/**
 * @file AbstractExitConditions.hpp
 * @author Andrew Hilton (2131N)
 * @brief Exit Conditions Interface for Controllers
 * @version 0.1
 * @date 2025-12-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <memory>

#include "2131N/Chassis/DifferentialChassis.hpp"
#include "2131N/Controllers/Target.hpp"

class AbstractExitCondition
{
 public:
  /**
   * @brief Check if the exit condition is satisfied given the current target and chassis state
   *
   * @param target target state for the motion being executed
   * @param pChassis pointer to the chassis being controlled, can be used to access the current
   * state of the chassis.
   * @return true The motion can exit.
   * @return false The motion should continue executing.
   */
  virtual bool isSatisfied(Target target, std::shared_ptr<DifferentialChassis> pChassis) = 0;

  /**
   * @brief Reset the exit condition to its initial state. This is called when the controller starts
   * executing a new motion, and can be used to reset any internal state of the exit condition.
   *
   */
  virtual void reset() = 0;
};