/**
 * @file AbstractController.hpp
 * @author Andrew Hilton (2131H)
 * @brief Abstract Controller interface
 * @version 0.1
 * @date 2025-12-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <memory>
#include <vector>

#include "2131H/Chassis/DifferentialChassis.hpp"
#include "2131H/Controllers/ExitConditions/AbstractExitConditions.hpp"
#include "2131H/Controllers/Target.hpp"

// Abstract controller class that defines the interface for all chassis controllers
class AbstractController
{
 protected:
  std::shared_ptr<DifferentialChassis> pChassis;  // Pointer to the chassis being controlled
  std::vector<std::shared_ptr<AbstractExitCondition>>
      exitConditions;  // List of exit conditions for the controller to check during motion
                       // execution
  Target target;  // Target state for the motion being executed, this is set by the builder and used
                  // by the controller to determine how to execute the motion
  pros::Mutex mutex;  // Mutex for synchronizing access to the controller's state, including the
                      // target and exit conditions

 public:
  /**
   * @brief Construct a new Abstract Controller
   *
   * @param pChassis  Pointer to the chassis being controlled
   * @param exitConditions  List of exit conditions for the controller to check during motion
   * execution
   */
  AbstractController(
      std::shared_ptr<DifferentialChassis> pChassis,
      const std::vector<std::shared_ptr<AbstractExitCondition>>& exitConditions = {})
      : pChassis(pChassis), exitConditions(exitConditions)
  {
  }

  /**
   * @brief Set the Target for the controller, the controller will need to interpret the target and
   * execute the appropriate motion to reach it.
   *
   * @param target
   */
  void setTarget(const Target& target)
  {
    std::lock_guard<pros::Mutex> lock(mutex);
    this->target = target;
  }

  /**
   * @brief Add an exit condition to the controller, this will be checked during motion execution to
   * determine if the motion should exit.
   * @param condition The exit condition to add to the controller
   */
  void addExitCondition(std::shared_ptr<AbstractExitCondition> condition)
  {
    std::lock_guard<pros::Mutex> lock(mutex);
    exitConditions.push_back(condition);
  }

  /**
   * @brief Set the exit conditions for the controller, these will be checked during motion
   * execution to determine if the motion should exit.
   * @param conditions The exit conditions to set for the controller
   */
  void setExitConditions(const std::vector<std::shared_ptr<AbstractExitCondition>>& conditions)
  {
    std::lock_guard<pros::Mutex> lock(mutex);
    exitConditions = conditions;
  }
};