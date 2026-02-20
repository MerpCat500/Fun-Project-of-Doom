/**
 * @file CustomExitCondition.hpp
 * @author Andrew Hilton (2131H)
 * @brief Allows for weird random exit conditions via a lambda function
 * @version 0.1
 * @date 2025-12-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <functional>

#include "2131H/Controllers/ExitConditions/AbstractExitConditions.hpp"

class CustomExitCondition : public AbstractExitCondition
{
 private:
  // Function that takes in the current target and chassis state and returns whether the exit
  // condition is satisfied.
  std::function<bool(Target, std::shared_ptr<DifferentialChassis>)> conditionFunction;

 public:
  /**
   * @brief Construct a new Custom Exit Condition object
   *
   * @param conditionFunction Function that takes in the current target and chassis state and
   * returns whether the exit condition is satisfied.
   */
  CustomExitCondition(
      std::function<bool(Target, std::shared_ptr<DifferentialChassis>)> conditionFunction)
      : conditionFunction(conditionFunction)
  {
  }

  bool isSatisfied(Target target, std::shared_ptr<DifferentialChassis> pChassis) override
  {
    return conditionFunction(target, pChassis);
  }

  void reset() override {}

  /**
   * @brief Builds a new CustomExitCondition as a shared pointer given a function that takes in the
   * current target and chassis state and returns whether the exit condition is satisfied.
   *
   * @param conditionFunction Conditional Function that takes in the current target and chassis
   * state and returns true if the exit condition is satisfied, false otherwise.
   * @return std::shared_ptr<CustomExitCondition> Shared pointer to the new CustomExitCondition
   * object
   */
  static std::shared_ptr<CustomExitCondition> build(
      std::function<bool(Target, std::shared_ptr<DifferentialChassis>)> conditionFunction)
  {
    return std::make_shared<CustomExitCondition>(conditionFunction);
  }
};