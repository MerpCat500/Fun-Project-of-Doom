/**
 * @file TankControl.hpp
 * @author Andrew Hilton (2131N)
 * @brief Implementation of a Tank Drive operator control scheme
 * @version 0.1
 * @date 2026-01-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <sys/types.h>

#include "2131N/Controllers/AbstractController.hpp"

class TankControl : public AbstractController
{
 private:
  std::uint64_t currentToken = 0;             // Token for tracking control access to the chassis
  std::shared_ptr<pros::Controller> primary;  // Primary controller for operator input

 public:
  TankControl(
      std::shared_ptr<DifferentialChassis> pChassis,
      std::shared_ptr<pros::Controller> primary,
      const std::vector<std::shared_ptr<AbstractExitCondition>>& exitConditions = {})
      : AbstractController(pChassis, exitConditions), primary(primary)
  {
  }

  /**
   * @brief This should be called at the start of operator control to gain access to the chassis.
   *
   */
  void start()
  {
    pChassis->endAutonomousMotion();  // Ensure that any autonomous motion is stopped before
                                      // starting operator control
    currentToken = pChassis->checkoutControlToken();  // Check out the control token for the chassis
  }

  /**
   * @brief This should be called in a loop during operator control to continuously update the
   * chassis based on the controller input.
   *
   */
  void operatorControl()
  {
    // This function should be called in a loop during operator control to continuously update the
    // chassis based on the controller input
    if (pChassis->checkControlToken(currentToken))
    {
      // Read controller input and set motor speeds accordingly
      float leftSpeed = primary->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 12000.0f;
      float rightSpeed = primary->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) / 127.0 * 12000.0f;

      pChassis->setWheelVoltages(leftSpeed, rightSpeed);
    }
  }

  static std::shared_ptr<TankControl> build(
      std::shared_ptr<DifferentialChassis> pChassis,
      std::shared_ptr<pros::Controller> primary,
      const std::vector<std::shared_ptr<AbstractExitCondition>>& exitConditions = {})
  {
    return std::make_shared<TankControl>(pChassis, primary, exitConditions);
  }
};