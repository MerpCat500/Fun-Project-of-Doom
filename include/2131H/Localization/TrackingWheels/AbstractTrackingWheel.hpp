/**
 * @file AbstractTrackingWheel.hpp
 * @author Andrew Hilton (2131H)
 * @brief
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

class AbstractTrackingWheel
{
 protected:
  /// @brief The offset of the tracking wheel from the center of the robot in inches
  const float offset;

  /**
   * @brief Internal update function, should be called by a thread / task.
   *
   */
  virtual void update_() = 0;

 public:
  AbstractTrackingWheel(float offsetFromCenter) : offset(offsetFromCenter) {}

  /**
   * @brief Get the displacement of the tracking wheel in inches since last reset
   *
   * @return float Displacement in inches
   */
  virtual float getDisplacement() = 0;

  /**
   * @brief Get the velocity of the tracking wheel in inches per seconds
   *
   * @return float
   */
  virtual float getVelocity() = 0;

  /**
   * @brief Reset the tracking wheel's displacement
   *
   */
  virtual void reset() = 0;

  /**
   * @brief Get the distance of the wheel from the center of the robot in inches
   *
   * @return float Distance offset in inches
   */
  virtual float offsetFromCenter() = 0;
};