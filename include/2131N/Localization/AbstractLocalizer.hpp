/**
 * @file AbstractLocalizer.hpp
 * @author Andrew Hilton (2131N)
 * @brief Abstracted Localizer Interface
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "2131N/Utilities/DiffDriveCmd.hpp"
#include "2131N/Utilities/Pose.hpp"

class AbstractLocalizer
{
 protected:
  virtual void update_() = 0;

 public:
  virtual Pose getPose() = 0;
  virtual void setPose(const Pose& newPose) = 0;

  virtual Pose getVelocity() = 0;
  virtual VelocityPair<Radians> getLocalVelocity() = 0;

  virtual void reset() = 0;
  virtual void calibrate() = 0;
};