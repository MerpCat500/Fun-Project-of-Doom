/**
 * @file Autonomous.cpp
 * @author Andrew Hilton (2131H)
 * @brief Autonomous Routes
 * @version 0.1
 * @date 2026-01-15
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "Competition/Autonomous.hpp"

#include "2131H/Controllers/BoomerangController.hpp"
#include "2131H/Controllers/Target.hpp"
#include "2131H/Controllers/TargetBuilder.hpp"
#include "RobotConfig.hpp"

void debug()
{
  TargetBuilder(chassis, boomerang)
      .pose({24, 24, 90.0_deg})
      .maxVelocity(0.8)
      .targetWith(Direction::BACK)
      .build()
      .setLead(0.5)
      .moveTo();
}