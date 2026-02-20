/**
 * @file RobotConfig.hpp
 * @author Andrew Hilton (2131H)
 * @brief Configuration and global declarations for the 2131H VEX Robotics
 * team
 * @version 0.1
 * @date 2026-01-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>

#include "2131H/Chassis/DifferentialChassis.hpp"
#include "2131H/Controllers/BoomerangController.hpp"
#include "2131H/Controllers/ExitConditions/CustomExitCondition.hpp"
#include "2131H/Controllers/SeekingController.hpp"
#include "2131H/Controllers/TankControl.hpp"
#include "2131H/Localization/TrackingWheels/IMETrackingWheel.hpp"
#include "2131H/Localization/WheelOdometry.hpp"

extern std::shared_ptr<pros::Controller> primary;

extern std::shared_ptr<IMETrackingWheel> leftTrackingWheel;
extern std::shared_ptr<IMETrackingWheel> rightTrackingWheel;

extern std::shared_ptr<WheelOdometry> odometry;
extern std::shared_ptr<DifferentialChassis> chassis;

extern std::shared_ptr<CustomExitCondition> buttonExit;

extern std::shared_ptr<SeekingController> seeking;
extern std::shared_ptr<BoomerangController> boomerang;
extern std::shared_ptr<TankControl> tankControl;