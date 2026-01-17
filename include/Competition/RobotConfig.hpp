/**
 * @file RobotConfig.hpp
 * @author Andrew Hilton (2131N)
 * @brief Configuration and global declarations for the 2131N VEX Robotics
 * team
 * @version 0.1
 * @date 2026-01-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <memory>

#include "2131N/Chassis/DifferentialChassis.hpp"
#include "2131N/Controllers/BoomerangController.hpp"
#include "2131N/Controllers/ExitConditions/CustomExitCondition.hpp"
#include "2131N/Controllers/SeekingController.hpp"
#include "2131N/Controllers/TankControl.hpp"
#include "2131N/Localization/TrackingWheels/IMETrackingWheel.hpp"
#include "2131N/Localization/WheelOdometry.hpp"

extern std::shared_ptr<pros::Controller> primary;

extern std::shared_ptr<IMETrackingWheel> leftTrackingWheel;
extern std::shared_ptr<IMETrackingWheel> rightTrackingWheel;

extern std::shared_ptr<WheelOdometry> odometry;
extern std::shared_ptr<DifferentialChassis> chassis;

extern std::shared_ptr<CustomExitCondition> buttonExit;

extern SeekingController seeking;
extern BoomerangController boomerang;
extern TankControl tankControl;