/**
 * @file RobotConfig.cpp
 * @author Andrew Hilton (2131H)
 * @brief
 * @version 0.1
 * @date 2026-01-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Competition/RobotConfig.hpp"

#include "2131H/Chassis/DifferentialChassis.hpp"

/**
 *-------------------------------------------------------------------------
 **          _+=+_
 **       .-`  .  `-.          8888888b.  8888888b.   .d88888b.   .d8888b.
 **    _+`     "     `+_       888   Y88b 888   Y88b d88P" "Y88b d88P  Y88b
 **   \\\sssssssssssss///      888    888 888    888 888     888 Y88b.
 **      .ss\  *  /ss.         888   d88P 888   d88P 888     888  "Y888b.
 **  .+bm  .s  *  s.  md+.     8888888P"  8888888P"  888     888     "Y88b.
 ** .hMMMMs .  *  . sMMMMh.    888        888 T88b   888     888       "888
 **  `\hMMMb \ | / dMMMh:      888        888  T88b  Y88b. .d88P Y88b  d88P
 **    -SNMNo  -  oNMNs-       888        888   T88b  "Y88888P"   "Y8888P"
 **      `+dMh\./dMd/
 **         `:yNy:`                      Powered by PROS for VEX V5
 **            "
 *---------------------------------------------------------------------------
 */

std::shared_ptr<pros::Controller> primary =
    std::make_shared<pros::Controller>(pros::E_CONTROLLER_MASTER);

std::shared_ptr<pros::MotorGroup> leftMotors =
    std::make_shared<pros::MotorGroup>(
        std::vector<int8_t>{20},
        pros::MotorGear::green,
        pros::MotorUnits::deg);

std::shared_ptr<pros::MotorGroup> rightMotors =
    std::make_shared<pros::MotorGroup>(
        std::vector<int8_t>{-11},
        pros::MotorGear::green,
        pros::MotorUnits::deg);

pros::Imu inertialSensor(10);

/**
 *?    ____   _   _      _      ____    ____                 ____
 *? U /"___| |'| |'| U  /"\  u / __"| u/ __"| u      ___    / __"| u
 *? \| | u  /| |_| |\ \/ _ \/ <\___ \/<\___ \/      |_"_|  <\___ \/
 *?  | |/__ U|  _  |u / ___ \  u___) | u___) |       | |    u___) |
 *?   \____| |_| |_| /_/   \_\ |____/>>|____/>>    U/| |\u  |____/>>
 *?  _// \\  //   \\  \\    >>  )(  (__))(  (__).-,_|___|_,-.)(  (__)
 *? (__)(__)(_") ("_)(__)  (__)(__)    (__)      \_)-' '-(_/(__)
 *?
 */

std::shared_ptr<IMETrackingWheel> leftTrackingWheel =
    IMETrackingWheel::build(-9.625f / 2.0f, 5.25, {20});
std::shared_ptr<IMETrackingWheel> rightTrackingWheel =
    IMETrackingWheel::build(9.625f / 2.0f, 5.25, {-11});

std::shared_ptr<WheelOdometry> odometry = WheelOdometry::build(
    {leftTrackingWheel, rightTrackingWheel}, {}, {&inertialSensor});

std::shared_ptr<DifferentialChassis> chassis = DifferentialChassis::build(
    odometry,
    leftMotors,
    rightMotors,
    0.0f,
    0.0f,
    {11.0f,
     13.0f,
     9.625f,
     0.00001f,
     400.0f / 600.0f,
     2.235f,
     66.0f,
     64.8f});

/**
 * !. 
 * ! #   #  ###  #   # ##### ####   ###  ##### ##### ##### ####   ####
 * ! #   # #   # #   #   #   #   # #   #  #  #  #  # #     #   # #
 * !  #### #   # #####   #   ####  #   #  #  #  #  # ####  ####  #
 * !     # #   # #   #   #   #     #   #  #  #  #  # #     #     #
 * !     #  ###  #   #   #   #      ###  #   # #   # ##### #      ####
 * !.
 */

std::shared_ptr<CustomExitCondition> buttonExit =
    CustomExitCondition::build(
        [](Target, std::shared_ptr<DifferentialChassis>) -> bool {
          return primary->get_digital(pros::E_CONTROLLER_DIGITAL_B);
        });

SeekingController seeking = SeekingController::build(
    chassis, PID(1.0f, 0.0f, 0.0f), PID(1.0f, 0.0f, 0.0f));

BoomerangController boomerang =
    BoomerangController::build(
        chassis, PID(1.0f, 0.0f, 0.0f), PID(1.0f, 0.0f, 0.0f));

TankControl tankControl =
    TankControl::build(chassis, primary);