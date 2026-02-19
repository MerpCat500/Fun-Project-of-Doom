/**
 * @file DifferentialChassis.hpp
 * @author Andrew Hilton (2131N)
 * @brief Differential Chassis Implementation
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>

#include "2131N/Localization/AbstractLocalizer.hpp"
#include "2131N/Utilities/Angle.hpp"
#include "2131N/Utilities/DiffDriveCmd.hpp"
#include "2131N/Utilities/HelperMath.hpp"
#include "pros/motor_group.hpp"

struct PhysicalProperties
{
  const float drivelength;  // In inches
  const float drivewidth;   // In inches

  const float trackwidth;   // In inches
  const float tracklength;  // In inches

  const float gearing;        // Scalar
  const float wheelDiameter;  // Diameter of wheel

  const float motorWattage;  // Wattage of motors

  const float maxVelocity;  // Max Velocity of Drivetrain
  const Angle<Radians>
      maxAngularVelocity;  // Max angular velocity of drivetrain

  PhysicalProperties(
      float drivelength,
      float drivewidth,
      float trackwidth,
      float tracklength,
      float gearing,
      float wheelDiameter,
      float motorWattage,
      float maxVelocity)
      : drivelength(drivelength),
        drivewidth(drivewidth),
        trackwidth(trackwidth),
        tracklength(tracklength),
        gearing(gearing),
        wheelDiameter(wheelDiameter),
        motorWattage(motorWattage),
        maxVelocity(maxVelocity),
        maxAngularVelocity(
            Angle<Radians>(maxVelocity / (trackwidth / 2.0f)))
  {
  }
};

class DifferentialChassis
    : public std::enable_shared_from_this<DifferentialChassis>
{
 private:
  // Devices
  std::shared_ptr<AbstractLocalizer> localizer;  // Positional Tracking

  std::shared_ptr<pros::MotorGroup> leftGroup;   // Left Motors
  std::shared_ptr<pros::MotorGroup> rightGroup;  // Right Motors

  // Motor Voltages
  float leftVoltage = 0.0f;   // In millivolts
  float rightVoltage = 0.0f;  // In millivolts

  // Motor Gains
  const float
      kS;  // Static gain (millivolts needed to overcome static friction)
  const float kV;  // Velocity gain (millivolts per in/s of wheel velocity)

  // Multi-threading Objects
  pros::Mutex updateMutex;  // Mutex for updating motor voltages
  pros::Task updateTask = pros::Task(
      []() {});  // Task for continuously updating motor voltages

  // Movement Mutex
  pros::Mutex movementMutex;  // Mutex for controlling access to the
                              // chassis for movement commands
  std::uint64_t currentToken =
      1;  // Token for tracking control access to the chassis

 private:
  /**
   * @brief  Updates the motor voltages based on the current leftVoltage
   * and rightVoltage values. This function is called continuously in a
   * separate task.
   *
   */
  void update_()
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);

    leftGroup->move_voltage(static_cast<int>(leftVoltage));
    rightGroup->move_voltage(static_cast<int>(rightVoltage));
  }

 public:
  const PhysicalProperties
      properties;  // Physical properties of the chassis

 public:
  /**
   * @brief Construct a new Differential Chassis object
   *
   * @param localizer Pointer to a localizing system for the chassis
   * @param leftGroup Pointer to the group of motors on the left side of
   * the chassis
   * @param rightGroup Pointer to the group of motors on the right side of
   * the chassis
   * @param kS Static gain (millivolts needed to overcome static friction)
   * @param kV Velocity gain (millivolts per in/s of wheel velocity)
   * @param properties Physical properties of the chassis
   */
  DifferentialChassis(
      std::shared_ptr<AbstractLocalizer> localizer,
      std::shared_ptr<pros::MotorGroup> leftGroup,
      std::shared_ptr<pros::MotorGroup> rightGroup,
      float kS,
      float kV,
      const PhysicalProperties properties)
      : localizer(localizer),
        leftGroup(leftGroup),
        rightGroup(rightGroup),
        kS(kS),
        kV(kV),
        properties(properties),
        updateTask([this]() {
          while (true)  // Continuously update motor voltages in task
          {
            this->update_();
            pros::delay(10);  // Don't Hog the CPU :D
          }
        })
  {
  }

  /**
   * @brief Set a (Linear and Angular) velocity command for the chassis.
   * This will be converted to wheel voltages and applied to the motors.
   *
   * @param cmd The desired linear and angular velocity command for the
   * chassis, in inches per second and radians per second
   */
  template <typename Unit>
  void setVelocityCommand(const VelocityPair<Unit>& cmd)
  {
    // This doesn't need to be locked because it doesn't read or write any
    // shared data.
    WheelVelocities wheelVelocities =
        PairToWheel(cmd, properties.trackwidth);

    this->setWheelVelocities(wheelVelocities);
  }

  /**
   * @brief Set the Wheel Velocities of the chassis. This will be converted
   * to voltages and applied to the motors.
   *
   * @param wheelVelocities The desired wheel velocities for the chassis,
   * in inches per second
   */
  void setWheelVelocities(WheelVelocities wheelVelocities)
  {
    // This doesn't need to be locked because it doesn't read or write any
    // shared data.

    // Convert Wheel Velocities to Voltages using the feedforward formula:
    // voltage = kS * sign(velocity) + kV * velocity
    float leftVoltage = kS * sign(wheelVelocities.leftVelocity) +
                        kV * wheelVelocities.leftVelocity;
    float rightVoltage = kS * sign(wheelVelocities.rightVelocity) +
                         kV * wheelVelocities.rightVelocity;

    // Set wheel Voltages
    this->setWheelVoltages(leftVoltage, rightVoltage);
  }

  /**
   * @brief Set the Wheel Voltages for the chassis. This will be applied to
   * the motors.
   *
   * @param leftVoltage The desired voltage for the left motors, in
   * millivolts
   * @param rightVoltage The desired voltage for the right motors, in
   * millivolts
   */
  void setWheelVoltages(float leftVoltage, float rightVoltage)
  {
    // Lock the update mutex to safely update the motor voltages
    std::lock_guard<pros::Mutex> lock(updateMutex);

    this->leftVoltage = leftVoltage;
    this->rightVoltage = rightVoltage;
  }

  /**
   * @brief Get the Localizer for the chassis, which can be used to get the
   * current pose of the robot.
   *
   * @return std::shared_ptr<AbstractLocalizer> Pointer to the localizer
   * for the chassis
   */
  std::shared_ptr<AbstractLocalizer> getLocalizer() const
  {
    // This doesn't need to be locked because the pointer isn't effected by
    // other code
    return localizer;
  }

  /**
   * @brief Checkout control of the robot, which allows the caller to send
   * movement commands to the chassis. This will lock the movement mutex
   * and return a token that can be used to check if the caller still has
   * control of the chassis.
   *
   * @return std::uint64_t A token that can be used to check if the caller
   * still has control of the chassis. The caller must call
   * releaseControlToken with this token to release control of the chassis.
   */
  std::uint64_t checkoutControlToken()
  {
    // Lock the update mutex to safely access the current token and lock
    // the movement mutex
    std::lock_guard<pros::Mutex> lock(updateMutex);

    // Lock the movement mutex to gain control of the chassis for movement
    // commands
    movementMutex.lock();
    return currentToken;  // Return the current token to the caller for
                          // tracking control access
  }

  /**
   * @brief Check if the provided control token is still valid, indicating
   * if the caller has control of the chassis.
   *
   * @param token The control token to check for validity
   * @return true If the token is valid and the caller has control of the
   * chassis
   * @return false If the token is invalid and the caller does not have
   * control of the chassis
   */
  bool checkControlToken(std::uint64_t token)
  {
    // Lock the update mutex to safely check the token against the current
    // token
    std::lock_guard<pros::Mutex> lock(updateMutex);
    return (
        token ==
        currentToken);  // Returns if the provided token matches the
                        // current token, indicating control access
  }

  /**
   * @brief Release control of the chassis for the caller with the provided
   * token. This will unlock the movement mutex if the token is valid,
   * allowing other callers to gain control of the chassis.
   *
   * @param token The control token to release control of the chassis
   */
  void releaseControlToken(std::uint64_t token)
  {
    // Lock the update mutex to safely check the token and release control
    // if valid
    std::lock_guard<pros::Mutex> lock(updateMutex);

    // Only release control if the provided token matches the current token
    if (token == currentToken)
    {
      movementMutex.unlock();
      currentToken++;  // Update the current token to invalidate any
                       // previous tokens
    }
  }

  /**
   * @brief This will invalidate the current token and release control of
   * the chassis, allowing other callers to gain control. This should only
   * be called at the end of the autonomous period or the start of operator
   * control. Usage inside of a controller is discouraged as it may lead to
   * unexpected behavior.
   *
   */
  void endAutonomousMotion()
  {
    // Lock the update mutex to safely release control of the chassis
    std::lock_guard<pros::Mutex> lock(updateMutex);
    movementMutex.unlock();
    currentToken++;  // Update the current token to invalidate any previous
                     // tokens
  }

  /**
   * @brief Build a new DifferentialChassis object with the provided
   * parameters. This is a static factory method that can be used to create
   * a new chassis with the specified localizer, motor groups, gains, and
   * physical properties.
   *
   * @param localizer Pointer to a localizing system for the chassis
   * @param leftGroup Pointer to the group of motors on the left side of
   * the chassis
   * @param rightGroup  Pointer to the group of motors on the right side of
   * the chassis
   * @param kS Static gain (millivolts needed to overcome static friction)
   * @param kV Velocity gain (millivolts per in/s of wheel velocity)
   * @param properties Physical properties of the chassis
   * @return std::shared_ptr<DifferentialChassis> A shared pointer to the
   * newly created DifferentialChassis object
   */
  static std::shared_ptr<DifferentialChassis> build(
      std::shared_ptr<AbstractLocalizer> localizer,
      std::shared_ptr<pros::MotorGroup> leftGroup,
      std::shared_ptr<pros::MotorGroup> rightGroup,
      float kS,
      float kV,
      const PhysicalProperties properties)
  {
    return std::make_shared<DifferentialChassis>(
        localizer, leftGroup, rightGroup, kS, kV, properties);
  }

  void startThreading()
  {
    // TODO: Update this to use Task class once tested
    std::weak_ptr<DifferentialChassis> self = weak_from_this();

    updateTask = pros::Task(
        [self]() {
          while (true)
          {
            if (auto s = self.lock()) { s->update_(); }
            else
            {
              // object is gone → exit task cleanly
              break;
            }
            pros::delay(10);
          }
        },
        "Differential Chassis Update Task");
  }
};

// RAII Wrapper for DifferentialChassis control token
class DifferentialChassisLock
{
 private:
  std::shared_ptr<DifferentialChassis>
      chassis;          // Pointer to the chassis being controlled
  std::uint64_t token;  // Control token for tracking ownership of the
                        // chassis control

 public:
  /**
   * @brief Construct a new Differential Chassis Lock object, which will
   * acquire control of the chassis for the caller.
   *
   * @param chassis
   */
  DifferentialChassisLock(std::shared_ptr<DifferentialChassis> chassis)
      : chassis(chassis)
  {
    token = chassis->checkoutControlToken();
  }

  /**
   * @brief Checks if the lock is still active, meaning the caller still
   * has control of the chassis.
   *
   * @return true Caller has control of the chassis
   * @return false Caller should release control of the chassis
   */
  bool isValid() const { return chassis->checkControlToken(token); }

  /**
   * @brief Destroy the Differential Chassis Lock object, which will
   * release control of the chassis if the lock is still valid.
   *
   */
  ~DifferentialChassisLock() { chassis->releaseControlToken(token); }
};