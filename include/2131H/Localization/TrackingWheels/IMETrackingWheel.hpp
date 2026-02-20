/**
 * @file IMETrackingWheel.hpp
 * @author Andrew Hilton (2131H)
 * @brief Internal Motor Encoder Tracking Wheel Implementation
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

#include "2131H/Localization/TrackingWheels/AbstractTrackingWheel.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

struct MotorPacket
{
  float position;           // In encoder ticks
  std::uint32_t timestamp;  // In milliseconds
};

class IMETrackingWheel
    : public AbstractTrackingWheel,
      public std::enable_shared_from_this<IMETrackingWheel>
{
 private:
  std::map<std::int8_t, MotorPacket> motorPorts;
  pros::Mutex updateMutex;

  float averageVelocity = 0.0f;
  float averageDisplacement = 0.0f;
  const float ticksPerInch;

  pros::Task updateTask = pros::Task([]() {});

  void update_() override
  {
    // Lock mutex to update data
    std::lock_guard<pros::Mutex> lock(updateMutex);

    float velocitySum = 0.0f;
    float displacementSum = 0.0f;
    float sensorCount = static_cast<float>(motorPorts.size());

    for (auto& [port, packet] : motorPorts)
    {
      float lastPosition = packet.position;
      float lastTimestamp = packet.timestamp;

      packet.position =
          pros::c::motor_get_raw_position(port, &packet.timestamp);

      float deltaPosition = packet.position - lastPosition;
      float deltaTime =
          static_cast<float>(packet.timestamp - lastTimestamp) /
          1000.0f;  // in seconds

      // Prevent division by zero or negative time
      if (deltaTime <= 0.0f) { sensorCount -= 1.0f; }
      else
      {
        displacementSum += packet.position;
        velocitySum += deltaPosition / deltaTime;  // ticks per second
      }
    }

    // Avoid divide by zero
    // (unlikely all motors dc for some reason but this should allow for
    // recovery)
    averageVelocity =
        (sensorCount > 0.0f) ? (velocitySum / sensorCount) : 0.0f;
    averageDisplacement =
        (sensorCount > 0.0f) ? (displacementSum / sensorCount) : 0.0f;
  }

 public:
  IMETrackingWheel(
      float offsetFromCenter,
      const float ticksPerInch,
      const std::vector<std::int8_t>& ports)
      : AbstractTrackingWheel(offsetFromCenter),
        ticksPerInch(ticksPerInch),
        motorPorts()

  {
    std::lock_guard<pros::Mutex> lock(updateMutex);

    for (const auto& port : ports)
    {
      MotorPacket packet;

      packet.position =
          pros::c::motor_get_raw_position(port, &packet.timestamp);

      motorPorts[port] = packet;
    }
  }

  float getDisplacement() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);
    return averageDisplacement / ticksPerInch;
  }

  float getVelocity() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);
    return averageVelocity / ticksPerInch;
  }

  void reset() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);

    for (auto& [port, packet] : motorPorts)
    {
      pros::c::motor_tare_position(port);
      packet.position =
          pros::c::motor_get_raw_position(port, &packet.timestamp);
    }

    averageDisplacement = 0.0f;
    averageVelocity = 0.0f;
  }

  float offsetFromCenter() override { return offset; }

  static std::shared_ptr<IMETrackingWheel> build(
      float offsetFromCenter,
      const float ticksPerInch,
      const std::vector<std::int8_t>& ports)
  {
    auto ref = std::make_shared<IMETrackingWheel>(
        offsetFromCenter, ticksPerInch, ports);

    ref->startThreading();
    return ref;
  }

  void startThreading()
  {
    std::weak_ptr<IMETrackingWheel> self = weak_from_this();

    // TODO: UPDATE this to use task class once tested
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
        "IME Tracking Wheel Update Task");
  }
};