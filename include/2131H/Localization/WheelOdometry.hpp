/**
 * @file WheelOdometry.hpp
 * @author Andrew Hilton (2131H)
 * @brief Wheel Odometry based Localizer Implementation
 * @version 0.1
 * @date 2025-12-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <sys/types.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include "2131H/Localization/AbstractLocalizer.hpp"
#include "2131H/Localization/TrackingWheels/AbstractTrackingWheel.hpp"
#include "2131H/Utilities/Angle.hpp"
#include "2131H/Utilities/HelperMath.hpp"
#include "2131H/Utilities/Threading.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

class WheelOdometry : public AbstractLocalizer,
                      public std::enable_shared_from_this<WheelOdometry>
{
 private:
  // Sensor Information
  std::vector<std::shared_ptr<AbstractTrackingWheel>> linearWheels;
  std::vector<std::shared_ptr<AbstractTrackingWheel>> lateralWheels;
  std::vector<pros::IMU*> inertialSensors;

  // Multithreading Objects
  pros::Mutex updateMutex;
  Task<WheelOdometry> updateTask;

  // Robot State
  Pose currentPose{0, 0, 0};
  Pose lastPose{0, 0, 0};

  Pose globalVelocity{0, 0, 0};
  VelocityPair<Radians> currentVelocity{0, 0};

  uint64_t lastUpdateTime = 0;

 private:
  void update_() override
  {
    // TODO: Implement IMU Drift Correction and check to see if it improves
    // accuracy

    std::lock_guard<pros::Mutex> lock(updateMutex);
    if (linearWheels.size() == 0 || inertialSensors.size() == 0) return;

    // Calculate average heading from all the IMU Sensors
    Angle<Radians> heading = 0.0f;
    for (auto& sensor : inertialSensors)
    {
      // * -sensor->get_heading() to convert from clockwise positive to
      // counterclockwise positive
      heading = heading + Angle<Radians>::toRadians(Angle<Degrees>(
                              360.0f - sensor->get_heading()));
    }
    if (std::isinf(heading.getValue()) || std::isnan(heading.getValue()))
      return;

    heading = Angle<Radians>(
        heading.getValue() / static_cast<float>(inertialSensors.size()));

    // Calculate change in heading
    Angle<Radians> deltaHeading = heading - lastPose.getAngle<Radians>();

    // Average heading during the last 10ms (in radians)
    float averageHeading = lastPose.getAngle<Radians>().getValue() +
                           (deltaHeading.getValue() / 2.0f);

    // Calculate the delta time since last update
    auto currentTime = pros::micros();
    float deltaTime = static_cast<float>(currentTime - lastUpdateTime) /
                      1000000.0f;  // in seconds

    if (deltaTime <= 0.0f)
      return;  // Prevent division by zero or negative time

    // Calculate the distance moved in the colinear direction

    float colinearDisplacement = 0.0f;
    for (auto& wheel : linearWheels)
    {
      float deltaX = wheel->getVelocity() * deltaTime;
      if (std::abs(deltaHeading.getValue()) < 2.0e-4f)
      {
        colinearDisplacement += deltaX;
      }
      else
      {
        colinearDisplacement += chordLength(
            deltaX / deltaHeading.getValue() + wheel->offsetFromCenter(),
            deltaHeading);
      }
    }
    colinearDisplacement /= static_cast<float>(linearWheels.size());

    // Calculate the distance moved in the lateral direction
    float lateralDisplacement = 0.0f;
    for (auto& wheel : lateralWheels)
    {
      float deltaY = wheel->getVelocity() * deltaTime;
      if (std::abs(deltaHeading.getValue()) < 2.0e-6f)
      {
        lateralDisplacement += deltaY;
      }
      else
      {
        lateralDisplacement += chordLength(
            deltaY / deltaHeading.getValue() + wheel->offsetFromCenter(),
            deltaHeading);
      }
    }
    if (lateralWheels.size() > 0)
      lateralDisplacement /= static_cast<float>(lateralWheels.size());

    // Apply rotation using average heading (radians)
    currentPose =
        lastPose + Pose{
                       colinearDisplacement * std::cos(averageHeading) -
                           lateralDisplacement * std::sin(averageHeading),
                       colinearDisplacement * std::sin(averageHeading) +
                           lateralDisplacement * std::cos(averageHeading),
                       deltaHeading,
                   };

    // Update the local and global velocities
    currentVelocity.linearVelocity =
        Point::distance(currentPose.getPoint(), lastPose.getPoint()) /
        deltaTime;
    currentVelocity.angularVelocity =
        Angle<Radians>::fromRadians(deltaHeading.getValue() / deltaTime);

    globalVelocity = (currentPose - lastPose) / deltaTime;

    // Cache previous values
    lastPose = currentPose;
    lastUpdateTime = currentTime;
  };

 protected:
  WheelOdometry(
      const std::vector<std::shared_ptr<AbstractTrackingWheel>>& colinear,
      const std::vector<std::shared_ptr<AbstractTrackingWheel>>& lateral,
      const std::vector<pros::IMU*>& headingSensorsList)
      : linearWheels(colinear),
        lateralWheels(lateral),
        inertialSensors(headingSensorsList)
  {
    lastUpdateTime = pros::micros();
  };

 public:
  Pose getPose() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);
    return currentPose;
  };

  void setPose(const Pose& newPose) override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);
    currentPose = newPose;
    lastPose = newPose;

    currentVelocity = {0, Angle<Radians>::fromRadians(0.0f)};
    globalVelocity = Pose{0, 0, 0};

    lastUpdateTime = pros::micros();
  };

  Pose getVelocity() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);

    return globalVelocity;
  };
  VelocityPair<Radians> getLocalVelocity() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);
    return currentVelocity;
  };

  void reset() override
  {
    std::lock_guard<pros::Mutex> lock(updateMutex);
    currentPose = Pose::zero();
    lastPose = Pose::zero();

    currentVelocity = {0, Angle<Radians>::fromRadians(0.0f)};
    globalVelocity = Pose{0, 0, 0};

    lastUpdateTime = pros::micros();

    for (auto& wheel : linearWheels) { wheel->reset(); }
    for (auto& wheel : lateralWheels) { wheel->reset(); }
  };

  void calibrate() override
  {
    for (auto& wheel : linearWheels) { wheel->reset(); }
    for (auto& wheel : lateralWheels) { wheel->reset(); }

    // Start Calibration on all imus
    for (auto& sensor : inertialSensors) { sensor->reset(false); }

    // Wait for all IMUs to finish calibrating before continuing
    for (auto& sensor : inertialSensors)
    {
      while (sensor->is_calibrating()) { pros::delay(10); }
    }
  };

  static std::shared_ptr<WheelOdometry> build(
      const std::vector<std::shared_ptr<AbstractTrackingWheel>>& linear,
      const std::vector<std::shared_ptr<AbstractTrackingWheel>>& lateral,
      const std::vector<pros::IMU*>& inertialSensors)
  {
    auto ref = std::shared_ptr<WheelOdometry>(
        new WheelOdometry(linear, lateral, inertialSensors));

    ref->startThreading();
    return ref;
  }

  void startThreading()
  {
    // TODO: Test this multithreading implementation, make sure to update
    // other classes if it works.
    updateTask = Task<WheelOdometry>(
        this,
        [this]() { this->update_(); },
        "Wheel Odometry Update Task",
        10);

    updateTask.start();
  }
};