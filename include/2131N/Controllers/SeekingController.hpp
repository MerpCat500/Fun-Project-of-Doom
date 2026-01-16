/**
 * @file SeekingController.hpp
 * @author Andrew Hilton (2131N)
 * @brief Seeking Controller featuring a simple linear and angular pid
 * control algorithm
 * @version 0.1
 * @date 2025-12-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "2131N/Chassis/DifferentialChassis.hpp"
#include "2131N/Controllers/AbstractController.hpp"
#include "2131N/Controllers/ExitConditions/AbstractExitConditions.hpp"
#include "2131N/Controllers/Target.hpp"
#include "2131N/Utilities/Angle.hpp"
#include "2131N/Utilities/PID.hpp"

class SeekingController : public AbstractController
{
 private:
  PID linearPID;
  PID angularPID;

 public:
  SeekingController(
      std::shared_ptr<DifferentialChassis> pChassis,
      PID linearPID,
      PID angularPID,
      const std::vector<std::shared_ptr<AbstractExitCondition>>&
          exitConditions = {})
      : AbstractController(pChassis, exitConditions),
        linearPID(linearPID),
        angularPID(angularPID)
  {
  }

  void moveTo()
  {
    mutex.lock();
    linearPID.reset();
    angularPID.reset();

    // Enforce that the target is valid
    if (!this->target.targetPoint.has_value())
    {
      mutex.unlock();
      return;
    }

    if (target.asAsync)
    {
      mutex.unlock();
      target.asAsync = false;

      pros::Task(
          [this]() { this->moveTo(); },
          "Async Seeking Controller Move To Task");
      return;
    }

    // Acquire lock on chassis movement
    std::uint64_t key = pChassis->checkoutControlToken();
    auto startTime = pros::millis();

    while (pChassis->checkControlToken(key) &&  // While holding control

           this->target.timeout.value_or(
               std::numeric_limits<std::uint32_t>::max()) >
               pros::millis() - startTime  // And within timeout
           && std::any_of(
                  std::begin(exitConditions),
                  std::end(exitConditions),
                  [&](std::shared_ptr<AbstractExitCondition> ec) {
                    return !ec->isSatisfied(
                        this->target, pChassis);  // or return !i ;
                  }))
    {
      // Get the current pose and calculate the delta
      Pose currentPose = pChassis->getLocalizer()->getPose();
      Point deltaPoint =
          this->target.targetPoint.value() - currentPose.getPoint();

      // Calculate Positional Error
      float distanceError = deltaPoint.magnitude();
      Angle<Radians> angularError =
          Angle<Radians>(std::atan2(deltaPoint.y, deltaPoint.x)) -
          currentPose.getAngle<Radians>();

      if (this->target.targetWithFront == Direction::BACK)
      {
        angularError =
            Angle<Radians>(std::atan2(-deltaPoint.y, -deltaPoint.x)) -
            currentPose.getAngle<Radians>();
      }

      angularError = Angle<Radians>(
          std::remainder(angularError.getValue(), 2.0f * pi));

      float linearSpeed = linearPID.calculate(distanceError, 0.01f);

      linearSpeed = std::min(
                        std::abs(linearSpeed),
                        this->target.maxVelocityPct *
                            pChassis->properties.maxVelocity) *
                    sign(linearSpeed);

      linearSpeed *= std::cos(angularError.getValue());
      linearSpeed *=
          (this->target.targetWithFront == Direction::BACK ? -1.0f : 1.0f);

      linearSpeed = std::max(
                        pChassis->properties.maxVelocity *
                            this->target.finalVelocityPct,
                        std::abs(linearSpeed)) *
                    sign(linearSpeed);

      float angularSpeed;
      if (distanceError < pChassis->properties.trackwidth / 2.0f)
      {
        if (this->target.targetHeading.has_value())
        {
          // turn to face the finale pose angle if executing a pose
          // movement
          Angle<Radians> poseError =
              this->target.targetHeading.value() - currentPose.heading;

          while (std::fabs(poseError.getValue()) > pi)
            poseError = poseError - Angle<Radians>(
                                        2.0f * pi * poseError.getValue() /
                                        std::fabs(poseError.getValue()));

          angularSpeed = angularPID.calculate(poseError.getValue(), 0.01f);
        }
        else { angularSpeed = 0.0f; }
      }
      else
      {
        if (fabs(angularError.getValue()) > pi / 2.0f &&
            this->target.targetWithFront == Direction::AUTOMATIC)
        {
          angularError =
              angularError - Angle<Radians>(
                                 (angularError.getValue() /
                                  fabs(angularError.getValue())) *
                                 pi);
          linearSpeed = -linearSpeed;
        }

        angularSpeed =
            angularPID.calculate(angularError.getValue(), 0.01f);
      }

      pChassis->setVelocityCommand(VelocityPair<Radians>{
          linearSpeed, Angle<Radians>(angularSpeed)});

      mutex.unlock();
      pros::delay(10);
      mutex.lock();
    }

    mutex.unlock();
    pChassis->releaseControlToken(key);
    pChassis->setWheelVoltages(0, 0);
  }

  void turnTo()
  {
    mutex.lock();
    linearPID.reset();
    angularPID.reset();

    if (target.asAsync)
    {
      mutex.unlock();
      target.asAsync = false;

      pros::Task(
          [this]() { this->moveTo(); },
          "Async Seeking Controller Move To Task");
      return;
    }

    // Acquire lock on chassis movement
    std::uint64_t key = pChassis->checkoutControlToken();
    auto startTime = pros::millis();

    while (pChassis->checkControlToken(key) &&  // While holding control
           this->target.timeout.value_or(
               std::numeric_limits<std::uint32_t>::max()) >
               pros::millis() - startTime  // And within timeout
           && std::any_of(
                  std::begin(exitConditions),
                  std::end(exitConditions),
                  [&](std::shared_ptr<AbstractExitCondition> ec) {
                    return !ec->isSatisfied(
                        this->target, pChassis);  // or return !i ;
                  }))
    {
      // Get the current pose and calculate the delta
      Pose currentPose = pChassis->getLocalizer()->getPose();

      float angularSpeed = 0.0f;

      if (this->target.targetPoint.has_value())
      {
        Point deltaPoint =
            this->target.targetPoint.value() - currentPose.getPoint();
        // Calculate Positional Error
        Angle<Radians> angularError =
            Angle<Radians>(std::atan2(deltaPoint.y, deltaPoint.x)) -
            currentPose.getAngle<Radians>();

        if (this->target.targetWithFront == Direction::BACK)
        {
          angularError =
              Angle<Radians>(std::atan2(-deltaPoint.y, -deltaPoint.x)) -
              currentPose.getAngle<Radians>();
        }

        angularError = Angle<Radians>(
            std::remainder(angularError.getValue(), 2 * pi));

        if (fabs(angularError.getValue()) > pi / 2.0f &&
            this->target.targetWithFront == Direction::AUTOMATIC)
        {
          angularError =
              angularError - Angle<Radians>(
                                 (angularError.getValue() /
                                  fabs(angularError.getValue())) *
                                 pi);
        }

        angularSpeed =
            angularPID.calculate(angularError.getValue(), 0.01f);
      }
      else if (this->target.targetHeading.has_value())
      {
        // turn to face the angle if the target is only thing specified
        Angle<Radians> poseError =
            this->target.targetHeading.value() - currentPose.heading;

        while (std::fabs(poseError.getValue()) > pi)
          poseError = poseError - Angle<Radians>(
                                      2.0f * pi * poseError.getValue() /
                                      std::fabs(poseError.getValue()));

        angularSpeed = angularPID.calculate(poseError.getValue(), 0.01f);
      }
      else { break; }

      angularSpeed =
          std::clamp(
              std::abs(angularSpeed),
              this->target.finalVelocityPct *
                  pChassis->properties.maxAngularVelocity.getValue(),
              this->target.maxVelocityPct *
                  pChassis->properties.maxAngularVelocity.getValue()) *
          sign(angularSpeed);

      pChassis->setVelocityCommand(
          VelocityPair<Radians>{0.0f, Angle<Radians>(angularSpeed)});

      mutex.unlock();
      pros::delay(10);
      mutex.lock();
    }

    mutex.unlock();
    pChassis->releaseControlToken(key);
    pChassis->setWheelVoltages(0, 0);
  }

  static std::shared_ptr<SeekingController> build(
      std::shared_ptr<DifferentialChassis> pChassis,
      PID linearPID,
      PID angularPID,
      const std::vector<std::shared_ptr<AbstractExitCondition>>&
          exitConditions = {})
  {
    return std::make_shared<SeekingController>(
        pChassis, linearPID, angularPID, exitConditions);
  }
};