/**
 * @file BoomerangController.hpp
 * @author Andrew Hilton (2131N)
 * @brief Boomerang Controller for better move to pose control
 * @version 0.1
 * @date 2025-12-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

#include "2131N/Controllers/AbstractController.hpp"
#include "2131N/Utilities/Angle.hpp"
#include "2131N/Utilities/DiffDriveCmd.hpp"
#include "2131N/Utilities/PID.hpp"
#include "pros/rtos.hpp"

class BoomerangController : public AbstractController
{
 private:
  PID angularPID;
  PID linearPID;
  float kLead;

 public:
  BoomerangController(
      std::shared_ptr<DifferentialChassis> pChassis,
      PID linearPID,
      PID angularPID,
      float kLead = 0.5f,
      const std::vector<std::shared_ptr<AbstractExitCondition>>&
          exitConditions = {})
      : AbstractController(pChassis, exitConditions),
        linearPID(linearPID),
        angularPID(angularPID),
        kLead(kLead)
  {
  }

  void moveTo()
  {
    std::cout << "BoomerangController: Starting moveTo with target: "
              << this->target.targetPoint.value().x << ", "
              << this->target.targetPoint.value().y << std::endl;
    mutex.lock();
    linearPID.reset();
    angularPID.reset();

    // Enforce that the target is valid
    if (!this->target.targetPoint.has_value() &&
        !this->target.targetHeading.has_value())
    {
      mutex.unlock();
      return;
    }

    if (target.asAsync)
    {
      target.asAsync = false;
      mutex.unlock();

      pros::Task([this]() { this->moveTo(); }, "Boomerang Move To Task");
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
                    return !ec->isSatisfied(this->target, pChassis);
                  }))
    {
      Pose currentPose = pChassis->getLocalizer()->getPose();
      float linearError = Point::distance(
          currentPose.getPoint(), this->target.targetPoint.value());

      Point carrotPoint =
          Point::fromPolar(
              -linearError * kLead, this->target.targetHeading.value()) +
          this->target.targetPoint.value();
      Point deltaPoint = carrotPoint - currentPose.getPoint();

      Angle<Radians> angularError =
          Angle<Radians>(atan2(deltaPoint.y, deltaPoint.x)) -
          currentPose.heading;  // Final heading error

      while (std::fabs(angularError.getValue()) > pi)
        angularError =
            angularError - Angle<Radians>(
                               2.0f * pi * angularError.getValue() /
                               std::fabs(angularError.getValue()));

      float linearSpeed = linearPID.calculate(linearError, 0.01f);

      linearSpeed = std::min(
                        std::abs(linearSpeed),
                        this->target.maxVelocityPct *
                            pChassis->properties.maxVelocity) *
                    sign(linearSpeed);

      float angularSpeed =
          angularPID.calculate(angularError.getValue(), 0.01f);

      if (linearError < pChassis->properties.trackwidth / 2)
      {
        Angle<Radians> finalAngularError =
            this->target.targetHeading.value() - currentPose.heading;
        angularSpeed =
            angularPID.calculate(finalAngularError.getValue(), 0.01f);
      }

      linearSpeed = std::max(
                        pChassis->properties.maxVelocity *
                            this->target.finalVelocityPct,
                        linearSpeed) *
                    sign(linearSpeed);

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

  BoomerangController& setLead(float kLead)
  {
    std::lock_guard<pros::Mutex> lock(mutex);
    this->kLead = kLead;
    return *this;
  }

  static BoomerangController build(
      std::shared_ptr<DifferentialChassis> pChassis,
      PID linearPID,
      PID angularPID,
      const std::vector<std::shared_ptr<AbstractExitCondition>>&
          exitConditions = {})
  {
    return {pChassis, linearPID, angularPID, 0.5, exitConditions};
  }
};