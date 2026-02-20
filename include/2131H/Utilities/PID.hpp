/**
 * @file PID.hpp
 * @author Andrew Hilton (2131H)
 * @brief PID Utility Class
 * @version 0.1
 * @date 2025-12-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

class PID
{
 private:
  const float kP;
  const float kI;
  const float kD;

  float previousError = 0.0f;
  float integral = 0.0f;

 public:
  PID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

  float calculate(float error, float deltaTime)
  {
    integral += (error + previousError) * 0.5f * deltaTime;
    float derivative = (error - previousError) / deltaTime;
    previousError = error;

    return (kP * error) + (kI * integral) + (kD * derivative);
  }

  void reset()
  {
    previousError = 0.0f;
    integral = 0.0f;
  }
};