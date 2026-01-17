#include "main.h"

#include "Competition/Autonomous.hpp"
#include "Competition/RobotConfig.hpp"

void initialize()
{
  chassis->getLocalizer()->calibrate();
  chassis->getLocalizer()->setPose({0.0f, 0.0f, 0.0_deg});
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
  tankControl.start();

  while (true)
  {
    tankControl.operatorControl();

    if (primary->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
      debug();
    }

    pros::delay(10);
  }
}