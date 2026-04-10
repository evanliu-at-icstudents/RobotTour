#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <Arduino.h>
#include <Romi32U4.h>

namespace motor_interface
{
  void begin(Romi32U4Motors* motorsPtr, float trackWidthCm);
  void resetController();
  void setWheelSpeed(float v, float w, float dt);
  void stop();
}

#endif