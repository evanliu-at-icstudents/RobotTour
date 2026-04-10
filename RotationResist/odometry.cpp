#include "odometry.h"

#include <Arduino.h>
#include <math.h>

#include <Romi32U4.h>

extern Romi32U4Encoders encoders;

namespace
{
  Pose state = {0.0f, 0.0f, 0.0f};

  long lastLeftCounts = 0;
  long lastRightCounts = 0;

  unsigned long lastUpdateMs = 0;

  float leftVelocityCmS = 0.0f;
  float rightVelocityCmS = 0.0f;

  bool initialized = false;
}

static float wrapAngleRad(float a)
{
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

void odometry::initialize()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  lastLeftCounts = 0;
  lastRightCounts = 0;
  lastUpdateMs = millis();

  state.x = 0.0f;
  state.y = 0.0f;
  state.theta = 0.0f;

  leftVelocityCmS = 0.0f;
  rightVelocityCmS = 0.0f;

  initialized = true;
}

void odometry::resetPose(float x, float y, float headingDeg)
{
  state.x = x;
  state.y = y;
  state.theta = headingDeg * PI / 180.0f;

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  lastLeftCounts = 0;
  lastRightCounts = 0;
  lastUpdateMs = millis();

  leftVelocityCmS = 0.0f;
  rightVelocityCmS = 0.0f;
}

void odometry::update()
{
  if (!initialized)
  {
    initialize();
  }

  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt <= 0.0f)
  {
    return;
  }

  long dLeftCounts  = encoders.getCountsAndResetLeft();
  long dRightCounts = encoders.getCountsAndResetRight();

  lastUpdateMs = now;

  float dLeftCm = (float)dLeftCounts / ODOM_COUNTS_PER_CM;
  float dRightCm = (float)dRightCounts / ODOM_COUNTS_PER_CM;

  float dCenterCm = 0.5f * (dLeftCm + dRightCm);
  float dTheta = (dRightCm - dLeftCm) / TRACK_WIDTH_CM;

  leftVelocityCmS = dLeftCm / dt;
  rightVelocityCmS = dRightCm / dt;

  float thetaMid = state.theta + 0.5f * dTheta;

  state.x += dCenterCm * cosf(thetaMid);
  state.y += dCenterCm * sinf(thetaMid);
  state.theta = wrapAngleRad(state.theta + dTheta);
}

Pose odometry::getPose()
{
  return state;
}

Pose odometry::getPoseDegrees()
{
  Pose p = state;
  p.theta = p.theta * 180.0f / PI;
  return p;
}

float odometry::getLeftVelocityCmS()
{
  return leftVelocityCmS;
}

float odometry::getRightVelocityCmS()
{
  return rightVelocityCmS;
}