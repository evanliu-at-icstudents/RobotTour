#include "odometry.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Romi32U4.h>
#include <LSM6.h>

extern Romi32U4Encoders encoders;

namespace
{
  Pose state = {0.0f, 0.0f, 0.0f};

  unsigned long lastUpdateMs = 0;

  float leftVelocityCmS = 0.0f;
  float rightVelocityCmS = 0.0f;

  bool initialized = false;

  LSM6 imu;
  bool imuReady = false;

  float gyroBiasYaw = 0.0f;

  constexpr float HEADING_ALPHA = 0.95f;
  constexpr float GYRO_DPS_PER_LSB = 0.035f;
  constexpr float GYRO_RAW_DEADBAND = 2.0f;

  constexpr float STILL_VELOCITY_THRESHOLD_CM_S = 0.2f;
  constexpr float BIAS_ADAPT_RATE = 0.001f;
}

static float wrapAngleRad(float a)
{
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

static float wrapAngleDiffRad(float a)
{
  while (a > PI)  a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int16_t readYawGyroRaw()
{
  return imu.g.z;
}

static bool initializeIMU()
{
  Wire.begin();

  if (!imu.init())
  {
    return false;
  }

  imu.enableDefault();

  imu.writeReg(LSM6::CTRL2_G, 0b10001000);

  delay(100);

  const int samples = 3000;
  long sum = 0;

  for (int i = 0; i < samples; ++i)
  {
    imu.read();
    sum += readYawGyroRaw();
    delay(2);
  }

  gyroBiasYaw = (float)sum / (float)samples;
  return true;
}

void odometry::initialize()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

  state.x = 0.0f;
  state.y = 0.0f;
  state.theta = 0.0f;

  lastUpdateMs = millis();

  leftVelocityCmS = 0.0f;
  rightVelocityCmS = 0.0f;

  imuReady = initializeIMU();
  initialized = true;
}

void odometry::resetPose(float x, float y, float headingDeg)
{
  state.x = x;
  state.y = y;
  state.theta = headingDeg * DEG_TO_RAD;

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();

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

  dt = clampf(dt, 0.0005f, 0.1f);

  long dLeftCounts  = encoders.getCountsAndResetLeft();
  long dRightCounts = encoders.getCountsAndResetRight();

  float dLeftCm  = (float)dLeftCounts  / ODOM_COUNTS_PER_CM;
  float dRightCm = (float)dRightCounts / ODOM_COUNTS_PER_CM;

  float dCenterCm = 0.5f * (dLeftCm + dRightCm);
  float dThetaEnc = (dRightCm - dLeftCm) / TRACK_WIDTH_CM;

  leftVelocityCmS = dLeftCm / dt;
  rightVelocityCmS = dRightCm / dt;

  float thetaOld = state.theta;
  float thetaEnc = wrapAngleRad(thetaOld + dThetaEnc);
  float thetaFused = thetaEnc;

  if (imuReady)
  {
    imu.read();

    float gyroRaw = (float)readYawGyroRaw() - gyroBiasYaw;

    if (fabs(gyroRaw) < GYRO_RAW_DEADBAND)
    {
      gyroRaw = 0.0f;
    }

    float gyroDps = gyroRaw * GYRO_DPS_PER_LSB;
    float gyroRadS = gyroDps * DEG_TO_RAD;
    float dThetaGyro = gyroRadS * dt;

    float thetaPred = wrapAngleRad(thetaOld + dThetaGyro);

    float err = wrapAngleDiffRad(thetaEnc - thetaPred);
    thetaFused = wrapAngleRad(thetaPred + (1.0f - HEADING_ALPHA) * err);

    if (fabs(leftVelocityCmS) < STILL_VELOCITY_THRESHOLD_CM_S &&
        fabs(rightVelocityCmS) < STILL_VELOCITY_THRESHOLD_CM_S)
    {
      float rawNow = (float)readYawGyroRaw();
      gyroBiasYaw = (1.0f - BIAS_ADAPT_RATE) * gyroBiasYaw + BIAS_ADAPT_RATE * rawNow;
    }
  }

  float dThetaUsed = wrapAngleDiffRad(thetaFused - thetaOld);
  float thetaMid = wrapAngleRad(thetaOld + 0.5f * dThetaUsed);

  state.x += dCenterCm * cosf(thetaMid);
  state.y += dCenterCm * sinf(thetaMid);
  state.theta = thetaFused;

  lastUpdateMs = now;
}

Pose odometry::getPose()
{
  return state;
}

Pose odometry::getPoseDegrees()
{
  Pose p = state;
  p.theta = p.theta * RAD_TO_DEG;
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
