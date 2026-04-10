#include "motor_interface.h"
#include "odometry.h"

namespace
{
  Romi32U4Motors* g_motors = nullptr;

  float g_trackWidth = 14.1f;

  const float WHEEL_KP                 = 8.0f;
  const float WHEEL_KD                 = 0.15f;
  const float BASE_MOTOR_CMD_PER_CM_S  = 8.0f;
  const float RIGHT_MOTOR_TRIM         = -2.0f;

  float prevLeftErr  = 0.0f;
  float prevRightErr = 0.0f;

  float clampf_local(float v, float lo, float hi)
  {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }
}

namespace motor_interface
{
  void begin(Romi32U4Motors* motorsPtr, float trackWidthCm)
  {
    g_motors = motorsPtr;
    g_trackWidth = trackWidthCm;
    resetController();
  }

  void resetController()
  {
    prevLeftErr = 0.0f;
    prevRightErr = 0.0f;
  }

  void setWheelSpeed(float v, float w, float dt)
  {
    if (g_motors == nullptr) return;

    float vL_target = v - 0.5f * g_trackWidth * w;
    float vR_target = v + 0.5f * g_trackWidth * w;

    float vL_meas = odometry::getLeftVelocityCmS();
    float vR_meas = odometry::getRightVelocityCmS();

    float leftErr  = vL_target - vL_meas;
    float rightErr = vR_target - vR_meas;

    float dLeftErr = 0.0f;
    float dRightErr = 0.0f;

    if (dt > 0.0f)
    {
      dLeftErr  = (leftErr  - prevLeftErr)  / dt;
      dRightErr = (rightErr - prevRightErr) / dt;
    }

    prevLeftErr  = leftErr;
    prevRightErr = rightErr;

    float leftCmdF = vL_target * BASE_MOTOR_CMD_PER_CM_S
                   + WHEEL_KP * leftErr
                   + WHEEL_KD * dLeftErr;

    float rightCmdF = vR_target * BASE_MOTOR_CMD_PER_CM_S
                    + WHEEL_KP * rightErr
                    + WHEEL_KD * dRightErr
                    + RIGHT_MOTOR_TRIM;

    g_motors->setSpeeds(
      (int)clampf_local(leftCmdF,  -300.0f, 300.0f),
      (int)clampf_local(rightCmdF, -300.0f, 300.0f)
    );
  }

  void stop()
  {
    if (g_motors == nullptr) return;
    g_motors->setSpeeds(0, 0);
  }
}