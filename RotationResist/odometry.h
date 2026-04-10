#ifndef ODOMETRY_H
#define ODOMETRY_H

struct Pose
{
  float x;      // cm
  float y;      // cm
  float theta;  // radians for getPose(), degrees for getPoseDegrees()
};

namespace odometry
{
  void initialize();
  void resetPose(float x, float y, float headingDeg = 0.0f);
  void update();

  Pose getPose();
  Pose getPoseDegrees();

  float getLeftVelocityCmS();
  float getRightVelocityCmS();
}

constexpr float ODOM_COUNTS_PER_CM = 65.0f;
constexpr float TRACK_WIDTH_CM = 14.0f;
constexpr float HEADING_FUSION_ALPHA = 0.95f;

#endif