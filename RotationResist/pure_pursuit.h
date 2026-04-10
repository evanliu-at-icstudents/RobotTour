#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "odometry.h"

struct PPPoint
{
  float x;  //cm
  float y;  //cm
};

struct PPSegment
{
  PPPoint p0;
  PPPoint p1;
};

namespace pure_pursuit
{
  void reset();

  PPPoint computeLookaheadPointOnPath(
      const PPSegment* path,
      int pathLen,
      int currentSegmentIndex,
      const Pose& pose,
      float lookaheadCm);

  float computeCurvature(const Pose& pose, const PPPoint& lookahead);

  float computeAngularVelocity(
      const Pose& pose,
      const PPSegment* path,
      int pathLen,
      int currentSegmentIndex,
      float lookaheadCm,
      float forwardSpeedCmS,
      float maxAngularSpeedRadS);

  bool segmentCompleteXY(
      const PPSegment& seg,
      const Pose& pose,
      float toleranceX,
      float toleranceY);

  bool pathCompleteXY(
      const PPSegment* path,
      int pathLen,
      const Pose& pose,
      float toleranceX,
      float toleranceY);

  float remainingDistanceAlongPath(
      const PPSegment* path,
      int pathLen,
      int currentSegmentIndex,
      const Pose& pose);
  bool segmentProjectionPast(const PPSegment& seg, const Pose& pose);
}

#endif