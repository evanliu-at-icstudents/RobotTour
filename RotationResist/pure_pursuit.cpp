#include "pure_pursuit.h"
#include <math.h>

namespace
{
  float clampf(float v, float lo, float hi)
  {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  }

  float segmentLength(const PPSegment& seg)
  {
    float dx = seg.p1.x - seg.p0.x;
    float dy = seg.p1.y - seg.p0.y;
    return sqrtf(dx * dx + dy * dy);
  }

  float projectionT(const PPSegment& seg, const Pose& pose)
  {
    float sx = seg.p1.x - seg.p0.x;
    float sy = seg.p1.y - seg.p0.y;
    float segLen2 = sx * sx + sy * sy;
    if (segLen2 < 1e-6f) return 1.0f;

    float rx = pose.x - seg.p0.x;
    float ry = pose.y - seg.p0.y;
    float t  = (rx * sx + ry * sy) / segLen2;
    return clampf(t, 0.0f, 1.0f);
  }

  float projectionTUnclamped(const PPSegment& seg, const Pose& pose)
  {
    float sx = seg.p1.x - seg.p0.x;
    float sy = seg.p1.y - seg.p0.y;
    float segLen2 = sx * sx + sy * sy;
    if (segLen2 < 1e-6f) return 1.0f;

    float rx = pose.x - seg.p0.x;
    float ry = pose.y - seg.p0.y;
    return (rx * sx + ry * sy) / segLen2;
  }

  PPPoint pointAtT(const PPSegment& seg, float t)
  {
    PPPoint p;
    p.x = seg.p0.x + t * (seg.p1.x - seg.p0.x);
    p.y = seg.p0.y + t * (seg.p1.y - seg.p0.y);
    return p;
  }
}

void pure_pursuit::reset() {}

bool pure_pursuit::segmentProjectionPast(const PPSegment& seg, const Pose& p)
{
  float sx = seg.p1.x - seg.p0.x;
  float sy = seg.p1.y - seg.p0.y;

  float denom = sx * sx + sy * sy;
  if (denom < 1e-6f) return true;

  float rx = p.x - seg.p0.x;
  float ry = p.y - seg.p0.y;

  float t = (rx * sx + ry * sy) / denom;

  return t > 1.02f;
}

bool pure_pursuit::segmentCompleteXY(
    const PPSegment& seg,
    const Pose& pose,
    float toleranceX,
    float toleranceY)
{
  return (fabsf(seg.p1.x - pose.x) <= toleranceX) &&
         (fabsf(seg.p1.y - pose.y) <= toleranceY);
}

bool pure_pursuit::pathCompleteXY(const PPSegment* path, int pathLen, const Pose& p,
                    float tolX, float tolY)
{
  if (pathLen <= 0) return true;

  const PPPoint& goal = path[pathLen - 1].p1;

  return (fabsf(p.x - goal.x) <= tolX) &&
         (fabsf(p.y - goal.y) <= tolY);
}

PPPoint pure_pursuit::computeLookaheadPointOnPath(
    const PPSegment* path,
    int pathLen,
    int currentSegmentIndex,
    const Pose& pose,
    float lookaheadCm)
{
  if (pathLen <= 0)   { PPPoint p = {pose.x, pose.y}; return p; }
  if (currentSegmentIndex >= pathLen) { return path[pathLen - 1].p1; }

  int segIdx = currentSegmentIndex;
  const PPSegment& currentSeg = path[segIdx];

  float tStart          = projectionT(currentSeg, pose);
  float currentSegLen   = segmentLength(currentSeg);
  float distAvailable   = (1.0f - tStart) * currentSegLen;

  if (lookaheadCm <= distAvailable)
  {
    float tLook = tStart + (currentSegLen > 1e-6f ? lookaheadCm / currentSegLen : 0.0f);
    return pointAtT(currentSeg, clampf(tLook, 0.0f, 1.0f));
  }

  float remaining = lookaheadCm - distAvailable;
  segIdx++;

  while (segIdx < pathLen)
  {
    const PPSegment& seg = path[segIdx];
    float len = segmentLength(seg);
    if (len < 1e-6f) { segIdx++; continue; }

    if (remaining <= len)
    {
      float tLook = clampf(remaining / len, 0.0f, 1.0f);
      return pointAtT(seg, tLook);
    }

    remaining -= len;
    segIdx++;
  }

  return path[pathLen - 1].p1;
}

float pure_pursuit::computeCurvature(const Pose& pose, const PPPoint& lookahead)
{
  float dx = lookahead.x - pose.x;
  float dy = lookahead.y - pose.y;

  float cosT =  cosf(pose.theta);
  float sinT =  sinf(pose.theta);

  float xLocal =  cosT * dx + sinT * dy;
  float yLocal = -sinT * dx + cosT * dy;

  float ld2 = xLocal * xLocal + yLocal * yLocal;
  if (ld2 < 1e-6f) return 0.0f;

  return 2.0f * yLocal / ld2;
}

float pure_pursuit::computeAngularVelocity(
    const Pose& pose,
    const PPSegment* path,
    int pathLen,
    int currentSegmentIndex,
    float lookaheadCm,
    float forwardSpeedCmS,
    float maxAngularSpeedRadS)
{
  PPPoint look = computeLookaheadPointOnPath(
      path, pathLen, currentSegmentIndex, pose, lookaheadCm);

  float kappa = computeCurvature(pose, look);
  float w     = forwardSpeedCmS * kappa;

  if (w >  maxAngularSpeedRadS) return  maxAngularSpeedRadS;
  if (w < -maxAngularSpeedRadS) return -maxAngularSpeedRadS;
  return w;
}

float pure_pursuit::remainingDistanceAlongPath(
    const PPSegment* path,
    int pathLen,
    int currentSegmentIndex,
    const Pose& pose)
{
  if (pathLen <= 0 || currentSegmentIndex >= pathLen) return 0.0f;

  float total = 0.0f;
  const PPSegment& currentSeg = path[currentSegmentIndex];
  float t = projectionT(currentSeg, pose);
  total += (1.0f - t) * segmentLength(currentSeg);

  for (int i = currentSegmentIndex + 1; i < pathLen; i++)
    total += segmentLength(path[i]);

  return total;
}