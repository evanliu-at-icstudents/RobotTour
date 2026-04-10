#include "path_builder.h"
#include <math.h>
#include <string.h>
#include <avr/pgmspace.h>

static float clampf_local(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float len2D(float dx, float dy)
{
  return sqrtf(dx * dx + dy * dy);
}

static float wrapAngle(float a)
{
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

PPPoint readWaypoint(const PPPoint* waypointArray, int i)
{
  PPPoint p;
  memcpy_P(&p, &waypointArray[i], sizeof(PPPoint));
  return p;
}

float computePathLength(const PPSegment* segs, int len)
{
  float total = 0.0f;

  for (int i = 0; i < len; i++)
  {
    float dx = segs[i].p1.x - segs[i].p0.x;
    float dy = segs[i].p1.y - segs[i].p0.y;
    total += sqrtf(dx * dx + dy * dy);
  }

  return total;
}

void buildPath(const PPPoint* waypoints, int numWaypoints,
               float cornerRadius, int arcSegs,
               PPSegment* outPath, int* outLen, int maxPathSegments)
{
  *outLen = 0;
  if (numWaypoints < 2) return;

  auto pushSeg = [&](PPPoint from, PPPoint to)
  {
    if (*outLen >= maxPathSegments) return;
    float d = len2D(to.x - from.x, to.y - from.y);
    if (d < 1e-4f) return;
    outPath[(*outLen)++] = { from, to };
  };

  PPPoint cursor = readWaypoint(waypoints, 0);

  for (int i = 1; i < numWaypoints; i++)
  {
    PPPoint cur = readWaypoint(waypoints, i);

    if (i == numWaypoints - 1)
    {
      pushSeg(cursor, cur);
      break;
    }

    PPPoint next = readWaypoint(waypoints, i + 1);

    float dx1 = cur.x  - cursor.x;
    float dy1 = cur.y  - cursor.y;
    float l1  = len2D(dx1, dy1);

    float dx2 = next.x - cur.x;
    float dy2 = next.y - cur.y;
    float l2  = len2D(dx2, dy2);

    if (l1 < 1e-6f || l2 < 1e-6f)
    {
      pushSeg(cursor, cur);
      cursor = cur;
      continue;
    }

    float u1x = dx1 / l1,  u1y = dy1 / l1;
    float u2x = dx2 / l2,  u2y = dy2 / l2;

    float cosA = clampf_local((-u1x) * u2x + (-u1y) * u2y, -1.0f, 1.0f);
    float intAngle = acosf(cosA);

    if (intAngle < 0.02f)
    {
      pushSeg(cursor, cur);
      cursor = cur;
      continue;
    }

    float trimDist = cornerRadius / tanf(intAngle / 2.0f);
    float maxTrim  = 0.45f * fminf(l1, l2);
    if (trimDist > maxTrim) trimDist = maxTrim;

    float actualR = trimDist * tanf(intAngle / 2.0f);

    PPPoint T1 = { cur.x - trimDist * u1x, cur.y - trimDist * u1y };
    PPPoint T2 = { cur.x + trimDist * u2x, cur.y + trimDist * u2y };

    float crossZ  = u1x * u2y - u1y * u2x;
    float sign    = (crossZ >= 0.0f) ? 1.0f : -1.0f;
    float cx = T1.x + actualR * sign * (-u1y);
    float cy = T1.y + actualR * sign * ( u1x);

    float a1    = atan2f(T1.y - cy, T1.x - cx);
    float a2    = atan2f(T2.y - cy, T2.x - cx);
    float sweep = wrapAngle(a2 - a1);

    if (crossZ >= 0.0f && sweep < 0.0f) sweep += 2.0f * (float)M_PI;
    if (crossZ <  0.0f && sweep > 0.0f) sweep -= 2.0f * (float)M_PI;

    pushSeg(cursor, T1);

    PPPoint arcPrev = T1;
    for (int s = 1; s <= arcSegs; s++)
    {
      float angle = a1 + sweep * ((float)s / (float)arcSegs);
      PPPoint arcPt = {
        cx + actualR * cosf(angle),
        cy + actualR * sinf(angle)
      };
      pushSeg(arcPrev, arcPt);
      arcPrev = arcPt;
    }

    cursor = T2;
  }
}