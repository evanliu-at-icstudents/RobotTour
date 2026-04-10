#ifndef PATH_BUILDER_H
#define PATH_BUILDER_H

#include <Arduino.h>
#include "pure_pursuit.h"

PPPoint readWaypoint(const PPPoint* waypointArray, int i);

void buildPath(const PPPoint* waypoints, int numWaypoints,
               float cornerRadius, int arcSegs,
               PPSegment* outPath, int* outLen, int maxPathSegments);

float computePathLength(const PPSegment* segs, int len);

#endif