#include <Romi32U4.h>
#include <math.h>
#include <string.h>

#include "odometry.h"
#include "pure_pursuit.h"
#include "path_builder.h"
#include "motor_interface.h"

Romi32U4Motors motors;
Romi32U4Encoders encoders;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4Buzzer buzzer;

// ---------------------------
// Control / robot constants (EDIT HERE)
// ---------------------------
const unsigned long LOOP_DT_MS = 10;      // ignore
const float DESIRED_TIME = 10.0f;         // Desired time to reach end

const float TRACK_WIDTH         = 14.1f;  // ignore: robot is already tuned
const float MIN_REM             = 0.1f;   // ignore: robot is already tuned
const float MAX_W_RAD_S         = 2.0f;   // ignore: robot is already tuned
const float LOOKAHEAD_CM        = 20.0f;  // Control of how tight turns are

const float END_TOLERANCE_X_CM  = 0.25f;  // ignore
const float END_TOLERANCE_Y_CM  = 0.25f;  // ignore

const float MIN_TIME_LEFT_S     = 0.20f;  // ignore
const float MIN_FORWARD_CM_S    = 2.0f;   // ignore: robot is already tuned
const float MAX_FORWARD_CM_S    = 30.0f;  // ignore: robot is already tuned

const float CORNER_RADIUS = 12.5f;        // Radius of turns
const int   ARC_SEGMENTS  = 6;            // ignore

bool beepMode = false;                    // Whether or not beepMode is ON or OFF by default

enum TravelMode                           //ignore
{
  TRAVEL_FORWARD = 1,
  TRAVEL_REVERSE = -1
};

struct PathDef                            //ignore
{
  const PPPoint* waypoints;
  int numWaypoints;
  TravelMode mode;
};

// COPY/PASTE INTO ROBOT CODE

static const PPPoint waypoints1[] PROGMEM = {
  {   0.0f,   0.0f },
  {  50.0f,   0.0f },
  {  50.0f,  50.0f },
};

static const PPPoint waypoints2[] PROGMEM = {
  {  50.0f,  50.0f },
  {  50.0f,   0.0f },
  {   0.0f,   0.0f },
};

static const PathDef paths[] = {
  { waypoints1, (int)(sizeof(waypoints1) / sizeof(waypoints1[0])), TRAVEL_FORWARD },
  { waypoints2, (int)(sizeof(waypoints2) / sizeof(waypoints2[0])), TRAVEL_REVERSE },
};

const int NUM_PATHS = (int)(sizeof(paths) / sizeof(paths[0]));
const int MAX_WAYPOINTS = 3;

// ---------------------------
// Main (DO NOT EDIT)
// ---------------------------

#define MAX_PATH_SEGMENTS \
  ((MAX_WAYPOINTS - 1) + (MAX_WAYPOINTS - 2) * ARC_SEGMENTS + 2)

PPSegment path[MAX_PATH_SEGMENTS];
int PATH_LEN = 0;

float pathLengths[NUM_PATHS];
float totalMissionDistance = 0.0f;

int   currentSegmentIndex = 0;
int   currentPathIndex    = 0;
int   travelSign          = 1;

bool  started      = false;
bool  finished     = false;

unsigned long lastLoopMs  = 0;
unsigned long startTimeMs = 0;

float commandedForwardCmS = 0.0f;
float timeLeftS = 0.0f;
float totalRemainingCm = 0.0f;

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void playChirp(const char* tune)
{
  if (!beepMode) return;
  buzzer.play(tune);
}

void signalBeepEnabled()
{
  buzzer.play(">g16>>c16");
}

void signalBeepDisabled()
{
  buzzer.play(">>c16>g16");
}

void handleBeepToggle()
{
  if (buttonB.getSingleDebouncedPress())
  {
    beepMode = !beepMode;

    if (beepMode)
    {
      signalBeepEnabled();
      Serial.println(F("Beep mode: ON"));
    }
    else
    {
      signalBeepDisabled();
      Serial.println(F("Beep mode: OFF"));
    }
  }
}

void computeMissionLengths()
{
  totalMissionDistance = 0.0f;

  PPSegment tempPath[MAX_PATH_SEGMENTS];
  int tempLen = 0;

  for (int i = 0; i < NUM_PATHS; i++)
  {
    buildPath(paths[i].waypoints, paths[i].numWaypoints,
              CORNER_RADIUS, ARC_SEGMENTS,
              tempPath, &tempLen, MAX_PATH_SEGMENTS);

    pathLengths[i] = computePathLength(tempPath, tempLen);
    totalMissionDistance += pathLengths[i];
  }
}

float computeTotalRemainingDistance(const Pose& p)
{
  float remaining = 0.0f;

  if (currentPathIndex < NUM_PATHS)
  {
    remaining += pure_pursuit::remainingDistanceAlongPath(
        path, PATH_LEN, currentSegmentIndex, p);
  }

  for (int i = currentPathIndex + 1; i < NUM_PATHS; i++)
  {
    remaining += pathLengths[i];
  }

  return remaining;
}

void loadCurrentPath()
{
  const PathDef& pd = paths[currentPathIndex];

  buildPath(pd.waypoints, pd.numWaypoints,
            CORNER_RADIUS, ARC_SEGMENTS,
            path, &PATH_LEN, MAX_PATH_SEGMENTS);

  travelSign = (int)pd.mode;
  currentSegmentIndex = 0;
  motor_interface::resetController();

  if (travelSign > 0)
  {
    playChirp(">c16>g16");
  }
  else
  {
    playChirp(">g16>c16");
  }

  Serial.print(F("Loaded path "));
  Serial.print(currentPathIndex);
  Serial.print(F(" mode="));
  Serial.print((travelSign > 0) ? F("FORWARD") : F("REVERSE"));
  Serial.print(F(" segments="));
  Serial.print(PATH_LEN);
  Serial.print(F(" length="));
  Serial.println(pathLengths[currentPathIndex], 2);
}

void advanceToNextPathOrFinish()
{
  motor_interface::stop();

  Serial.println(F("Path complete. Pausing..."));
  playChirp("l16>e g");

  delay(500);

  currentPathIndex++;

  if (currentPathIndex >= NUM_PATHS)
  {
    finished = true;
    Serial.println(F("All paths complete."));
    playChirp("l16>c>e>g>c");
    return;
  }

  loadCurrentPath();
  Serial.println(F("Switching to next path."));
  playChirp("l16>a>c");
}

// ---------------------------
// Setup / loop
// ---------------------------
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("RotationResist v1");
  Serial.println("-----------------");
  Serial.println(F("Press A to start."));
  Serial.println(F("Press B to toggle beep mode (default OFF)."));

  while (true)
  {
    handleBeepToggle();

    if (buttonA.getSingleDebouncedPress())
    {
      break;
    }
  }

  delay(500);

  playChirp("l16>c>e>g");

  odometry::initialize();
  pure_pursuit::reset();

  motor_interface::begin(&motors, TRACK_WIDTH);

  computeMissionLengths();

  Serial.print(F("Total mission distance: "));
  Serial.println(totalMissionDistance, 2);

  currentSegmentIndex = 0;
  currentPathIndex = 0;
  finished = false;

  loadCurrentPath();

  lastLoopMs = millis();
  startTimeMs = lastLoopMs;
  started = true;
}

void loop()
{
  handleBeepToggle();

  if (!started || finished) return;

  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_DT_MS) return;

  float dt = (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;

  odometry::update();
  Pose p = odometry::getPose();

  if (pure_pursuit::pathCompleteXY(
        path, PATH_LEN, p,
        END_TOLERANCE_X_CM, END_TOLERANCE_Y_CM))
  {
    advanceToNextPathOrFinish();
    return;
  }

  while (currentSegmentIndex < PATH_LEN &&
         pure_pursuit::segmentProjectionPast(path[currentSegmentIndex], p))
  {
    currentSegmentIndex++;
  }

  float remCurrent = pure_pursuit::remainingDistanceAlongPath(
      path, PATH_LEN, currentSegmentIndex, p);

  totalRemainingCm = computeTotalRemainingDistance(p);
  float rem_safe = fmax(totalRemainingCm, MIN_REM);

  float elapsed_s = (now - startTimeMs) / 1000.0f;
  timeLeftS = DESIRED_TIME - elapsed_s;
  float time_left_safe = fmax(timeLeftS, MIN_TIME_LEFT_S);

  commandedForwardCmS = rem_safe / time_left_safe;
  commandedForwardCmS = clampf(
      commandedForwardCmS,
      MIN_FORWARD_CM_S,
      MAX_FORWARD_CM_S);

  float effectiveLookahead = remCurrent * 0.7f;
  if (effectiveLookahead > LOOKAHEAD_CM) effectiveLookahead = LOOKAHEAD_CM;
  if (effectiveLookahead < 3.0f)         effectiveLookahead = 3.0f;

  float w = pure_pursuit::computeAngularVelocity(
      p, path, PATH_LEN, currentSegmentIndex,
      effectiveLookahead, commandedForwardCmS, MAX_W_RAD_S);

  if (travelSign < 0)
  {
    w = -w;
  }

  float commandedV = travelSign * commandedForwardCmS;
  motor_interface::setWheelSpeed(commandedV, w, dt);

  static int printCounter = 0;
  if (++printCounter >= 10)
  {
    printCounter = 0;

    Pose pd = odometry::getPoseDegrees();
    PPPoint look = pure_pursuit::computeLookaheadPointOnPath(
        path, PATH_LEN, currentSegmentIndex, p, effectiveLookahead);
    float kappa = pure_pursuit::computeCurvature(p, look);

    Serial.print(F("path:"));       Serial.print(currentPathIndex);
    Serial.print(F(" seg:"));       Serial.print(currentSegmentIndex);
    Serial.print(F(" mode:"));      Serial.print((travelSign > 0) ? F("FWD") : F("REV"));
    Serial.print(F(" x:"));         Serial.print(pd.x, 2);
    Serial.print(F(" y:"));         Serial.print(pd.y, 2);
    Serial.print(F(" th:"));        Serial.print(pd.theta, 2);
    Serial.print(F(" remCur:"));    Serial.print(remCurrent, 2);
    Serial.print(F(" remTot:"));    Serial.print(totalRemainingCm, 2);
    Serial.print(F(" elapsed:"));   Serial.print(elapsed_s, 2);
    Serial.print(F(" tleft:"));     Serial.print(timeLeftS, 2);
    Serial.print(F(" vmag:"));      Serial.print(commandedForwardCmS, 2);
    Serial.print(F(" vcmd:"));      Serial.print(commandedV, 2);
    Serial.print(F(" k:"));         Serial.print(kappa, 4);
    Serial.print(F(" w:"));         Serial.println(w, 3);
  }
}