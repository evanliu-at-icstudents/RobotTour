/**
 * @file RomiRobot.ino
 * @brief Romi 32U4 Robot controller with gyro calibration and script parsing.
 * * This program allows for manual gyro calibration (Button A/B) and executes
 * movement scripts defined in mission.h via a command parser (Button C).
 */

#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "TurnSensor.h"
#include "mission.h"

/** @brief EEPROM address where the errorPer90 calibration value is stored. */
const int EEPROM_ADDR = 0;

/** @brief Maximum motor speed (0-300). */
const int16_t maxSpeed = 300;

/** @brief Encoder counts per centimeter of travel. */
const int32_t countsPerCm = 65;

/** @brief Distance target for the MOVE command (50cm). */
const int32_t dist50cm = 50 * countsPerCm;

Romi32U4LCD lcd;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Motors motors;
Romi32U4Encoders encoders;
LSM6 imu;

/** @brief The heading the robot is currently trying to maintain. */
int32_t targetAngle = 0;

/** @brief Calibration offset applied to every 90-degree turn to counter gyro drift. */
int32_t errorPer90 = 0;

/** @brief State flag to stop PID corrections during manual repositioning. */
bool isWaitingForManualReset = false;

// --- Core Logic Functions ---

/**
 * @brief Performs a stationary turn-in-place to face the current targetAngle.
 * * Uses a basic PD control loop involving current turnAngle and turnRate.
 */
void performTurn() {
  turnSensorUpdate();
  int32_t error = targetAngle - turnAngle;
  int32_t turnSpeed = (error / (turnAngle1 / 15)) - (turnRate / 73);
  turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
  motors.setSpeeds(-turnSpeed, turnSpeed);
}

/**
 * @brief Drives the robot forward while using the gyro to maintain heading.
 * @param forwardSpeed The base speed for both motors.
 * * Adjusts individual motor speeds based on deviations from targetAngle.
 */
void driveStraight(int16_t forwardSpeed) {
  turnSensorUpdate();
  int32_t error = targetAngle - turnAngle;
  int32_t turnSpeed = (error / (turnAngle1 / 15)) - (turnRate / 73);
  motors.setSpeeds(forwardSpeed - turnSpeed, forwardSpeed + turnSpeed);
}

// --- Parser Command Wrappers ---

/**
 * @brief Executes a forward movement command.
 * * Resets encoders and drives 50cm while correcting heading.
 */
void executeMove() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  while (((encoders.getCountsLeft() + encoders.getCountsRight()) / 2) < dist50cm) {
    driveStraight(150);
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

/**
 * @brief Executes a 90-degree turn command.
 * @param isRight True for clockwise (Right), False for counter-clockwise (Left).
 * * Incorporates the calibrated errorPer90 into the turn calculation.
 */
void executeTurn(bool isRight) {
  if (isRight) targetAngle += (turnAngle90 + errorPer90);
  else         targetAngle -= (turnAngle90 + errorPer90);

  uint32_t turnStart = millis();
  while (millis() - turnStart < 2000) {
    performTurn();
    if (abs(targetAngle - turnAngle) < 100 && abs(turnRate) < 10) break;
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

// --- The Parser ---

/**
 * @brief Parses the mission script from mission.h and executes movements.
 * * Copies script from Flash to RAM and uses strtok to process individual tokens.
 */
void parseAndRunMission() {
  char buffer[128]; 
  strncpy_P(buffer, mission_run, sizeof(buffer));
  
  char* command = strtok(buffer, "\n\r ");
  
  while (command != NULL) {
    lcd.clear();
    lcd.print(command);

    if (strcmp(command, "MOVE") == 0) {
      executeMove();
    } 
    else if (strcmp(command, "RIGHT") == 0) {
      executeTurn(true);
    } 
    else if (strcmp(command, "LEFT") == 0) {
      executeTurn(false);
    }

    command = strtok(NULL, "\n\r ");
  }
  
  lcd.clear();
  lcd.print(F("Done"));
}

/**
 * @brief Standard Arduino setup function.
 * * Initializes sensors, loads calibration from EEPROM, and displays status.
 */
void setup() {
  turnSensorSetup();
  delay(500);
  turnSensorReset();

  EEPROM.get(EEPROM_ADDR, errorPer90);
  if (abs(errorPer90) > 200000) errorPer90 = 0;

  lcd.clear();
  lcd.print(F("Err:"));
  lcd.print(errorPer90);
  lcd.gotoXY(0, 1);
  lcd.print(F("A:Cal B:S C:Run"));
}

/**
 * @brief Standard Arduino loop function.
 * * Handles button presses for calibration and mission execution.
 */
void loop() {
  turnSensorUpdate();

  // Button A: Calibration (12 Turns)
  if (buttonA.getSingleDebouncedPress()) {
    lcd.clear(); lcd.print(F("Testing"));
    targetAngle = 0;
    turnSensorReset();
    delay(1000);
    for (int i = 0; i < 12; i++) {
      targetAngle += (turnAngle90 + errorPer90); 
      uint32_t turnStart = millis();
      while (millis() - turnStart < 2000) {
        performTurn();
        if (abs(targetAngle - turnAngle) < 100 && abs(turnRate) < 10) break;
      }
      motors.setSpeeds(0, 0);
      lcd.gotoXY(0, 1); lcd.print(i + 1); lcd.print(F("/12"));
      delay(400);
    }
    motors.setSpeeds(0, 0);
    isWaitingForManualReset = true;
    lcd.clear(); lcd.print(F("Stopped"));
  }

  // Button B: Cumulative Save to EEPROM
  if (buttonB.getSingleDebouncedPress()) {
    delay(500);
    int32_t totalDrift = turnAngle - targetAngle;
    errorPer90 += (totalDrift / 12); 
    EEPROM.put(EEPROM_ADDR, errorPer90);
    turnSensorReset();
    targetAngle = 0;
    isWaitingForManualReset = false;
    lcd.clear(); lcd.print(F("Saved!"));
    motors.setSpeeds(100, -100); delay(100);
    motors.setSpeeds(-100, 100); delay(100);
    motors.setSpeeds(0, 0);
  }

  // Button C: Run the Mission script
  if (buttonC.getSingleDebouncedPress()) {
    turnSensorReset();
    targetAngle = 0;
    parseAndRunMission();
  }

  // Orientation Correction
  if (!isWaitingForManualReset) {
    performTurn();
  } else {
    motors.setSpeeds(0, 0);
  }
}