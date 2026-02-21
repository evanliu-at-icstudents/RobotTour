/**
 * @file mission.h
 * @brief Defines the movement script for the Romi robot mission.
 */

#ifndef MISSION_H
#define MISSION_H

#include <avr/pgmspace.h>

/**
 * @brief Raw mission script stored in Flash memory.
 * * Commands supported by the parser:
 * - MOVE: Drives forward 50cm using gyro-stabilization.
 * - LEFT: Turns 90 degrees left using calibrated offset.
 * - RIGHT: Turns 90 degrees right using calibrated offset.
 */
const char mission_run[] PROGMEM = R"=====(
MOVE
RIGHT
MOVE
LEFT
MOVE
RIGHT
)=====";

#endif