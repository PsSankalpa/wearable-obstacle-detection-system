#ifndef HAPTICS_H
#define HAPTICS_H

#include <Arduino.h>


// Initialize both haptic drivers
void hapticsInit();

// Continuous vibration (0–255)
void vibrateLeft(uint8_t strength);
void vibrateRight(uint8_t strength);

// Stop all haptics
void stopHaptics();

// STOP pattern (short warning)
void vibrateStop();

//  To give signle after bootup
void vibrateBoot();

// To check haptics are working
bool checkHapticLeft();
bool checkHapticRight();

#endif
