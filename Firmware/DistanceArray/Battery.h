#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// Initialize battery monitoring hardware
void batteryInit();

// Read actual battery voltage (in volts)
float readBatteryVoltage();

// Check if battery is below safe threshold
bool isBatteryLow();

#endif
