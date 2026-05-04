#include "Haptics.h"
#include <Wire.h>
#include <Adafruit_DRV2605.h>

#define TCA_ADDR       0x70
#define DRV2605_ADDR   0x5A

#define HAPTIC_RIGHT_CH 6
#define HAPTIC_LEFT_CH  7

Adafruit_DRV2605 drvLeft;
Adafruit_DRV2605 drvRight;

// --------------------------------------------------------
// selectMux — same two-step pattern as ToFSensor
// --------------------------------------------------------
static void selectMux(uint8_t channel)
{
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();

    delayMicroseconds(100);
}

// --------------------------------------------------------
// hapticsInit
// FIX: removed while(1) halts — replaced with return + Serial warn.
// A missing haptic driver should NOT brick the entire system.
// FIX: stopHaptics() called after init to ensure RTP value starts at 0.
// --------------------------------------------------------
void hapticsInit()
{
    delay(10);

    // LEFT driver
    selectMux(HAPTIC_LEFT_CH);
    if (!drvLeft.begin()) {
        Serial.println("WARNING: Left haptic not detected — continuing");
        // Do NOT while(1) — system can still operate with one motor
    } else {
        drvLeft.setMode(DRV2605_MODE_REALTIME);
        drvLeft.setRealtimeValue(0);   // FIX: ensure motor is off after init
    }

    delay(10);

    // RIGHT driver
    selectMux(HAPTIC_RIGHT_CH);
    if (!drvRight.begin()) {
        Serial.println("WARNING: Right haptic not detected — continuing");
    } else {
        drvRight.setMode(DRV2605_MODE_REALTIME);
        drvRight.setRealtimeValue(0);  // FIX: ensure motor is off after init
    }

    Serial.println("Haptics initialized (RTP mode)");
}

// --------------------------------------------------------
// vibrateLeft
// --------------------------------------------------------
void vibrateLeft(uint8_t strength)
{
    selectMux(HAPTIC_LEFT_CH);
    drvLeft.setRealtimeValue(strength);
}

// --------------------------------------------------------
// vibrateRight
// --------------------------------------------------------
void vibrateRight(uint8_t strength)
{
    selectMux(HAPTIC_RIGHT_CH);
    drvRight.setRealtimeValue(strength);
}

// --------------------------------------------------------
// stopHaptics — zero both motors
// FIX: select each channel individually; don't assume previous channel is active
// --------------------------------------------------------
void stopHaptics()
{
    selectMux(HAPTIC_LEFT_CH);
    drvLeft.setRealtimeValue(0);

    selectMux(HAPTIC_RIGHT_CH);
    drvRight.setRealtimeValue(0);
}

// --------------------------------------------------------
// vibrateStop — short double pulse on both motors
// FIX: two pulses separated by gap; original was single long buzz
// --------------------------------------------------------
void vibrateStop()
{
    // Pulse 1
    vibrateLeft(160);
    vibrateRight(160);
    delay(150);
    stopHaptics();
    delay(100);
    // Pulse 2
    vibrateLeft(160);
    vibrateRight(160);
    delay(150);
    stopHaptics();
}

// --------------------------------------------------------
// vibrateBoot — both motors together, single pulse
// --------------------------------------------------------
void vibrateBoot()
{
    vibrateLeft(120);
    vibrateRight(120);
    delay(400);
    stopHaptics();
}

// --------------------------------------------------------
// checkHapticLeft / checkHapticRight — I2C presence check
// --------------------------------------------------------
bool checkHapticLeft()
{
    selectMux(HAPTIC_LEFT_CH);
    Wire.beginTransmission(DRV2605_ADDR);
    return (Wire.endTransmission() == 0);
}

bool checkHapticRight()
{
    selectMux(HAPTIC_RIGHT_CH);
    Wire.beginTransmission(DRV2605_ADDR);
    return (Wire.endTransmission() == 0);
}
