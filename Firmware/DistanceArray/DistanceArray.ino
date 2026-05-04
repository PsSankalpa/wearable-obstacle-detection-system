/*
 * DistanceArray.ino — ESP32 Multi-Sensor Guidance System
 * FIXED + DEBUG VERSION
 *
 * Serial output format (115200 baud):
 *
 *   [BOOT]
 *   ===================================
 *      Distance Array — Booting...
 *   ===================================
 *   Init CENTER sensor (ch0)... OK
 *   Init RIGHT sensor (ch4)...  OK
 *   Init LEFT sensor (ch2)...   OK
 *   All sensors initialized
 *   Center sensor ranging started
 *   Haptics initialized (RTP mode)
 *   ===================================
 *           System READY
 *   ===================================
 *
 *   [NORMAL LOOP — every frame ~200ms at 5Hz]
 *   --------------------------------------------------
 *   Frame #42
 *   CENTER: 1823mm  LEFT: 1650mm  RIGHT:  720mm
 *   Left safe: YES  Right safe: YES
 *   Decision: MOVE_FORWARD
 *   (no change)
 *   --------------------------------------------------
 *
 *   [ON DECISION CHANGE]
 *   >>> Decision changed: MOVE_FORWARD -> MOVE_LEFT
 *   [Side Validation] Switching to LEFT sensor...
 *   [Side Validation] Waiting 1 frame at 5Hz (~220ms)...
 *   [Side Validation] Frame 1: avg=1342mm  bottomSafe=YES  stableCount=1
 *   [Side Validation] Frame 2: avg=1510mm  bottomSafe=YES  stableCount=2
 *   [Side Validation] Path CLEAR — vibrating LEFT motor
 *   [Side Validation] Returned to CENTER sensor
 *
 *   [BATTERY — every 10s]
 *   Battery: 3.84V  [OK]
 *   Battery: 3.18V  [LOW — entering deep sleep]
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "Decisions.h"
#include "Haptics.h"
#include "Battery.h"
#include "ToFSensor.h"
#include "esp_sleep.h"

// -------------------------------------------------------
// USB_DEBUG_MODE: Comment out this line when running on battery.
// While defined, battery monitoring is skipped so a floating GPIO34
// (no Li-Po connected during USB development) does not trigger deep sleep.
// -------------------------------------------------------
// #define USB_DEBUG_MODE

#define STATUS_LED_PIN 2
#define TCA_ADDR       0x70

#define CENTER_LPN 18
#define RIGHT_LPN  25
#define LEFT_LPN   32

#define CENTER_CH 0
#define RIGHT_CH  4
#define LEFT_CH   2

#define BATTERY_CHECK_INTERVAL_MS 10000UL

const int imageWidth = 8;

ToFSensor centerSensor(CENTER_CH, CENTER_LPN);
ToFSensor leftSensor  (LEFT_CH,   LEFT_LPN);
ToFSensor rightSensor (RIGHT_CH,  RIGHT_LPN);

Decision lastDecision = STOP;

// Reminder pulse: re-vibrate if same LEFT/RIGHT decision persists > 2s
// Tracks when the current directional decision was first issued
unsigned long lastDirectionVibrateTime = 0;
const unsigned long REMINDER_INTERVAL_MS = 2000UL;

// --------------------------------------------------------
// Decision name helper
// --------------------------------------------------------
static const char* decName(Decision d) {
  switch (d) {
    case MOVE_FORWARD: return "MOVE_FORWARD";
    case MOVE_LEFT:    return "MOVE_LEFT";
    case MOVE_RIGHT:   return "MOVE_RIGHT";
    case STOP:         return "STOP";
    default:           return "UNKNOWN";
  }
}

// --------------------------------------------------------
// Mux helper
// --------------------------------------------------------
static void muxDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delayMicroseconds(100);
}

// --------------------------------------------------------
// Status LED
// --------------------------------------------------------
void blinkReadyLED() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH); delay(150);
    digitalWrite(STATUS_LED_PIN, LOW);  delay(150);
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===================================");
  Serial.println("   Distance Array -- Booting...");
  Serial.println("===================================");

  batteryInit();

  Wire.begin(21, 22);
  Wire.setClock(100000);
  Wire.setTimeOut(50);

  // All LPN LOW — prevents I2C address collision during init.
  // All three VL53L5CX share address 0x52; only one can be powered at a time.
  pinMode(CENTER_LPN, OUTPUT); digitalWrite(CENTER_LPN, LOW);
  pinMode(RIGHT_LPN,  OUTPUT); digitalWrite(RIGHT_LPN,  LOW);
  pinMode(LEFT_LPN,   OUTPUT); digitalWrite(LEFT_LPN,   LOW);
  delay(50);

  muxDisableAll();

  // ---- Init CENTER sensor with retry ----
Serial.print("Init CENTER sensor (ch0)... ");
{
  int attempts = 0;
  bool ok = false;
  while (attempts < 5 && !ok) {
    if (attempts > 0) {
      Serial.print("[retry] ");
      muxDisableAll();
      digitalWrite(CENTER_LPN, LOW);
      delay(200);
    }
    ok = centerSensor.init();
    attempts++;
  }
  if (!ok) {
    Serial.println("FAILED after 5 attempts.");
    Serial.println("FATAL: Cannot continue without center sensor.");
    while (1) { delay(1000); }
  }
  Serial.println("OK");
}
digitalWrite(CENTER_LPN, LOW);
muxDisableAll();
delay(50);

// ---- Init RIGHT sensor with retry ----
Serial.print("Init RIGHT sensor (ch4)...  ");
{
  int attempts = 0;
  bool ok = false;
  while (attempts < 5 && !ok) {
    if (attempts > 0) {
      Serial.print("[retry] ");
      muxDisableAll();
      digitalWrite(RIGHT_LPN, LOW);
      delay(200);
    }
    ok = rightSensor.init();
    attempts++;
  }
  if (!ok) {
    Serial.println("FAILED after 5 attempts.");
    Serial.println("FATAL: Cannot continue without right sensor.");
    while (1) { delay(1000); }
  }
  Serial.println("OK");
}
digitalWrite(RIGHT_LPN, LOW);
muxDisableAll();
delay(50);

// ---- Init LEFT sensor with retry ----
Serial.print("Init LEFT sensor (ch2)...   ");
{
  int attempts = 0;
  bool ok = false;
  while (attempts < 5 && !ok) {
    if (attempts > 0) {
      Serial.print("[retry] ");
      muxDisableAll();
      digitalWrite(LEFT_LPN, LOW);
      delay(200);
    }
    ok = leftSensor.init();
    attempts++;
  }
  if (!ok) {
    Serial.println("FAILED after 5 attempts.");
    Serial.println("FATAL: Cannot continue without left sensor.");
    while (1) { delay(1000); }
  }
  Serial.println("OK");
}
digitalWrite(LEFT_LPN, LOW);
muxDisableAll();
delay(50);

  Serial.println("-----------------------------------");
  Serial.println("All sensors initialized");

  // Power up and start center sensor for normal operation
  digitalWrite(CENTER_LPN, HIGH);
  delay(10);
  centerSensor.start();
  Serial.println("Center sensor ranging started");

  // Haptics init
  hapticsInit();
  blinkReadyLED();
  vibrateBoot();

  Serial.println("===================================");
  Serial.println("         System READY");
  Serial.println("===================================\n");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  static int16_t       frame[64];
  static unsigned long lastBatteryCheck = 0;
  static uint32_t      frameCount       = 0;

  // ---- Battery check every 10 s ----
  // Skipped in USB_DEBUG_MODE: GPIO34 floats when no Li-Po is connected,
  // reading ~0-2V and falsely triggering deep sleep.
#ifndef USB_DEBUG_MODE
  if (millis() - lastBatteryCheck > BATTERY_CHECK_INTERVAL_MS) {
    lastBatteryCheck = millis();
    float v = readBatteryVoltage();

    // Floating pin guard: real Li-Po can never read below 2.5V while ESP32 runs
    if (v < 2.5f) {
      Serial.printf("Battery: %.2fV  [INVALID -- ADC floating or wiring fault, skipping]\n", v);
    }
    // isBatteryLow() uses 5 consecutive low readings before returning true (debounce)
    else if (isBatteryLow()) {
      Serial.printf("Battery: %.2fV  [LOW -- 5 consecutive low reads, entering deep sleep]\n", v);
      stopHaptics();
      muxDisableAll();
      delay(100);
      esp_deep_sleep_start();
    } else {
      Serial.printf("Battery: %.2fV  [OK]\n", v);
    }
  }
#else
  if (millis() - lastBatteryCheck > BATTERY_CHECK_INTERVAL_MS) {
    lastBatteryCheck = millis();
    Serial.println("Battery: [SKIPPED -- USB_DEBUG_MODE active]");
  }
#endif

  // ---- Wait for center frame ----
  if (!centerSensor.isDataReady()) return;
  if (!centerSensor.readFrame(frame)) return;

  frameCount++;

  // ---- Process frame, get zone averages for debug output ----
  int  centerAvg = 0, leftAvg = 0, rightAvg = 0;
  bool leftSafe  = true, rightSafe = true;

  Decision decision = processFrame(frame, imageWidth,
                                   &centerAvg, &leftAvg, &rightAvg,
                                   &leftSafe,  &rightSafe);

  // ---- Print every frame ----
  Serial.println("--------------------------------------------------");
  Serial.printf("Frame #%u\n", frameCount);
  Serial.printf("CENTER: %4dmm  LEFT: %4dmm  RIGHT: %4dmm\n",
                centerAvg, leftAvg, rightAvg);
  Serial.printf("Left safe: %-3s  Right safe: %-3s\n",
                leftSafe  ? "YES" : "NO",
                rightSafe ? "YES" : "NO");
  Serial.printf("Decision: %s\n", decName(decision));

  // ---- Debounce + reminder pulse ----
  if (decision == lastDecision) {
    // Same decision — check if a reminder pulse is due for LEFT/RIGHT
    if ((decision == MOVE_LEFT || decision == MOVE_RIGHT) &&
        (millis() - lastDirectionVibrateTime >= REMINDER_INTERVAL_MS)) {

      const char* sideName = (decision == MOVE_LEFT) ? "LEFT" : "RIGHT";
      Serial.printf("(no change) [REMINDER pulse -- %s still blocked for 2s+]\n", sideName);

      if (decision == MOVE_LEFT) vibrateLeft(120);
      else                        vibrateRight(120);
      delay(120);
      stopHaptics();

      lastDirectionVibrateTime = millis();  // reset timer for next reminder
    } else {
      Serial.println("(no change)");
    }
    return;
  }

  Serial.printf(">>> Decision changed: %s -> %s\n",
                decName(lastDecision), decName(decision));

  // ---- MOVE_FORWARD ----
  if (decision == MOVE_FORWARD) {
    Serial.println("Action: Path clear -- stopping haptics");
    stopHaptics();
    lastDecision = decision;
    return;
  }

  // ---- STOP ----
  if (decision == STOP) {
    Serial.println("Action: Obstacle detected -- double-pulse both motors");
    vibrateStop();
    lastDecision = decision;
    return;
  }

  // ---- MOVE_LEFT / MOVE_RIGHT: side sensor validation ----
  if (decision == MOVE_LEFT || decision == MOVE_RIGHT) {

    const char* sideName  = (decision == MOVE_LEFT) ? "LEFT"        : "RIGHT";
    ToFSensor*  sideSensor = (decision == MOVE_LEFT) ? &leftSensor  : &rightSensor;
    uint8_t     sideLPN    = (decision == MOVE_LEFT) ? LEFT_LPN     : RIGHT_LPN;

    Serial.printf("[Side Validation] Switching to %s sensor...\n", sideName);

    // Stop center, isolate bus, power up side sensor
    centerSensor.stop();
    muxDisableAll();
    delay(20);
    digitalWrite(sideLPN, HIGH);
    delay(10);
    sideSensor->start();

    Serial.println("[Side Validation] Waiting 1 frame at 5Hz (~220ms)...");
    delay(220);

    bool          pathClear   = false;
    int           stableCount = 0;
    unsigned long startTime   = millis();
    int           validFrames = 0;

    // Validation loop — max 700 ms, require 2 stable clear frames
    while (millis() - startTime < 700) {

      if (!sideSensor->isDataReady()) {
        delay(2);
        continue;
      }
      if (!sideSensor->readFrame(frame)) {
        delay(2);
        continue;
      }

      validFrames++;

      long sideSum    = 0;
      int  sideCount  = 0;
      bool bottomSafe = true;

      // Geometry-aware weighted sum for offset side sensors.
      // LEFT sensor is 229mm left of center: its RIGHT columns (5-7) face inward
      //   toward the corridor opening — weight them higher.
      // RIGHT sensor is 229mm right of center: its LEFT columns (0-2) face inward
      //   toward the corridor opening — weight them higher.
      // Outer columns of an offset sensor are more likely to clip the side wall
      //   and drag the average down — weight them lower.
      // Weight table per column: inward cols = 2, middle = 1, outer col = 0 (excluded)
      // For LEFT sensor:  col 7=2, col 6=2, col 5=2, col 4=1, col 3=1, col 2=1, col 1=0, col 0=0
      // For RIGHT sensor: col 0=2, col 1=2, col 2=2, col 3=1, col 4=1, col 5=1, col 6=0, col 7=0
      const bool isLeftSensor = (sideLPN == LEFT_LPN);
      long weightedSum = 0;
      int  weightedCount = 0;

      for (int row = 0; row < imageWidth; row++) {
        for (int col = 0; col < imageWidth; col++) {
          int d = frame[row * imageWidth + col];
          if (d <= 0) continue;

          // Determine column weight based on which sensor and inward direction
          int colWeight;
          if (isLeftSensor) {
            // LEFT sensor: inward = right side of its frame (high cols)
            if      (col >= 5) colWeight = 2;  // inward — corridor opening
            else if (col >= 2) colWeight = 1;  // middle
            else               colWeight = 0;  // outer wall-clipping cols — skip
          } else {
            // RIGHT sensor: inward = left side of its frame (low cols)
            if      (col <= 2) colWeight = 2;  // inward — corridor opening
            else if (col <= 5) colWeight = 1;  // middle
            else               colWeight = 0;  // outer wall-clipping cols — skip
          }

          if (colWeight == 0) continue;  // exclude outer wall columns

          weightedSum   += (long)d * colWeight;
          weightedCount += colWeight;

          if (row >= 5 && d < 500) bottomSafe = false;
        }
      }

      // Also add all columns to sideCount for the per-frame serial print
      for (int i = 0; i < 64; i++) if (frame[i] > 0) sideCount++;
      int avg = (weightedCount > 0) ? (int)(weightedSum / weightedCount) : 0;

      Serial.printf("[Side Validation] Frame %d: avg=%4dmm  bottomSafe=%-3s  stableCount=%d\n",
                    validFrames, avg,
                    bottomSafe ? "YES" : "NO",
                    stableCount);

      if (avg > 700 && bottomSafe) {  // 700mm: supports corridors as narrow as 37 inches
        stableCount++;
        if (stableCount >= 2) {
          pathClear = true;
          break;
        }
      } else {
        if (stableCount > 0)
          Serial.println("[Side Validation] Stable count reset -- obstacle in path");
        stableCount = 0;
      }
    }

    // Return to center sensor
    sideSensor->stop();
    muxDisableAll();
    digitalWrite(sideLPN, LOW);
    delay(20);
    digitalWrite(CENTER_LPN, HIGH);
    delay(10);
    centerSensor.start();
    delay(50);
    Serial.println("[Side Validation] Returned to CENTER sensor");

    if (pathClear) {
      Serial.printf("[Side Validation] Path CLEAR -- vibrating %s motor\n", sideName);
      if (decision == MOVE_LEFT) vibrateLeft(120);
      else                        vibrateRight(120);
      delay(120);
      stopHaptics();
      lastDirectionVibrateTime = millis();  // start reminder timer from this vibration
    } else {
      Serial.printf("[Side Validation] Path BLOCKED after 700ms -- issuing STOP\n");
      vibrateStop();
      decision = STOP;
    }

    lastDecision = decision;
  }
}
