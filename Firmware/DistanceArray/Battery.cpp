#include "Battery.h"

#define BATTERY_ADC_PIN    34
#define ADC_MAX_VALUE      4095.0f
#define DIVIDER_RATIO      2.0f
#define BATTERY_LOW_THRESHOLD 3.2f

// FIX: ESP32 ADC at 11dB attenuation has a nonlinear characteristic.
// The raw ADC→voltage relationship is NOT a perfect linear ramp.
// The reference after calibration is effectively ~3.1V not 3.3V at 11dB.
// Using 3.1V here gives significantly better real-world accuracy.
// If you have esp_adc_cal available, use that instead for proper calibration.
#define ADC_EFFECTIVE_REF  3.1f

void batteryInit()
{
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db);
    // FIX: throw away first few reads — ADC needs to settle after attenuation change
    for (int i = 0; i < 5; i++) {
        analogRead(BATTERY_ADC_PIN);
        delay(5);
    }
}

float readBatteryVoltage()
{
    const int SAMPLES = 16;   // FIX: 16 samples gives better noise rejection than 10
    long sum = 0;

    for (int i = 0; i < SAMPLES; i++) {
        sum += analogRead(BATTERY_ADC_PIN);
        delay(2);
    }

    float raw            = (float)sum / SAMPLES;
    float voltageAtADC   = (raw / ADC_MAX_VALUE) * ADC_EFFECTIVE_REF;
    float batteryVoltage = voltageAtADC * DIVIDER_RATIO;

    return batteryVoltage;
}

bool isBatteryLow()
{
    // FIX: lowCount is static — it persists correctly between calls.
    // But it never decrements below 0; original version was fine here.
    static int lowCount = 0;
    const  int LOW_LIMIT = 5;

    float v = readBatteryVoltage();
    // Note: caller prints the voltage — no print here to avoid double output

    if (v < BATTERY_LOW_THRESHOLD) {
        lowCount++;
    } else {
        lowCount = 0;
    }

    return (lowCount >= LOW_LIMIT);
}
