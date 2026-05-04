#include "ToFSensor.h"
#include <Wire.h>

#define TCA_ADDR 0x70

ToFSensor::ToFSensor(uint8_t channel, uint8_t lpnPin)
{
    _channel     = channel;
    _lpnPin      = lpnPin;
    _initialized = false;
    _ranging     = false;    // FIX: init ranging flag
}

// --------------------------------------------------------
// selectMux — always disable all channels first, then enable one
// --------------------------------------------------------
void ToFSensor::selectMux()
{
    // Disable all channels
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("MUX disable failed: %d\n", err);
        return;
    }

    // Enable only this sensor's channel
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << _channel);
    err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("MUX ch%d enable failed: %d\n", _channel, err);
        return;
    }

    delayMicroseconds(100);  // FIX: 100 µs settling (50 µs was marginal)
}

// --------------------------------------------------------
// init — called ONCE per sensor, WITH only this sensor's LPN HIGH
// Caller is responsible for ensuring all OTHER LPN lines are LOW
// before calling init() on this sensor.
// --------------------------------------------------------
bool ToFSensor::init()
{
    // Power ON this sensor via its LPN pin
    // (Caller must have already set all OTHER LPN pins LOW)
    pinMode(_lpnPin, OUTPUT);
    digitalWrite(_lpnPin, HIGH);
    delay(200);  // FIX: 200 ms — give sensor time to boot before firmware upload
                 // 1000 ms in original was excessive; 200 ms is the datasheet minimum

    selectMux();

    // Attempt begin() — firmware upload happens here (~several seconds)
    if (!_imager.begin()) {
        Serial.printf("ToFSensor ch%d: begin() failed\n", _channel);
        digitalWrite(_lpnPin, LOW);
        return false;
    }

    _imager.setResolution(8 * 8);
    _imager.setRangingFrequency(5);     // 5 Hz
    _imager.setIntegrationTime(20);     // 20 ms

    _initialized = true;
    _ranging     = false;
    Serial.printf("ToFSensor ch%d: init OK\n", _channel);
    return true;
}

// --------------------------------------------------------
// start — begin ranging; guard against double-start
// --------------------------------------------------------
void ToFSensor::start()
{
    if (!_initialized) return;
    if (_ranging) return;    // FIX: don't call startRanging twice

    selectMux();
    _imager.startRanging();
    _ranging = true;
}

// --------------------------------------------------------
// stop — stop ranging; guard against double-stop
// --------------------------------------------------------
void ToFSensor::stop()
{
    if (!_initialized) return;
    if (!_ranging) return;   // FIX: don't call stopRanging if not ranging

    selectMux();
    _imager.stopRanging();
    _ranging = false;
}

// --------------------------------------------------------
// isDataReady
// --------------------------------------------------------
bool ToFSensor::isDataReady()
{
    if (!_initialized || !_ranging) return false;

    selectMux();
    return _imager.isDataReady();
}

// --------------------------------------------------------
// readFrame — returns false on failure
// --------------------------------------------------------
bool ToFSensor::readFrame(int16_t* buffer)
{
    if (!_initialized || !_ranging) return false;

    selectMux();

    if (!_imager.getRangingData(&_data))
        return false;

    for (int i = 0; i < 64; i++)
        buffer[i] = _data.distance_mm[i];

    return true;
}
