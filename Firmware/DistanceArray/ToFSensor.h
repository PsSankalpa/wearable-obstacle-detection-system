#ifndef TOF_SENSOR_H
#define TOF_SENSOR_H

#include <Arduino.h>
#include <SparkFun_VL53L5CX_Library.h>

class ToFSensor
{
private:
    uint8_t  _channel;
    uint8_t  _lpnPin;
    bool     _initialized;
    bool     _ranging;        // FIX: track ranging state to avoid double-start

    SparkFun_VL53L5CX  _imager;
    VL53L5CX_ResultsData _data;

    void selectMux();

public:
    ToFSensor(uint8_t channel, uint8_t lpnPin);

    bool init();
    void start();
    void stop();

    bool isDataReady();
    bool readFrame(int16_t* buffer);

    bool isInitialized() const { return _initialized; }
    bool isRanging()     const { return _ranging; }
};

#endif
