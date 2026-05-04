# Wearable Obstacle Detection System
### A Wearable Obstacle Detection System Using ToF Wide Field Sensors 
### and Haptic Feedback for Visually Impaired Users

Master of Computer Science Thesis  
University of Colombo School of Computing (UCSC)

---

## Overview
A belt-mounted wearable device that provides direction-aware haptic 
navigation guidance for visually impaired users using three VL53L5CX 
wide-field Time-of-Flight sensors and an ESP32 microcontroller.

---

## Hardware Components
- ESP32 microcontroller
- VL53L5CX wide-field ToF sensor × 3 (CENTER, LEFT, RIGHT)
- TCA9548A I2C multiplexer
- DRV2605 haptic motor driver × 2
- Vibration motors × 2
- 3.7V 1600mAh Li-Po battery
- TP4056 charging module
- MT3608 boost converter (3.7V → 5V)

---

## Pin Configuration
| GPIO | Function |
|------|----------|
| GPIO21 | I2C SDA |
| GPIO22 | I2C SCL |
| GPIO18 | CENTER sensor LPN |
| GPIO25 | RIGHT sensor LPN |
| GPIO32 | LEFT sensor LPN |
| GPIO34 | Battery ADC |
| GPIO2  | Status LED |

---

## TCA9548A Channel Map
| Channel | Device |
|---------|--------|
| 0 | CENTER ToF sensor |
| 2 | LEFT ToF sensor |
| 4 | RIGHT ToF sensor |
| 6 | RIGHT haptic driver (DRV2605) |
| 7 | LEFT haptic driver (DRV2605) |

---

## Key Thresholds
| Parameter | Value |
|-----------|-------|
| Forward clearance | 1600 mm |
| Side clearance | 700 mm |
| Center bottom safety | 600 mm |
| Side bottom safety | 500 mm |
| Reminder pulse interval | 2000 ms |
| Battery cutoff | 3.2V (5 consecutive readings) |

---

## Libraries Required
- SparkFun_VL53L5CX_Library
- Adafruit_DRV2605
- Wire.h (built-in)
- esp_sleep.h (built-in)

---

## System Decisions
| Decision | Meaning | Haptic Output |
|----------|---------|---------------|
| MOVE_FORWARD | Path clear | Silent |
| MOVE_LEFT | Turn left | Left motor single pulse |
| MOVE_RIGHT | Turn right | Right motor single pulse |
| STOP | All paths blocked | Both motors double pulse |

---

## Important Hardware Note
The TCA9548A RST pin must be connected to 3.3V permanently.
Leaving it floating will cause intermittent or complete MUX failure.
