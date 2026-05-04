#ifndef DECISIONS_H
#define DECISIONS_H

#include <Arduino.h>

enum Decision {
  MOVE_FORWARD,
  MOVE_LEFT,
  MOVE_RIGHT,
  STOP
};

// outCenterAvg / outLeftAvg / outRightAvg — zone averages in mm (pass nullptr to ignore)
// outLeftSafe / outRightSafe             — bottom-row safety flags (pass nullptr to ignore)
Decision processFrame(int16_t* distance, int imageWidth,
                      int*  outCenterAvg = nullptr,
                      int*  outLeftAvg   = nullptr,
                      int*  outRightAvg  = nullptr,
                      bool* outLeftSafe  = nullptr,
                      bool* outRightSafe = nullptr);

#endif