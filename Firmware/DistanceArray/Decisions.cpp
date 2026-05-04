#include "Decisions.h"

/*
 * processFrame — Analyze 8x8 VL53L5CX frame and return navigation decision.
 *
 * ── THRESHOLD DESIGN ─────────────────────────────────────────────────────
 *
 * Target environment: belt-mounted device, user walking at normal pace.
 * Minimum passable corridor: 37 inches = 940 mm wide.
 *
 * FORWARD_CLEAR_MM (1600 mm):
 *   Center path declared clear when avg > 1600mm.
 *   ← Raised from 1500mm due to 50° actual FOV finding:
 *   At 50° FOV the full 37" corridor (940mm) only fits in the sensor frame
 *   at distances > 1008mm. Below that, corridor walls fall outside the frame
 *   and the center average reads artificially high (open air instead of wall).
 *   At 1600mm the sensor covers 1493mm wide — safely wider than 940mm corridor.
 *   This ensures FORWARD is only declared while walls are still visible in frame.
 *   At walking speed 1.4 m/s this gives ~1.1 seconds of reaction time.
 *
 * SIDE_CLEAR_MM (700 mm):
 *   Side path declared clear during validation when avg > 700mm.
 *   ← CRITICAL for narrow corridor support:
 *   In a 37-inch (940mm) corridor, if belt is centered, each side wall
 *   is ~470mm away. A side sensor aimed slightly outward sees 500–700mm
 *   to the wall. Using 1000mm here (old value) would ALWAYS fail in any
 *   corridor under ~2000mm wide — system would never guide left/right.
 *   700mm allows guiding through corridors as narrow as ~1400mm total width.
 *   Below 700mm the turn path is too tight for safe navigation at walking speed.
 *
 * BOTTOM_SAFE_MM (600 mm):
 *   Ground/low obstacle safety check on rows 5–7.
 *   Old value was 1000mm — too aggressive for belt mount. At waist height
 *   (~900mm off ground), a floor obstacle 600mm away is ~1 step distance.
 *   Reducing to 600mm prevents false STOP from normal floor detection
 *   while still catching stairs, curbs, and large low obstacles.
 *
 * SIDE_BOTTOM_SAFE_MM (500 mm):
 *   Bottom safety during side validation. Slightly more permissive than
 *   center bottom check — the side sensor sees the corridor wall at an
 *   angle; bottom rows may report shorter distances due to floor proximity.
 *
 * ── ZONE LAYOUT ──────────────────────────────────────────────────────────
 *   LEFT   = cols 0–2   (3 cols)
 *   CENTER = cols 3–4   (2 cols)
 *   RIGHT  = cols 5–7   (3 cols)
 *   BOTTOM = rows 5–7
 *
 * ── DECISION MAPPING ─────────────────────────────────────────────────────
 *   MOVE_FORWARD = 0  → centerAvg > FORWARD_CLEAR_MM
 *   MOVE_LEFT    = 1  → center blocked, left more open & bottom-safe
 *   MOVE_RIGHT   = 2  → center blocked, right open or left unsafe
 *   STOP         = 3  → all paths blocked / frame unreliable
 */

// ── Tunable thresholds ────────────────────────────────────────────────────
// Adjust these for different environments without touching logic below.

#define FORWARD_CLEAR_MM 1600    // Raised: real ~45° FOV needs >1008mm to see full 37" corridor
#define SIDE_CLEAR_MM 700        // Side zones declared clear above this distance
#define BOTTOM_SAFE_MM 600       // Bottom rows unsafe if closer than this (center sensor)
#define SIDE_BOTTOM_SAFE_MM 500  // Bottom rows unsafe if closer than this (side validation)
#define MIN_CENTER_PIXELS 4      // Min valid center pixels before trusting the frame

// ── Column weights for center sensor left/right zone averaging ─────────────
// CENTER sensor is at body midline. Its outermost columns (0 and 7) see the
// widest angle — in a 37" corridor these clip the walls at distances > ~600mm
// and drag leftAvg / rightAvg down, making one side appear more blocked than
// it really is. Weight inner columns of each side zone more heavily.
//
// LEFT zone  (cols 0–2):  col 2 = innermost (weight 2), col 1 (weight 1), col 0 outer (weight 0)
// RIGHT zone (cols 5–7):  col 5 = innermost (weight 2), col 6 (weight 1), col 7 outer (weight 0)
// CENTER zone (cols 3–4): equal weight 1 each — these drive FORWARD decision only
static const int COL_WEIGHT[8] = {
  0,  // col 0 — outer LEFT,  high wall-clip risk, excluded
  1,  // col 1 — mid LEFT
  2,  // col 2 — inner LEFT,  most reliable for left-path clearance
  1,  // col 3 — center-left
  1,  // col 4 — center-right
  2,  // col 5 — inner RIGHT, most reliable for right-path clearance
  1,  // col 6 — mid RIGHT
  0   // col 7 — outer RIGHT, high wall-clip risk, excluded
};

Decision processFrame(int16_t* distance, int imageWidth,
                      int* outCenterAvg, int* outLeftAvg, int* outRightAvg,
                      bool* outLeftSafe, bool* outRightSafe) {
  long centerSum = 0, leftSum = 0, rightSum = 0;
  int centerCount = 0, leftCount = 0, rightCount = 0;
  bool leftLowerSafe = true;
  bool rightLowerSafe = true;
  bool centerLowerSafe = true;

  for (int row = 0; row < imageWidth; row++) {
    for (int col = 0; col < imageWidth; col++) {
      int index = row * imageWidth + col;
      int d = distance[index];

      if (d <= 0) continue;

      int w = COL_WEIGHT[col];

      if (col <= 2) {
        // LEFT zone — bottom safety always checked regardless of weight
        if (row >= 5 && d < BOTTOM_SAFE_MM)
          leftLowerSafe = false;
        if (w == 0) continue;  // exclude outer wall-clipping column
        leftSum += (long)d * w;
        leftCount += w;
      } else if (col == 3 || col == 4) {
        // CENTER zone — no weighting needed, both cols equal
        if (row >= 5 && d < BOTTOM_SAFE_MM)
          centerLowerSafe = false;
        centerSum += d;
        centerCount++;
      } else {
        // RIGHT zone — bottom safety always checked regardless of weight
        if (row >= 5 && d < BOTTOM_SAFE_MM)
          rightLowerSafe = false;
        if (w == 0) continue;  // exclude outer wall-clipping column
        rightSum += (long)d * w;
        rightCount += w;
      }
    }
  }

  // Weighted averages — leftCount/rightCount now hold sum-of-weights, not pixel count
  int centerAvg = (centerCount > 0) ? (int)(centerSum / centerCount) : 0;
  int leftAvg = (leftCount > 0) ? (int)(leftSum / leftCount) : 0;
  int rightAvg = (rightCount > 0) ? (int)(rightSum / rightCount) : 0;

  // Expose values to caller for serial printing
  if (outCenterAvg) *outCenterAvg = centerAvg;
  if (outLeftAvg) *outLeftAvg = leftAvg;
  if (outRightAvg) *outRightAvg = rightAvg;
  if (outLeftSafe) *outLeftSafe = leftLowerSafe;
  if (outRightSafe) *outRightSafe = rightLowerSafe;

  // Unreliable frame guard
  if (centerCount < MIN_CENTER_PIXELS)
    return STOP;

  if (centerAvg > FORWARD_CLEAR_MM && centerLowerSafe)
    return MOVE_FORWARD;

  if (centerAvg > FORWARD_CLEAR_MM && !centerLowerSafe) {
    // Treat as blocked — choose safer side
    if (leftAvg > rightAvg && leftLowerSafe)
      return MOVE_LEFT;
    if (rightLowerSafe)
      return MOVE_RIGHT;
    return STOP;
  }

  if (leftAvg > rightAvg && leftLowerSafe)
    return MOVE_LEFT;

  if (rightLowerSafe)
    return MOVE_RIGHT;

  return STOP;
}
