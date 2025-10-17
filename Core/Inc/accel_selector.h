/*
 * accel_selector.h
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

#ifndef INC_ACCEL_SELECTOR_H_
#define INC_ACCEL_SELECTOR_H_

#include <stdint.h>

// Which accel is currently feeding orientation
typedef enum {
    ACCEL_SRC_312   = 0,  // ADXL312  (±12 g)
    ACCEL_SRC_356B  = 1,  // ADXL356B (±20 g)
    ACCEL_SRC_356C  = 2   // ADXL356C (±40 g)
} AccelSource;

// Tunables (hysteresis thresholds, in g)
#ifndef T12_UP_G
#define T12_UP_G     9.0f   // if using 312 and |a| >= 9 g -> want 356B
#endif
#ifndef T12_DOWN_G
#define T12_DOWN_G   7.0f   // if using 356B and |a| <= 7 g -> want 312
#endif
#ifndef T23_UP_G
#define T23_UP_G     16.0f  // if using 356B and |a| >= 16 g -> want 356C
#endif
#ifndef T23_DOWN_G
#define T23_DOWN_G   13.0f  // if using 356C and |a| <= 13 g -> want 356B
#endif

// Require this many consecutive samples asking for a new source before switching
#ifndef ACCEL_SWITCH_DWELL
#define ACCEL_SWITCH_DWELL  5
#endif

typedef struct {
    AccelSource current;        // active sensor
    AccelSource pending;        // proposed sensor (if different)
    uint8_t     pending_count;  // how long the proposal has persisted
} AccelSelectState;

// Initialize selector (start on the low-range sensor by default)
void accel_selector_init(AccelSelectState* st, AccelSource start);

// Decide which sensor should be used for this sample and output the chosen vector
// Inputs are CALIBRATED g-vectors for each sensor.
AccelSource accel_selector_update(const float a312[3],
                                  const float a356b[3],
                                  const float a356c[3],
                                  AccelSelectState* st,
                                  float out_g[3]);

#endif // INC_ACCEL_SELECTOR_H_

