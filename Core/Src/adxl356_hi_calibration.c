/*
 * adxl356_hi_calibration.c
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

/*
 * adxl356_hi_calibration.c
 */

#include "adxl356_hi_calibration.h"

float accel356_hi_bias_x = 0.0f;
float accel356_hi_bias_y = 0.0f;
float accel356_hi_bias_z = 0.0f;
bool  accel356_hi_CalibrationComplete = false;

static unsigned count = 0;
static double sx = 0.0, sy = 0.0, sz = 0.0;

void calibrateAccel356_HI(float xg, float yg, float zg)
{
    if (accel356_hi_CalibrationComplete) return;

    sx += xg; sy += yg; sz += zg;
    if (++count >= ACCEL356_HI_CAL_SAMPLES) {
        accel356_hi_bias_x = (float)(sx / count);
        accel356_hi_bias_y = (float)(sy / count);
        accel356_hi_bias_z = (float)(sz / count) - 1.0f; // remove +1g on Z (upright)
        accel356_hi_CalibrationComplete = true;

        // reset accumulators for potential re-use
        count = 0; sx = sy = sz = 0.0;
    }
}

void getCalibratedAccel356_HI(float rx, float ry, float rz,
                              float* cx, float* cy, float* cz)
{
    *cx = rx - accel356_hi_bias_x;
    *cy = ry - accel356_hi_bias_y;
    *cz = rz - accel356_hi_bias_z;
}


