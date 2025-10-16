/*
 * acceleration_calibration.c
 *
 *  Created on: Jul 31, 2025
 *      Author: yosif
 */


#include "accel_calibration.h"
#include "adxl312_driver.h"   // for ADXL312_ReadXYZ
#include <stdint.h>

// Globals defined in the header
float accel_bias_x = 0.0f;
float accel_bias_y = 0.0f;
float accel_bias_z = 0.0f;
bool accelCalibrationComplete = false;

// Internal accumulation state
extern float g_adxl312_lsb_per_g;  // add at top of file
static uint16_t sampleCount = 0;
static float sum_x = 0.0f;
static float sum_y = 0.0f;
static float sum_z = 0.0f;

void calibrateAccelerometer(void)
{
    int16_t rx, ry, rz;
    // 1) read raw counts
    ADXL312_ReadXYZ(&rx, &ry, &rz);
    // 2) convert to g units
    float xg = rx / g_adxl312_lsb_per_g;
    float yg = ry / g_adxl312_lsb_per_g;
    float zg = rz / g_adxl312_lsb_per_g;
    // 3) accumulate
    sum_x += xg;
    sum_y += yg;
    sum_z += zg;
    sampleCount++;

    // 4) when enough samples are in, compute biases
    if (sampleCount >= ACCEL_CALIB_SAMPLES)
    {
        accel_bias_x = sum_x / sampleCount;
        accel_bias_y = sum_y / sampleCount;
        // subtract 1g from Z to remove gravity
        accel_bias_z = (sum_z / sampleCount) - 1.0f;

        accelCalibrationComplete = true;

        // (Optional) Reset accumulation if you ever want to recalibrate:
        // sampleCount = 0;
        // sum_x = sum_y = sum_z = 0.0f;
    }
}

void getCalibratedAccel(int16_t raw_x,
                        int16_t raw_y,
                        int16_t raw_z,
                        float *out_x,
                        float *out_y,
                        float *out_z)
{
    // Convert raw to g
	float xg = raw_x / g_adxl312_lsb_per_g;
	float yg = raw_y / g_adxl312_lsb_per_g;
	float zg = raw_z / g_adxl312_lsb_per_g;
    // Subtract biases
    *out_x = xg - accel_bias_x;
    *out_y = yg - accel_bias_y;
    *out_z = zg - accel_bias_z;
}
