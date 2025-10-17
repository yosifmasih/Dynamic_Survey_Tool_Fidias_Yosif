/*
 * mag_calibration.c
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#include "mag_calibration.h"
#include "lis2mdl_driver.h"
#include <limits.h>
#include <math.h>

float mag_mid_x=0, mag_mid_y=0, mag_mid_z=0;
float mag_field_x=1, mag_field_y=1, mag_field_z=1;
float mag_sx=1, mag_sy=1, mag_sz=1;
bool  magCalibrationComplete=false;

static int16_t min_x=INT16_MAX, min_y=INT16_MAX, min_z=INT16_MAX;
static int16_t max_x=INT16_MIN, max_y=INT16_MIN, max_z=INT16_MIN;
static uint16_t count=0;

void calibrateMagnetometer(void)
{
    int16_t rx, ry, rz;
    if (!LIS2MDL_ReadRaw(&rx, &ry, &rz)) return; // non-blocking if sensor absent

    if (rx < min_x) min_x = rx;
    if (rx > max_x) max_x = rx;

    if (ry < min_y) min_y = ry;
    if (ry > max_y) max_y = ry;

    if (rz < min_z) min_z = rz;
    if (rz > max_z) max_z = rz;

    if (++count >= MAG_CALIB_SAMPLES) {
        mag_mid_x = 0.5f * (max_x + min_x);
        mag_mid_y = 0.5f * (max_y + min_y);
        mag_mid_z = 0.5f * (max_z + min_z);

        mag_field_x = 0.5f * (max_x - min_x);
        mag_field_y = 0.5f * (max_y - min_y);
        mag_field_z = 0.5f * (max_z - min_z);

        float favg = (mag_field_x + mag_field_y + mag_field_z) / 3.0f;
        mag_sx = (mag_field_x > 1e-6f) ? (favg / mag_field_x) : 1.0f;
        mag_sy = (mag_field_y > 1e-6f) ? (favg / mag_field_y) : 1.0f;
        mag_sz = (mag_field_z > 1e-6f) ? (favg / mag_field_z) : 1.0f;

        magCalibrationComplete = true;
    }
}

void getCalibratedMag(int16_t rx, int16_t ry, int16_t rz,
                      float *mx_uT, float *my_uT, float *mz_uT)
{
    // Hard-iron offsets (counts)
    float cx = (float)rx - mag_mid_x;
    float cy = (float)ry - mag_mid_y;
    float cz = (float)rz - mag_mid_z;

    // Soft-iron scales, then convert to µT (LIS2MDL sensitivity ≈ 0.15 µT/LSB)
    *mx_uT = cx * mag_sx * 0.15f;
    *my_uT = cy * mag_sy * 0.15f;
    *mz_uT = cz * mag_sz * 0.15f;
}

