/*
 * mag_calibration.h
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#ifndef INC_MAG_CALIBRATION_H_
#define INC_MAG_CALIBRATION_H_

#include <stdbool.h>
#include <stdint.h>

// Collect enough diverse orientations
#define MAG_CALIB_SAMPLES 600

// Hard-iron offsets (counts)
extern float mag_mid_x, mag_mid_y, mag_mid_z;
// Semi-axes (counts)
extern float mag_field_x, mag_field_y, mag_field_z;
// Per-axis soft-iron scales (unitless)
extern float mag_sx, mag_sy, mag_sz;
// Completion flag
extern bool  magCalibrationComplete;

void calibrateMagnetometer(void);  // call repeatedly until complete
void getCalibratedMag(int16_t rx, int16_t ry, int16_t rz,
                      float *mx_uT, float *my_uT, float *mz_uT);

#endif

