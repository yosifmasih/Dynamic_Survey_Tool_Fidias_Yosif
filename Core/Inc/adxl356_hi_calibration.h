/*
 * adxl356_hi_calibration.h
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

#ifndef INC_ADXL356_HI_CALIBRATION_H_
#define INC_ADXL356_HI_CALIBRATION_H_

#include <stdbool.h>

// Number of samples to average for bias (in g)
#define ACCEL356_HI_CAL_SAMPLES  500

// Computed biases (in g) - match .c variable names
extern float accel356_hi_bias_x;
extern float accel356_hi_bias_y;
extern float accel356_hi_bias_z;

// Calibration completion flag
extern bool accel356_hi_CalibrationComplete;

/**
 * @brief Run the ADXL356 HI-range bias calibration.
 *        Call repeatedly with calibrated samples (xg,yg,zg)
 *        until accel356_hi_CalibrationComplete == true.
 */
void calibrateAccel356_HI(float xg, float yg, float zg);

/**
 * @brief Apply the pre-computed biases to a raw reading (in g).
 */
void getCalibratedAccel356_HI(float raw_xg,
                              float raw_yg,
                              float raw_zg,
                              float *out_xg,
                              float *out_yg,
                              float *out_zg);

#endif /* INC_ADXL356_HI_CALIBRATION_H_ */

