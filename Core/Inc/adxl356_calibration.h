/*
 * adxl356_calibration.h
 *
 *  Created on: Aug 5, 2025
 *      Author: yosif
 */

#ifndef INC_ADXL356_CALIBRATION_H_
#define INC_ADXL356_CALIBRATION_H_

#include <stdint.h>
#include <stdbool.h>

// Number of samples to average for bias (in g)
#define ADXL356_CALIB_SAMPLES  500

// Computed biases (in g)
extern float adxl356_bias_x;
extern float adxl356_bias_y;
extern float adxl356_bias_z;

// Calibration completion flag
extern bool accel356CalibrationComplete;

/**
 * @brief  Run the ADXL356 bias calibration. Call repeatedly (e.g. in setup loop)
 *         until accel356CalibrationComplete == true.
 */
void calibrateAccel356(void);

/**
 * @brief  Apply the pre-computed biases to a raw reading.
 * @param  raw_xg  Raw acceleration in g (from ADXL356_ReadAccelG)
 * @param  raw_yg  Raw Y in g
 * @param  raw_zg  Raw Z in g
 * @param  out_xg  Calibrated X in g
 * @param  out_yg  Calibrated Y in g
 * @param  out_zg  Calibrated Z in g
 */
void getCalibratedAccel356(float raw_xg,
                           float raw_yg,
                           float raw_zg,
                           float *out_xg,
                           float *out_yg,
                           float *out_zg);

#endif // INC_ADXL356_CALIBRATION_H_

