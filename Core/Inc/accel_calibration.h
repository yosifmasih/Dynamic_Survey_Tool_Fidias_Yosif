/*
 * accelerometer_calibration.h
 *
 *  Created on: Jul 31, 2025
 *      Author: yosif
 */

#ifndef INC_ACCEL_CALIBRATION_H_
#define INC_ACCEL_CALIBRATION_H_

#include <stdint.h>
#include <stdbool.h>

// how many samples to average for bias calculation
#define ACCEL_CALIB_SAMPLES  500

// Calibration results (in g)
extern float accel_bias_x;
extern float accel_bias_y;
extern float accel_bias_z;

// Flag that goes true when calibration is complete
extern bool accelCalibrationComplete;

/**
 * @brief  Run the accelerometer bias calibration.
 *         Must be called repeatedly (e.g. in setup) until accelCalibrationComplete == true.
 */
void calibrateAccelerometer(void);

/**
 * @brief  Take a raw ADXL312 reading and apply the computed bias.
 * @param  raw_x  Raw X count from ADXL312_ReadXYZ()
 * @param  raw_y  Raw Y count
 * @param  raw_z  Raw Z count
 * @param  out_x  Calibrated X in g
 * @param  out_y  Calibrated Y in g
 * @param  out_z  Calibrated Z in g
 */
void getCalibratedAccel(int16_t raw_x,
                        int16_t raw_y,
                        int16_t raw_z,
                        float *out_x,
                        float *out_y,
                        float *out_z);

#endif /* INC_ACCEL_CALIBRATION_H_ */
