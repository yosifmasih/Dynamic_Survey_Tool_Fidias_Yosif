/*
 * adxl356_calibration.c
 *
 *  Created on: Aug 5, 2025
 *      Author: yosif
 */

#include "adxl356_calibration.h"
#include "adxl356_dual.h"
extern ADC_HandleTypeDef hadc1;

// Globals defined in the header
float adxl356_bias_x = 0.0f;
float adxl356_bias_y = 0.0f;
float adxl356_bias_z = 0.0f;
bool accel356CalibrationComplete = false;

// Internal accumulators
static uint16_t sampleCount = 0;
static float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

void calibrateAccel356(void)
{
	ADXL356_DualRaw r;
	    ADXL356_ReadSixChannels(&hadc1, &r); // ranks must match CubeMX
	    float xg = volt_to_g_mid(adc_to_volt(r.mid_x_raw));
	    float yg = volt_to_g_mid(adc_to_volt(r.mid_y_raw));
	    float zg = volt_to_g_mid(adc_to_volt(r.mid_z_raw));
    // 2) accumulate
    sum_x += xg;
    sum_y += yg;
    sum_z += zg;
    sampleCount++;

    // 3) once enough samples, compute biases
    if (sampleCount >= ADXL356_CALIB_SAMPLES)
    {
        adxl356_bias_x = sum_x / sampleCount;
        adxl356_bias_y = sum_y / sampleCount;
        // subtract 1 g on Z to remove gravity
        adxl356_bias_z = (sum_z / sampleCount) - 1.0f;
        accel356CalibrationComplete = true;
        // (Optional) reset if you want to recalibrate later
        // sampleCount = 0; sum_x = sum_y = sum_z = 0.0f;
    }
}

void getCalibratedAccel356(float raw_xg,
                           float raw_yg,
                           float raw_zg,
                           float *out_xg,
                           float *out_yg,
                           float *out_zg)
{
    *out_xg = raw_xg - adxl356_bias_x;
    *out_yg = raw_yg - adxl356_bias_y;
    *out_zg = raw_zg - adxl356_bias_z;
}

