/*
 * sensors_config.h
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

#ifndef INC_SENSORS_CONFIG_H_
#define INC_SENSORS_CONFIG_H_

// ADC reference and resolution
#define ADC_VREF    3.3f
#define ADC_COUNTS  4095.0f   // 12-bit

// ADXL356B (±20 g) conversion (adjust if your supply/range differs)
#define ADXL356_MID_ZERO_V    0.90f
#define ADXL356_MID_V_PER_G   0.08f

// ADXL356C (±40 g) conversion
#define ADXL356_HI_ZERO_V     0.90f
#define ADXL356_HI_V_PER_G    0.04f

#endif
