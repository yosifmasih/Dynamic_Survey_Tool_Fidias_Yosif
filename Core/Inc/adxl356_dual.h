/*
 * adxl356_dual.h
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

#ifndef INC_ADXL356_DUAL_H_
#define INC_ADXL356_DUAL_H_

#include "stm32l4xx_hal.h"
#include "sensors_config.h"
    #include <stdbool.h>

bool adxl356_init(void);  // optional symmetry for power profile
void adxl356_stop(void);

typedef struct {
    // MID sensor raw ADC counts (ranks 1..3)
    uint16_t mid_x_raw, mid_y_raw, mid_z_raw;
    // HI sensor raw ADC counts (ranks 4..6)
    uint16_t hi_x_raw,  hi_y_raw,  hi_z_raw;
} ADXL356_DualRaw;

void ADXL356_ReadSixChannels(ADC_HandleTypeDef* hadc, ADXL356_DualRaw* out);

static inline float adc_to_volt(uint16_t raw) {
    return ((float)raw / (float)ADC_COUNTS) * ADC_VREF;  // <- float math
    // or define ADC_COUNTS as 4095.0f in sensors_config.h
}


// ADD these externs:
extern float g_adxl356_mid_zero_v;
extern float g_adxl356_mid_v_per_g;

// CHANGE volt_to_g_mid to use the globals:
static inline float volt_to_g_mid(float v) {
    return (v - g_adxl356_mid_zero_v) / g_adxl356_mid_v_per_g;
}
static inline float volt_to_g_hi(float v) {
    return (v - ADXL356_HI_ZERO_V) / ADXL356_HI_V_PER_G;
}

#endif

