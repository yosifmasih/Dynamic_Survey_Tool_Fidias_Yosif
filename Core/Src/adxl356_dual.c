/*
 * adxl356_dual.c
 *
 *  Created on: Aug 17, 2025
 *      Author: yosif
 */

#include "adxl356_dual.h"
#include <stdbool.h>

extern ADC_HandleTypeDef hadc1;
bool adxl356_init(void){ return true; }          // ADC already set up by MX_ADC1_Init
void adxl356_stop(void){ HAL_ADC_Stop(&hadc1); } // ensure ADC is halted for low-power

void ADXL356_ReadSixChannels(ADC_HandleTypeDef* hadc, ADXL356_DualRaw* out)
{
    HAL_ADC_Stop(hadc);
    HAL_ADC_Start(hadc);

    // Order must match CubeMX ranks
    if (HAL_ADC_PollForConversion(hadc, 100)==HAL_OK) out->mid_x_raw = HAL_ADC_GetValue(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100)==HAL_OK) out->mid_y_raw = HAL_ADC_GetValue(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100)==HAL_OK) out->mid_z_raw = HAL_ADC_GetValue(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100)==HAL_OK) out->hi_x_raw  = HAL_ADC_GetValue(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100)==HAL_OK) out->hi_y_raw  = HAL_ADC_GetValue(hadc);
    if (HAL_ADC_PollForConversion(hadc, 100)==HAL_OK) out->hi_z_raw  = HAL_ADC_GetValue(hadc);
}

