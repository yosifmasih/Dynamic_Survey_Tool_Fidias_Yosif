/*
 * adxl312_driver.h
 *
 *  Created on: Jul 31, 2025
 *      Author: yosif
 */

// Inc/adxl312_driver.h
#ifndef INC_ADXL312_DRIVER_H_
#define INC_ADXL312_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"   // for GPIO & HAL_Delay
#include <stdbool.h>

void adxl312_set_measure(bool on);

// Chip-select control (PA4)
#define ADXL312_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define ADXL312_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

void ADXL312_WriteRegister(uint8_t reg, uint8_t val);
void ADXL312_Config100Hz(void);


// Public API
void ADXL312_Init(void);
uint8_t ADXL312_ReadRegister(uint8_t reg);
void ADXL312_ReadXYZ(int16_t *x, int16_t *y, int16_t *z);

#endif // INC_ADXL312_DRIVER_H_

