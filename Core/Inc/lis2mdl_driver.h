/*
 * lis2mdl_driver.h
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#ifndef INC_LIS2MDL_DRIVER_H_
#define INC_LIS2MDL_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

void lis2mdl_set_powerdown(void);
void lis2mdl_config_100hz(void); // if not already declared

// HAL uses 8-bit I2C address. LIS2MDL 7-bit=0x1E -> 8-bit=0x3C
#define LIS2MDL_ADDR_8BIT    0x3C

// Registers we use
#define LIS2MDL_REG_WHOAMI   0x4F
#define LIS2MDL_REG_CFG_A    0x60
#define LIS2MDL_REG_CFG_C    0x62
#define LIS2MDL_REG_OUTX_L   0x68

// Public API
uint8_t LIS2MDL_WhoAmI(uint8_t *who);     // returns 1 on success
uint8_t LIS2MDL_Init(void);               // BDU=1, continuous-conversion; returns 1 on success
uint8_t LIS2MDL_ReadRaw(int16_t *x, int16_t *y, int16_t *z); // returns 1 on success

void LIS2MDL_Config100Hz(void);

#endif



