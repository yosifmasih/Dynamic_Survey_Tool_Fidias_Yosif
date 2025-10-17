/*
 * lis2mdl_driver.c
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#include "lis2mdl_driver.h"

extern I2C_HandleTypeDef hi2c1;

static HAL_StatusTypeDef _wr(uint8_t reg, uint8_t val, uint32_t to_ms)
{
    return HAL_I2C_Mem_Write(&hi2c1, LIS2MDL_ADDR_8BIT, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, to_ms);
}
static HAL_StatusTypeDef _rd(uint8_t reg, uint8_t *buf, uint16_t len, uint32_t to_ms)
{
    return HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_ADDR_8BIT, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, to_ms);
}

uint8_t LIS2MDL_WhoAmI(uint8_t *who)
{
    return (_rd(LIS2MDL_REG_WHOAMI, who, 1, 10) == HAL_OK);
}

uint8_t LIS2MDL_Init(void)
{
    // BDU=1 (block data update) in CFG_REG_C
    if (_wr(LIS2MDL_REG_CFG_C, 0x10, 10) != HAL_OK) return 0;

    // Continuous-conversion mode in CFG_REG_A (MD[1:0]=00). Leave other bits default.
    if (_wr(LIS2MDL_REG_CFG_A, 0x00, 10) != HAL_OK) return 0;

    HAL_Delay(10); // allow first sample
    return 1;
}

void LIS2MDL_Config100Hz(void)
{
    // CFG_REG_A (0x60):
    // TEMP_COMP=1 (bit7), LP=0, ODR=11b (100 Hz on LIS2MDL), MD=00 (continuous)
    uint8_t cfg_a = 0x8C; // 1000 1100
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x60, I2C_MEMADD_SIZE_8BIT, &cfg_a, 1, HAL_MAX_DELAY);

    // CFG_REG_C (0x62): BDU=1 (bit4)
    uint8_t cfg_c = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, 0x3C, 0x62, I2C_MEMADD_SIZE_8BIT, &cfg_c, 1, HAL_MAX_DELAY);
}


uint8_t LIS2MDL_ReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t d[6];
    if (_rd(LIS2MDL_REG_OUTX_L, d, 6, 10) != HAL_OK) return 0;

    *x = (int16_t)((d[1] << 8) | d[0]);
    *y = (int16_t)((d[3] << 8) | d[2]);
    *z = (int16_t)((d[5] << 8) | d[4]);
    return 1;
}

// --- add below your existing functions ---

/* Put LIS2MDL into power-down: CFG_A.MD[1:0] = 11b */
void lis2mdl_set_powerdown(void)
{
    uint8_t a = 0;
    if (_rd(LIS2MDL_REG_CFG_A, &a, 1, 10) != HAL_OK) return;

    a &= ~(0x03u); // clear MD[1:0]
    a |=  (0x03u); // MD = 11b (power-down)

    (void)_wr(LIS2MDL_REG_CFG_A, a, 10);
}

/* Shim-friendly alias that reuses your existing 100 Hz config */
void lis2mdl_config_100hz(void)
{
    LIS2MDL_Config100Hz();
}

