/*
 * adxl312_driver.c
 *
 *  Created on: Jul 31, 2025
 *      Author: yosif
 */
// Src/adxl312_driver.c
#include "adxl312_driver.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include <stdlib.h>   // for abs()


extern SPI_HandleTypeDef hspi1;
#include <stdbool.h>

void adxl312_set_measure(bool on){
    ADXL312_WriteRegister(0x2D, on ? 0x08 : 0x00); // MEASURE bit
}

// Initialize into measurement mode
void ADXL312_Init(void)
{
    uint8_t tx[2] = { 0x2D, 0x08 };   // POWER_CTL reg, Measure bit
    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();
    HAL_Delay(10);
}

// Read a single register via SPI (MSB=1 for read)
uint8_t ADXL312_ReadRegister(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = {0};

    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();

    return rx[1];
}

void ADXL312_WriteRegister(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val };      // MSB=0 for write
    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();
}

// 100 Hz ODR, ±12 g range
void ADXL312_Config100Hz(void)
{
    // BW_RATE (0x2C): 0x0A ≈ 100 Hz on ADXL3xx family
    ADXL312_WriteRegister(0x2C, 0x0A);

    // DATA_FORMAT (0x31): range = ±12 g => 0x03 (keep INT_INVERT=0, etc.)
    ADXL312_WriteRegister(0x31, 0x03);
}


// Burst-read X, Y, Z (0x32 start, multibyte)
void ADXL312_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t cmd = (uint8_t)(0xC0 | 0x32);   // READ(0x80)|MB(0x40)|start=0x32
    uint8_t d[6];

    ADXL312_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);   // send command only
    HAL_SPI_Receive(&hspi1, d, sizeof(d), HAL_MAX_DELAY); // clock out 6 data bytes
    ADXL312_CS_HIGH();

    *x = (int16_t)((d[1] << 8) | d[0]);
    *y = (int16_t)((d[3] << 8) | d[2]);
    *z = (int16_t)((d[5] << 8) | d[4]);
}


static inline int valid_xyz(int16_t x,int16_t y,int16_t z){
    const int16_t LIM = 20000;
    return (abs(x)<LIM)&&(abs(y)<LIM)&&(abs(z)<LIM);
}

int ADXL312_ReadXYZ_Filtered(int16_t *x, int16_t *y, int16_t *z)
{
    int16_t x1,y1,z1,x2,y2,z2;

    // One sample to prime, one sample to use (simple deglitch)
    ADXL312_ReadXYZ(&x1,&y1,&z1);
    ADXL312_ReadXYZ(&x2,&y2,&z2);

    if(!valid_xyz(x2,y2,z2)) return 0;
    if((abs(x2-x1)>1024)||(abs(y2-y1)>1024)||(abs(z2-z1)>1024)) return 0;

    *x=x2; *y=y2; *z=z2;
    return 1;
}





