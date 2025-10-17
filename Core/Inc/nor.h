/*
 * nor.h
 *
 *  Created on: Oct 14, 2025
 *      Author: yosif
 */

#ifndef INC_NOR_H_
#define INC_NOR_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

bool NOR_Init(void);                       // FMC timing, ID read, etc.
bool NOR_EraseSector(uint32_t sector_addr);
bool NOR_ProgramWords(uint32_t addr, const uint16_t* data, uint32_t count);
void NOR_Read(uint32_t addr, uint16_t* data, uint32_t count);

// Log helpers
#define LOG_SECTOR_BYTES   (128u*1024u)
#define LOG_REGION_BYTES   (LOG_SECTOR_BYTES * 4u)
#define LOG_START_ADDR     (0x60000000u + LOG_SECTOR_BYTES)


#endif /* INC_NOR_H_ */
