/*
 * nor.c
 *
 *  Created on: Oct 14, 2025
 *      Author: yosif
 */

#include "nor.h"
#include "stm32l4xx_hal.h"

// If you’re using HAL NOR, wrap HAL_NOR_* here.
// If you’re bit-banging command sequences, keep your NOR16() + nor_cmd() here.

#define NOR16(addr) (*((volatile uint16_t*)(0x60000000u + (addr))))

static inline void nor_cmd(uint32_t addr, uint16_t data){ NOR16(addr<<1) = data; }
static inline void nor_unlock(void){ nor_cmd(0x555,0xAA); nor_cmd(0x2AA,0x55); }

bool NOR_Init(void) {
  // TODO: FMC timing (via MX_FMC_Init or here), optional manufacturer/device ID check
  return true;
}

bool NOR_EraseSector(uint32_t sector_addr){
  nor_unlock(); nor_cmd(0x555,0x80); nor_unlock(); NOR16(sector_addr<<1) = 0x30;
  // TODO: poll for ready
  return true;
}

bool NOR_ProgramWords(uint32_t addr, const uint16_t* data, uint32_t count){
  for (uint32_t i=0;i<count;i++){
    nor_unlock(); nor_cmd(0x555,0xA0); NOR16(addr<<1) = data[i];
    addr += 1;
    // TODO: poll DQ7/DQ5 or use read-back verification
  }
  return true;
}

void NOR_Read(uint32_t addr, uint16_t* data, uint32_t count){
  for (uint32_t i=0;i<count;i++) data[i] = NOR16((addr+i)<<1);
}

