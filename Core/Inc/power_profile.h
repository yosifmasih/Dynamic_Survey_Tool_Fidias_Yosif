/*
 * power_profile.h
 *
 *  Created on: Sep 23, 2025
 *      Author: yosif
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"

// ======= Public States =======
typedef enum {
  P0_MIN = 0,  // absolute minimum code/run
  P1_NOM = 1,  // normal: 100 Hz sampling, 1 Hz calculations
  P2_STRESS = 2 // high-power but bounded stress
} pwr_state_t;

// ======= API =======
void PowerProfile_Init(void);
void PowerProfile_Set(pwr_state_t s);
pwr_state_t PowerProfile_Get(void);

// Call this once per loop tick (~100 Hz)
// - In P0_MIN it returns immediately (you'll be asleep)
// - In P1_NOM it does nothing (your normal loop runs)
// - In P2_STRESS it runs bounded CPU/memory stress
void PowerProfile_Tick100Hz(void);

// Optional: set how “hard” P2 runs (0..100%)
void PowerProfile_SetStressDuty(uint8_t percent);

// Scope/DAQ sync: pulses a GPIO high for ~50us at each state change
void PowerProfile_SyncPulse(void);

