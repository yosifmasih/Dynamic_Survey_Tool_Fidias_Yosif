/*
 * sensors_shim.c
 *
 *  Created on: Oct 11, 2025
 *      Author: yosif
 */

// Core/Src/sensors_shim.c
#include <stdbool.h>
#include "main.h"
#include "adxl312_driver.h"
#include "lis2mdl_driver.h"

extern COMP_HandleTypeDef hcomp1;

/* Optional: used by your demo loop to trigger a “flat spin” test */
void maybe_start_flat_spin_from_uart(void) { /* no-op for now */ }

/* === ADXL312 (SPI) === */
bool adxl312_init(void) {
  ADXL312_Init();
  /* If you want the shim to set ODR too, uncomment: */
  // ADXL312_Config100Hz();
  return true;
}

/* If power_profile.c references this, keep a weak stub unless your driver provides it */
__attribute__((weak)) void adxl312_set_measure(bool on) { (void)on; }

/* === ADXL356 (ADC) — provide weak stubs if your driver doesn’t define them === */
__attribute__((weak)) bool adxl356_init(void) { return true; }
__attribute__((weak)) void adxl356_stop(void) {}

/* === LIS2MDL (I2C) ===
 * power_profile.c calls lis2mdl_init(), lis2mdl_config_100hz(), lis2mdl_set_powerdown().
 * The last two are already implemented in your driver; just bridge the init with the right signature.
 */
bool lis2mdl_init(void) {        // <- must return bool to match power_profile.c
  return (LIS2MDL_Init() != 0);
}
/* The following two come from lis2mdl_driver.c; no need to redeclare/define here:
   void lis2mdl_config_100hz(void);   // calls LIS2MDL_Config100Hz() internally
   void lis2mdl_set_powerdown(void);  // writes CFG_A.MD=11b (power-down)
*/

/* === LMT01 via COMP1 === */
void lmt01_start(void) { HAL_COMP_Start_IT(&hcomp1); }
void lmt01_stop(void)  { HAL_COMP_Stop_IT(&hcomp1); }
