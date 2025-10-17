/*
 * power_profile.c
 *
 *  Created on: Sep 23, 2025
 *      Author: yosif
 */

#include "power_profile.h"
#include "main.h"   // brings in SystemClock_Config prototype

// ===== User-tunable pins (edit to real ones) =====
#define SYNC_GPIO_Port   GPIOB
#define SYNC_Pin         GPIO_PIN_3   // free GPIO to scope-sync state changes

// If you wired NOR RESET# to a GPIO you can put it here, else ignore
//#define FLASH_RST_Port   GPIOB
//#define FLASH_RST_Pin    GPIO_PIN_5

// ===== Local state =====
static pwr_state_t s_state = P1_NOM;
static uint8_t s_stress_duty = 70; // % of each 10ms tick we burn CPU in P2
static uint32_t s_tick_ms = 0;

// ===== Forward decls =====
static void sensors_powerdown(void);
static void sensors_enable_nominal(void);
static void clocks_low(void);
static void clocks_nominal(void);
static void stress_burn_cycles_us(uint32_t usec);

// ===== DWT cycle counter for precise burns =====
static inline void dwt_enable(void){
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}
static inline uint32_t dwt_cycles(void){ return DWT->CYCCNT; }

// ===== GPIO helper =====
static inline void pin_pulse(GPIO_TypeDef* port, uint16_t pin){
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  // ~50us pulse; adjust if needed
  for (volatile int i = 0; i < 300; ++i) __NOP();
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

// ===== Public =====
void PowerProfile_Init(void){
  // Sync pin (optional, harmless if not populated)
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gi = {0};
  gi.Pin = SYNC_Pin;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYNC_GPIO_Port, &gi);
  HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);

  dwt_enable();
  s_tick_ms = HAL_GetTick();
}

void PowerProfile_Set(pwr_state_t s){
  if (s == s_state) return;
  s_state = s;
  PowerProfile_SyncPulse();

  switch (s_state){
    case P0_MIN:
      // Turn off everything we can, then enter STOP2 and wait for RTC/button
      sensors_powerdown();
      clocks_low();
      // Gate off peripheral clocks you’re not using in P0:
      __HAL_RCC_SPI1_CLK_DISABLE();
      __HAL_RCC_I2C1_CLK_DISABLE();
      __HAL_RCC_ADC_CLK_DISABLE();
      // Keep USART for control if you want; or disable it too.

      // Enter STOP2 (deep idle). Wake via RTC or EXTI.
      HAL_SuspendTick();
      HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
      HAL_ResumeTick();
      SystemClock_Config(); // re-init clocks after STOP2
      // NOTE: You’ll likely set P1_NOM/P2_STRESS via a button/UART,
      // so P0 will immediately go back to sleep on next loop pass.
      break;

    case P1_NOM:
      clocks_nominal();
      sensors_enable_nominal();
      break;

    case P2_STRESS:
      clocks_nominal();
      sensors_enable_nominal();
      // No immediate action; stress runs in Tick
      break;
  }
}

pwr_state_t PowerProfile_Get(void){ return s_state; }

void PowerProfile_Tick100Hz(void){
  if (s_state == P2_STRESS){
    // Each 10ms tick: burn s_stress_duty% CPU with mixed math+mem
    const uint32_t burn_us = (uint32_t)(s_stress_duty * 100u / 10u); // duty% of 10ms
    stress_burn_cycles_us(burn_us);
  }
  // In P0_MIN we’ll be asleep between RTC/EXTI wakes; nothing to do here.
}

void PowerProfile_SetStressDuty(uint8_t percent){
  if (percent > 100) percent = 100;
  s_stress_duty = percent;
}

void PowerProfile_SyncPulse(void){ pin_pulse(SYNC_GPIO_Port, SYNC_Pin); }

// ===== Private impls =====

// Put sensors into lowest power the firmware can reach (no hardware EN rails assumed)
static void sensors_powerdown(void){
  // ADXL312: standby (clear MEASURE bit)
  extern void adxl312_set_measure(bool on);
  adxl312_set_measure(false);

  // LIS2MDL: power-down mode (CFG_REG_A)
  extern void lis2mdl_set_powerdown(void);
  lis2mdl_set_powerdown();

  // ADXL356: stop ADC sampling; if you have a STANDBY pin, assert it
  extern void adxl356_stop(void);
  adxl356_stop();

  // LMT01 comparator/timer off
  extern void lmt01_stop(void);
  lmt01_stop();

  // NOR: CE# high by default; no action needed if wired to FMC_NEx
}

// Enable nominal 100 Hz path (your current behavior)
static void sensors_enable_nominal(void){
  extern bool adxl312_init(void);
  extern bool lis2mdl_init(void);
  extern bool adxl356_init(void);
  adxl312_init();
  lis2mdl_init();
  adxl356_init();

  // Configure ODRs/averaging as you currently do in main.c
  extern void lis2mdl_config_100hz(void);
  lis2mdl_config_100hz();

  // Restart comparator/timer for LMT01 if used
  extern void lmt01_start(void);
  lmt01_start();
}

static void clocks_low(void){
  // Optional: scale down SYSCLK if you want P0 to be RUN-idle instead of STOP2
  // For true minimum we’re using STOP2, so nothing else needed here.
}

static void clocks_nominal(void){
  // Make sure system clocks and peripheral clocks are on for P1/P2
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_ADC_CLK_ENABLE();
  // … add what your build needs (TIM, COMP, FMC, etc.)
}

// Bounded stress: math (FPU), memory (cache/bus), optional FMC read bursts
static void stress_burn_cycles_us(uint32_t usec){
  if (usec == 0) return;
  // Approximate: convert target us to cycles
  uint32_t sysclk_hz = HAL_RCC_GetSysClockFreq();
  uint32_t target_cycles = (sysclk_hz / 1000000u) * usec;
  uint32_t start = dwt_cycles();

  // Mix of compute + memory to raise both core and bus power a bit
  volatile float a=1.2345f, b=2.3456f, c=3.4567f;
  volatile uint32_t sum = 0;
  static uint32_t blob[256]; // small working set (fits cache, still exercises bus)
  for (uint32_t i=0; ; ++i){
    // FPU math
    a = a*b + c;
    b = b*c + a;
    c = c*a + b;
    // memory thrash
    blob[i & 255] = (blob[(i-1)&255] ^ 0xA5A5A5A5u) + i;
    sum += blob[i & 255];

    // Optional: FMC read burst if NOR is mapped; comment out if not ready
    // volatile uint16_t *NOR = (uint16_t*)0x60000000; // Bank1
    // sum ^= NOR[i & 0xFF];

    if ((dwt_cycles() - start) >= target_cycles) break;
  }
  (void)sum;
}

