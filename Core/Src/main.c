/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdarg.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// ---- TEMP: bench bring-up toggles ----

#define PRINT_TO_TERMINAL   1   // prints 1 Hz log over UART/CDC
#define LOG_TO_MEMORY       0   // enables NOR circular log
#define RAW_DATA            0   // prints raw sensor counts

#ifndef SHOWCASE
#define SHOWCASE 1
#endif

#if SHOWCASE
  #define KP(tag, fmt, ...) do { \
    printf("%08lu %-12s " fmt "\r\n", (unsigned long)HAL_GetTick(), tag, ##__VA_ARGS__); \
  } while(0)
#else
  #define KP(tag, fmt, ...) do { } while(0)
#endif

#define NOR_PRESENT  0   // <-- bench: no external NOR connected
#define ADXL356_MID_PRESENT 1
#define ADXL356_HI_PRESENT  0   // unwired on bench
#define ADXL312_PRESENT     1   // if your 312 is wired
     // set 0 if the SPI 312 isn’t wired
#define MAG_PRESENT 1
#define ACCEL_STREAM_1HZ  1   // print a clean snapshot once per second
#define ACCEL_SELECTOR 0

/* ===================== Logging configuration ===================== */
#define LOG_EVERY_MS            1000u       /* 1 Hz */
#define TEMP_PRESENT            0           /* set to 1 when LMT01 (COMP1) is active */
#define USE_USB_CDC             0           /* set to 1 if you enabled USB Device (CDC) */
#define AUTO_USB_OFFLOAD        0           /* stream each record as CSV when produced */


/* ---------- NOR mapping (FMC Bank1/NE1 by default) ---------- */
#define NOR_BASE_ADDR           ((uint32_t)0x60000000U)
#define LOG_SECTOR_BYTES        (128u*1024u)        /* S29GL01GT uniform sector size */
#define LOG_REGION_BYTES        (LOG_SECTOR_BYTES * 4u) /* we’ll use 4 sectors (~512 KB) */
#define LOG_START_ADDR          (NOR_BASE_ADDR + LOG_SECTOR_BYTES) /* keep sector 0 free */


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "accel_calibration.h"
#include "lis2mdl_driver.h"
#include "mag_calibration.h"
#include "orientation.h"
#include "adxl312_driver.h"        // for ADXL312 bias
#include "adxl356_dual.h"               // adc_to_volt(), volt_to_g_mid/hi(), ADXL356_DualRaw, ADXL356_ReadSixChannels
#include "adxl356_calibration.h"        // getCalibratedAccel356(...)  (your MID-range 356 bias module)
#include "adxl356_hi_calibration.h"     // calibrateAccel356_HI(...), getCalibratedAccel356_HI(...)
#define T12_UP_G   3.0f   // 312 -> 356MID at >= 3 g
#define T12_DOWN_G 2.0f   // 356MID -> 312 at <= 2 g

static uint8_t g_nor_ok = 0;

float g_adxl312_lsb_per_g = 256.0f;  // was 85.0f


// Defaults (will be auto-fitted); start with your current constants
float g_adxl356_mid_zero_v = ADXL356_MID_ZERO_V;
float g_adxl356_mid_v_per_g = ADXL356_MID_V_PER_G;


static uint32_t   g_log_ptr  = LOG_START_ADDR;   /* next NOR write address in bytes */
// put this just before the "NOR low-level (guarded)" block
extern NOR_HandleTypeDef hnor1;
/* FMC initialization function */




#if ACCEL_SELECTOR
#include "accel_selector.h"
#endif

#include "power_profile.h"

    #include <stdbool.h>

// somewhere global:
	    	extern int CDC_Write(const void*, uint16_t); // from usbd_cdc_if.c



	    	static void dump_last_lines_over_usb(uint32_t lines) {
	    	  // Example: walk backward from g_log_ptr in your ring and send the last N lines.
	    	  // If you don’t yet track exact line boundaries, start simple: send a fixed window.
	    	  const uint32_t BYTES = 4096; // quick test window
	    	  uint32_t start = (g_log_ptr >= BYTES) ? (g_log_ptr - BYTES) : LOG_START_ADDR;
	    	  const uint8_t *p = (const uint8_t*)(NOR_BASE_ADDR + start);
	    	  CDC_Write(p, BYTES);
	    	}


	    	//float gz = mid_z / sqrtf(mid_x * mid_x + mid_y * mid_y + mid_z * mid_z);
static void jump_to_system_bootloader(void){
  // address varies by L4 part; common: 0x1FFF0000
  uint32_t sysmem = 0x1FFF0000UL;
  typedef void (*pFunction)(void);
  __disable_irq();

  HAL_RCC_DeInit();
  HAL_DeInit();
  SysTick->CTRL = 0; SysTick->LOAD = 0; SysTick->VAL = 0;

  __set_MSP(*((uint32_t*)sysmem));
  pFunction boot = (pFunction)*((uint32_t*)(sysmem + 4));
  boot();
}


extern int8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

static void CDC_Printf(const char* fmt, ...) {
    static char line[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n > 0) CDC_Transmit_FS((uint8_t*)line, (uint16_t)n);
}



// Assumes 16-bit bus, base 0x60000000
#define NOR16(addr) (*((volatile uint16_t*)(0x60000000u + (addr))))

static void nor_cmd(uint32_t addr, uint16_t data){ NOR16(addr<<1) = data; } // <<1 if A0 not used on 16-bit bus

static void nor_unlock(void){
  nor_cmd(0x555, 0xAA);
  nor_cmd(0x2AA, 0x55);
}

static void nor_write_word(uint32_t addr, uint16_t data){
  nor_unlock();
  nor_cmd(0x555, 0xA0);
  NOR16(addr<<1) = data;
  // poll DQ7/DQ6 if desired, or add small wait
}

static void nor_erase_sector(uint32_t sec_base){
  nor_unlock(); nor_cmd(0x555, 0x80);
  nor_unlock(); nor_cmd(sec_base, 0x30);
  // wait on DQ7 or timeout
}



/* Optional: pull in HAL NOR/USB if you plan to enable them later */


#if USE_USB_CDC
  #include "usbd_cdc_if.h"          /* CDC_Transmit_FS() */
#endif

/* ===================== Record & helpers ===================== */
typedef struct __attribute__((packed)) {
  uint32_t magic;       /* 0x4C4F4721 'LOG!' */
  uint32_t seq;         /* monotonically increasing sequence */
  uint32_t timestamp_s; /* HAL_GetTick()/1000 (or RTC seconds if you later switch) */
  float    azimuth_deg;
  float    inclination_deg;
  float    dip_deg;
  float    temperature_c;
  uint32_t d2;
  uint32_t d3;
  uint32_t d4;
  uint32_t d5;
  uint32_t crc32;       /* over fields up to (but not including) crc32 */
} log_record_t;

static uint32_t   g_log_seq  = 0;
static uint32_t   g_log_ptrap = LOG_START_ADDR + LOG_REGION_BYTES;

/* ====== tiny CRC32 (polynomial 0xEDB88320), tableless to keep code light ====== */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
  crc = ~crc;
  for (size_t i=0; i<len; ++i) {
    crc ^= data[i];
    for (int b=0; b<8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (-(int)(crc & 1)));
  }
  return ~crc;
}

/* ===================== NOR low-level (guarded) ===================== */
#if NOR_PRESENT
/* Erase one 128KB sector that contains 'addr' */
static HAL_StatusTypeDef nor_erase_sector_containing(uint32_t addr)
{
  uint32_t block_off  = (addr - NOR_BASE_ADDR) / LOG_SECTOR_BYTES;
  uint32_t block_addr = block_off * LOG_SECTOR_BYTES;       // offset from base
  return HAL_NOR_Erase_Block(&hnor1, block_addr, 0);        // AddressSpace=0
}


/* Program a buffer (any size) as 16-bit halfwords */
static HAL_StatusTypeDef nor_program_words(uint32_t dst_addr, const void *buf, size_t nbytes)
{
  const uint16_t *p = (const uint16_t*)buf;
  size_t          hw = (nbytes + 1u) / 2u;          // halfwords
  uint32_t        addr;                             // absolute FMC address
  for (size_t i = 0; i < hw; ++i) {
    uint16_t half = p[i];
    addr = dst_addr;                                // byte address in NOR space
    addr += (uint32_t)(i * 2u);                     // advance by halfword
    uint32_t phys = NOR_BASE_ADDR + addr;           // FMC-mapped absolute address
    HAL_StatusTypeDef st = HAL_NOR_Program(&hnor1, &phys, &half);
    if (st != HAL_OK) return st;
  }
  return HAL_OK;
}

#endif /* NOR_PRESENT */

/* ===================== High-level logger API ===================== */

/* Call once at boot (after clocks, before loop). Safe if NOR/USB disabled. */
static void Log_Init(void)
{
#if NOR_PRESENT
  MX_FMC_Init();
  if (!g_nor_ok) { g_log_seq = 0; g_log_ptr = LOG_START_ADDR; return; }
  (void)nor_erase_sector_containing(LOG_START_ADDR + 0*LOG_SECTOR_BYTES);
  (void)nor_erase_sector_containing(LOG_START_ADDR + 1*LOG_SECTOR_BYTES);
#endif

  g_log_seq  = 0;
  g_log_ptr  = LOG_START_ADDR;
}

/* Append one record to NOR (if enabled) and optionally stream over USB/UART */
static void Log_Push(float az_deg, float inc_deg, float dip_deg, float temp_c, uint32_t d2, uint32_t d3, uint32_t d4, uint32_t d5)
{
  log_record_t r;
  r.magic        = 0x4C4F4721u; /* 'LOG!' */
  r.seq          = g_log_seq++;
  r.timestamp_s  = HAL_GetTick()/1000u;
#if TEMP_PRESENT
  r.temperature_c = temp_c;
#else
  r.temperature_c = NAN;  /* sensor not present for now */
#endif
  r.azimuth_deg     = az_deg;
  r.inclination_deg = inc_deg;
  r.dip_deg         = dip_deg;
  r.d2 = d2;
  r.d3 = d3;
  r.d4 = d4;
  r.d5 = d5;


  r.crc32 = crc32_update(0, (const uint8_t*)&r, sizeof(r) - sizeof(r.crc32));

#if NOR_PRESENT
  if (g_nor_ok) {
    if (g_log_ptr + sizeof(r) > g_log_ptrap) g_log_ptr = LOG_START_ADDR;
    if (((g_log_ptr - LOG_START_ADDR) % LOG_SECTOR_BYTES) == 0) {
      (void)nor_erase_sector_containing(g_log_ptr);
    }
    (void)nor_program_words(g_log_ptr, &r, sizeof(r));
    g_log_ptr += (uint32_t)((sizeof(r)+1)&~1u);
  }
#endif


#if AUTO_USB_OFFLOAD
  /* Stream CSV immediately (USB CDC if enabled, else UART printf) */
  char line[160];
  int n = snprintf(line, sizeof(line),
		  "%.2f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%lu\r\n",
                   (unsigned long)r.seq,
                   (unsigned long)r.timestamp_s,
                   r.azimuth_deg, r.inclination_deg, r.dip_deg, r.temperature_c, r.t2, r.t3, r.t4, r.t5);
#if USE_USB_CDC
  (void)CDC_Transmit_FS((uint8_t*)line, (uint16_t)n);
#else
  /* falls back to your existing printf/UART path */
  printf("%s", line);
#endif
#endif /* AUTO_USB_OFFLOAD */
}



typedef enum { SRC_356MID=0, SRC_312=1 } accel_src_t;
static accel_src_t g_src = SRC_356MID;
static uint8_t hi_count=0, lo_count=0;
#define TH_HI_ENTER  8.5f
#define TH_HI_EXIT   7.5f
#define N_HOLD       3



extern UART_HandleTypeDef huart2;   // <— add this before maybe_start_flat_spin_from_uart()

#include <math.h>



static float clampf(float x, float a, float b){ return x < a ? a : (x > b ? b : x); }

#define ORI_HIST_CAP 10
typedef struct {
    float g_sum[3];      // gravity unit vectors sum
    float mh_sum[3];     // mag leveled sum: [mxh, myh, mzh]
    uint32_t nsamp;

    // in your state struct
    float g_hist[ORI_HIST_CAP][3];
    float mh_hist[ORI_HIST_CAP][3];
    uint8_t hist_len, hist_idx;

    uint32_t t0_ms;
} OrientAgg;


static OrientAgg O = {0};

static void v3_norm(const float v[3], float out[3]){
    float n = sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); if(n<1e-6f){ out[0]=out[1]=0; out[2]=1; } else {
        out[0]=v[0]/n; out[1]=v[1]/n; out[2]=v[2]/n;
    }
}

// ===== Horizontal soft-iron correction (leveled plane) =====
static float Hcorr_kx = 1.0f;   // keep X as reference (usually 1.0)
static float Hcorr_ky = .66f;   // we'll measure and set this

static inline void apply_horizontal_softiron(float *xh, float *yh){
    *xh *= Hcorr_kx;
    *yh *= Hcorr_ky;
}

// Tilt-compensate mag using roll/pitch from gravity unit vector


static void reset_second_window(OrientAgg* A, uint32_t now_ms){
    A->g_sum[0]=A->g_sum[1]=A->g_sum[2]=0.f;
    A->mh_sum[0]=A->mh_sum[1]=A->mh_sum[2]=0.f;
    A->nsamp=0;
    A->t0_ms = now_ms;
}


// orientation.h (or a small mag_map.h you #include from main.c)
#ifndef MAG_MAP_H
#define MAG_MAP_H
#include <stdint.h>

// Signed-permutation mapping: body = MAP * sensor
// Only -1,0,+1 entries; one nonzero per row/column.
typedef struct { int8_t M[3][3]; } MagMap;

// Start with identity; adjust signs/swaps below.
static const MagMap MAG_MAP = { .M = {
    { +1,  0,  0 },   // body X = +sensor X
    {  0, +1,  0 },   // body Y = +sensor Y
    {  0,  0, +1 }    // body Z = +sensor Z
}};

static inline void mag_remap(float mx, float my, float mz,
                             float* bx, float* by, float* bz)
{
    *bx = MAG_MAP.M[0][0]*mx + MAG_MAP.M[0][1]*my + MAG_MAP.M[0][2]*mz;
    *by = MAG_MAP.M[1][0]*mx + MAG_MAP.M[1][1]*my + MAG_MAP.M[1][2]*mz;
    *bz = MAG_MAP.M[2][0]*mx + MAG_MAP.M[2][1]*my + MAG_MAP.M[2][2]*mz;
}
#endif

// learned at boot; use 85.0f as a harmless default
//float g_adxl312_lsb_per_g = 85.0f;

#if ACCEL_SELECTOR
static AccelSelectState a_sel;
#endif





//set your local magnetic declination (degrees). Keep 0.0 for magnetic heading.
#define MAG_DECLINATION_DEG   2.4f
#define PRINT_RAW  0  // set to 1 to enable once-per-second raw snapshot

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef hlpuart1;  // or huart2 if you're using USART2
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

COMP_HandleTypeDef hcomp1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

NOR_HandleTypeDef hnor1;

/* USER CODE BEGIN PV */
volatile uint32_t lmt01_edges = 0;
uint8_t count=0;
uint8_t tx_buffer[27]="Welcome! \n\r";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_FMC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// Forward declarations
static inline float LMT01_PulsesToC(uint32_t pulses);
bool ADXL312_ReadXYZ_Filtered(int16_t *rx, int16_t *ry, int16_t *rz);

static void adxl312_bootstrap_scale_from_356(float seconds);

static void ADXL356_ReadAccelG(float *gx, float *gy, float *gz);
void maybe_start_flat_spin_from_uart(void);

static void GPIO_Init_TIM_CH1_Pins(void);
static void TIM_All_CH1_ExternalClock_Init(void);
static void Reapply_COMP1_ForLMT01(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN USB_DUMP */
static void dump_last_bytes_over_usb(uint32_t bytes)
{
#if USE_USB_CDC
  // Boundaries of the log region
  const uint32_t region_base = LOG_START_ADDR;
  const uint32_t region_end  = LOG_START_ADDR + LOG_REGION_BYTES;

  // Where we last wrote
  uint32_t end   = g_log_ptr;                         // byte offset within NOR window
  if (end < region_base) end = region_base;
  if (end > region_end)  end = region_end;

  // Choose start so we send at most `bytes`
  uint32_t start = (end > region_base + bytes) ? (end - bytes) : region_base;

  // Fragment into small CDC writes (don’t assume big buffers)
  uint32_t remaining = end - start;
  const uint8_t *p = (const uint8_t *)(NOR_BASE_ADDR + start);

  while (remaining) {
    uint16_t chunk = (remaining > 512u) ? 512u : (uint16_t)remaining;
    (void)CDC_Write(p, chunk);
    p         += chunk;
    remaining -= chunk;
    HAL_Delay(1);              // give USB stack time
  }
#else
  (void)bytes;
#endif
}
/* USER CODE END USB_DUMP */


int __io_putchar(int ch)
{
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); KP("BOOT", "HAL");

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); KP("CLK",  "sysclk ready");

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config(); KP("CLK",  "periph clocks ready");

  /* USER CODE BEGIN SysInit */

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_LPUART1_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &g);

  MX_LPUART1_UART_Init();
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\nBoOT OK (LPUART1 PG7/PG8)\r\n");

  // SPI1 pins: PA5=SCK, PA6=MISO, PA7=MOSI
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gp = {0};
  gp.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gp.Mode      = GPIO_MODE_AF_PP;
  gp.Pull      = GPIO_NOPULL;          // or PULLUP if your wiring prefers
  gp.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  gp.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &gp);


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init(); KP("I2C1", "init ok");
  MX_LPUART1_UART_Init();
  MX_SPI1_Init(); KP("SPI1", "init ok, mode=3");
  MX_ADC1_Init(); KP("ADC1", "init ok");
  MX_COMP1_Init();

  //printf("Before FMC Init()\r\n");

#if LOG_TO_MEMORY
  MX_FMC_Init();
  KP("FMC",  "init enter");
    MX_FMC_Init();                     // keep your function
    KP("FMC",  "init exit");
    KP("NOR",  "state=%s", g_nor_ok ? "ready" : "unavailable");
#endif

  //printf("After FMC Init() \r\n");

  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM3_Init(); KP("TIM",  "IC/base start (2,3,4,5)");
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init(); KP("USB",  "CDC device up");
  /* USER CODE BEGIN 2 */

  // Start input capture on TI1 and the counter
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start(&htim3);

    HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
      HAL_TIM_Base_Start(&htim4);

      HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_1);
        HAL_TIM_Base_Start(&htim5);


        __HAL_TIM_ENABLE(&htim2);
        __HAL_TIM_ENABLE(&htim5);
        __HAL_TIM_ENABLE(&htim3);
        __HAL_TIM_ENABLE(&htim4);


  MX_USB_DEVICE_Init();


  __HAL_RCC_GPIOG_CLK_ENABLE();
  GPIO_InitTypeDef g_fmc = {0};
  g_fmc.Pin = GPIO_PIN_14;
  g_fmc.Mode = GPIO_MODE_AF_PP;
  g_fmc.Pull = GPIO_NOPULL;
  g_fmc.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g_fmc.Alternate = GPIO_AF12_FMC;  // A25
  HAL_GPIO_Init(GPIOG, &g_fmc);


  HAL_ADC_Start(&hadc1);


  // call once, before TIM_All_CH1_ExternalClock_Init():
  GPIO_Init_TIM_CH1_Pins();
  TIM_All_CH1_ExternalClock_Init();
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);


  //TIMX_CH1 TESTING CODE END

  Reapply_COMP1_ForLMT01();


  printf("After Temp Sensor Reapplied\r\n");
  //TEMP SENSOR TESTING CODE END

  reset_second_window(&O, HAL_GetTick());

  //Power Profile GPT Code Start

    PowerProfile_Init();
    PowerProfile_Set(P1_NOM);            // start in your normal 100 Hz, 1 Hz compute mode
    PowerProfile_SetStressDuty(70);

    //Power Profile GPT Code End


//ALL SENSORS GPT CODE WORKING CODE BEGIN


#if ADXL312_PRESENT
{
    // Read WHOAMI
    uint8_t devid = ADXL312_ReadRegister(0x00);
    //printf("ADXL312: DEVID=0x%02X (expect 0xE5)\n", devid);

    // Configure ODR and range, and ensure 4-wire SPI (SPI bit = 0)
    // BW_RATE (0x2C)   = 0x0A (~100 Hz)
    // DATA_FORMAT (0x31) bits: [SELF_TEST, SPI, INT_INV, 0, FULL_RES, JUSTIFY, RANGE1, RANGE0]
    // Example: FULL_RES=1, ±12 g → 0b00001011 = 0x0B (SPI=0)
    ADXL312_WriteRegister(0x2C, 0x0A);
    ADXL312_WriteRegister(0x31, 0x0B);

    // Put into MEASURE mode (POWER_CTL 0x2D, bit3=1)
    ADXL312_WriteRegister(0x2D, 0x08);
    HAL_Delay(2);

    // Read back key regs to verify the state we THINK we set
    uint8_t pctl = ADXL312_ReadRegister(0x2D);
    uint8_t fmt  = ADXL312_ReadRegister(0x31);
    uint8_t rate = ADXL312_ReadRegister(0x2C);


    //KP("ADXL312", "cfg: ODR=100Hz, range=±12g, 4-wire");
}
#endif


  // --- ADXL312 ---
  ADXL312_Init();
  ADXL312_Config100Hz();
  while (!accelCalibrationComplete) { calibrateAccelerometer(); }

  // --- ADXL356 ---
  while (!accel356CalibrationComplete) { calibrateAccel356(); }

  // after the MID calibration loop you already have:
 // while (!accel356HI_CalibrationComplete) { calibrateAccel356_HI(); HAL_Delay(5); }

#if ACCEL_SELECTOR
  accel_selector_init(&a_sel, ACCEL_SRC_312);
#endif

  // --- LIS2MDL (you just reset this; keep the working version) ---

#if MAG_PRESENT
  uint8_t who=0;
  LIS2MDL_WhoAmI(&who);
  // printf("LIS2MDL WHO_AM_I=0x%02X\r\n", who);
  LIS2MDL_Init();
  LIS2MDL_Config100Hz();

  // Non-blocking calibrate (timeout so we never stall)
  uint32_t t0 = HAL_GetTick();
  while (!magCalibrationComplete && (HAL_GetTick() - t0 < 500)) {
    calibrateMagnetometer();
    HAL_Delay(5);
  }
#else
  printf("MAG skipped (bench)\r\n");
#endif


//ALL SENSORS GPT CODE WORKING CODE END
Log_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_ADC_Start(&hadc1); //CRUCIAL FOR ADXL356 COM
  while (1)
  {



	  //Power Profile GPT Code Start
	  uint32_t t0 = HAL_GetTick();

	    // Only run your normal 100 Hz path when not in P0_MIN
	    if (PowerProfile_Get() != P0_MIN) {
	      // >>> YOUR EXISTING 100 Hz CODE HERE <<<



	    	// in your command handler (over UART or CDC):
	    	// 'S' → stream a small window; 'D' with args → arbitrary window


	    	// --- 1) Read + calibrate both accels first ---
	    	float a312x=0, a312y=0, a312z=0;
	    	#if ADXL312_PRESENT
	    	{
	    		int16_t rx,ry,rz;
	    		if (ADXL312_ReadXYZ_Filtered(&rx,&ry,&rz)) {
	    		    getCalibratedAccel(rx,ry,rz,&a312x,&a312y,&a312z);
#if RAW_DATA
	    		    printf("ADXL312 raw=%d,%d,%d  g=%.3f,%.3f,%.3f\r\n",
	    		               rx, ry, rz, a312x, a312y, a312z);
	    		    HAL_Delay(1000);
#endif
	    		}


	    	}
	    	//printf("ADXL312 calibrated\r\n");
	    	#endif

	    	ADXL356_DualRaw r;

	    	#if ADXL356_MID_PRESENT

	    	  ADXL356_ReadSixChannels(&hadc1, &r);
	    	  float mid_x = volt_to_g_mid(adc_to_volt(r.mid_x_raw));
	    	  float mid_y = volt_to_g_mid(adc_to_volt(r.mid_y_raw));
	    	  float mid_z = volt_to_g_mid(adc_to_volt(r.mid_z_raw));
	    	  if (!accel356CalibrationComplete) calibrateAccel356();
	    	  getCalibratedAccel356(mid_x, mid_y, mid_z, &mid_x, &mid_y, &mid_z);
	    	  //printf("ADXL356 calibrated\r\n");
	    	#else
	    	  float mid_x=0, mid_y=0, mid_z=0;
	    	#endif






	    	  //adxl312_bootstrap_scale_from_356(1.5f);

	    	#if ADXL356_HI_PRESENT
	    	  float hix = volt_to_g_hi(adc_to_volt(r.hi_x_raw));
	    	  float hiy = volt_to_g_hi(adc_to_volt(r.hi_y_raw));
	    	  float hiz = volt_to_g_hi(adc_to_volt(r.hi_z_raw));
	    	  if (!accel356HI_CalibrationComplete) { calibrateAccel356_HI(); /* … */ }
	    	  float hi_cx, hi_cy, hi_cz;
	    	  getCalibratedAccel356_HI(hix, hiy, hiz, &hi_cx, &hi_cy, &hi_cz);
	    	#else
	    	  float hi_cx=0, hi_cy=0, hi_cz=0;
	    	#endif


	    	  // --- 1b) Decide which accel to use for this sample ---
	    	  float a312[3]  = { a312x, a312y, a312z };
	    	  float a356B[3] = { mid_x,  mid_y,  mid_z };
	    	  float a356C[3] = { 0, 0, 0 };
	    	  float a_use[3] = { mid_x, mid_y, mid_z };   // seed; will be overwritten by selector
	    	  //AccelSource active = accel_selector_update(a312, a356B, NULL, &a_sel, a_use);


	    	  // after accel_selector_update(...)
	    	  float n312 = sqrtf(a312x*a312x + a312y*a312y + a312z*a312z);
	    	  float n356 = sqrtf(mid_x*mid_x + mid_y*mid_y + mid_z*mid_z);

	    	  // simple validity gate for 312 (tune the bounds as you like)
	    	  bool a312_ok = isfinite(n312) && (n312 > 0.01f) && (n312 < 20.0f);

#if ACCEL_SELECTOR
	    	  AccelSource active = accel_selector_update(a312, a356B, NULL, &a_sel, a_use);


	    	  printf("[SEL] src=%s  |a312|=%.2f g  |a356|=%.2f g\r\n",
	    	         (active==ACCEL_SRC_312)?"ADXL312":"ADXL356MID", n312, n356);
#endif
	    	// --- 2) Read & calibrate magnetometer ---
	    	int16_t m_rx=0, m_ry=0, m_rz=0; float mx=0, my=0, mz=0;
	    	#if MAG_PRESENT
	    	  LIS2MDL_ReadRaw(&m_rx, &m_ry, &m_rz);
	    	  getCalibratedMag(m_rx, m_ry, m_rz, &mx, &my, &mz);
	    	#else
	    	  // leave mx,my,mz at 0 on bench so the rest of the math runs but doesn't block
	    	#endif

	    	  //printf("Mag calibrated\r\n");

	      // --- 3) Tilt compensate + horizontal soft-iron scaling ---
	      float ghat[3];  v3_norm(a_use, ghat);
	      float mx_b, my_b, mz_b, mxh, myh, mzh;
	      mag_remap(mx, my, mz, &mx_b, &my_b, &mz_b);
	      orientation_tilt_compensate_mag(ghat, mx_b, my_b, mz_b, &mxh, &myh, &mzh);
	      apply_horizontal_softiron(&mxh, &myh);

	      // globals
	      static float mxh_f=0, myh_f=0, mzh_f=0;

	      // after orientation_tilt_compensate_mag(...) and apply_horizontal_softiron(...)
	      const float beta = 0.20f;  // 0..1; 0.15–0.30 is a sweet spot
	      mxh_f += beta*(mxh - mxh_f);
	      myh_f += beta*(myh - myh_f);
	      mzh_f += beta*(mzh - mzh_f);

	      // accumulate one 100 Hz sample into the 1-s window
	      O.g_sum[0] += ghat[0];
	      O.g_sum[1] += ghat[1];
	      O.g_sum[2] += ghat[2];

	      O.mh_sum[0] += mxh_f;   // you already do this
	      O.mh_sum[1] += myh_f;
	      O.mh_sum[2] += mzh_f;

	      O.nsamp++;




	      // --- 5) Once per second: push mean into ring, compute angles ---
	      uint32_t now = HAL_GetTick();
	      if (now - O.t0_ms >= 1000U && O.nsamp > 0)
	      {
	          float g_mean[3]  = { O.g_sum[0]/O.nsamp, O.g_sum[1]/O.nsamp, O.g_sum[2]/O.nsamp };
	          float mh_mean[3] = { O.mh_sum[0]/O.nsamp, O.mh_sum[1]/O.nsamp, O.mh_sum[2]/O.nsamp };

	          // push once
	          O.g_hist[O.hist_idx][0]=g_mean[0];
	          O.g_hist[O.hist_idx][1]=g_mean[1];
	          O.g_hist[O.hist_idx][2]=g_mean[2];
	          O.mh_hist[O.hist_idx][0]=mh_mean[0];
	          O.mh_hist[O.hist_idx][1]=mh_mean[1];
	          O.mh_hist[O.hist_idx][2]=mh_mean[2];
	          if (O.hist_len < ORI_HIST_CAP) O.hist_len++;
	          O.hist_idx = (O.hist_idx + 1) % ORI_HIST_CAP;


	          // vector average over last <=5 seconds
	          float g5[3]={0,0,0}, m5[3]={0,0,0};
	          for (uint8_t i=0;i<O.hist_len;i++){
	              g5[0]+=O.g_hist[i][0]; g5[1]+=O.g_hist[i][1]; g5[2]+=O.g_hist[i][2];
	              m5[0]+=O.mh_hist[i][0]; m5[1]+=O.mh_hist[i][1]; m5[2]+=O.mh_hist[i][2];
	          }
	          g5[0]/=O.hist_len; g5[1]/=O.hist_len; g5[2]/=O.hist_len;
	          m5[0]/=O.hist_len; m5[1]/=O.hist_len; m5[2]/=O.hist_len;

	          v3_norm(g5, g5);

	          // ... after you have mx,my,mz and gx,gy,gz and lmt01_edges
	          float az_deg  = atan2f(my, mx) * (180.0f / M_PI);
	          if (az_deg < 0) az_deg += 360.0f;
	          float norm = sqrtf(mid_x * mid_x + mid_y * mid_y + mid_z * mid_z);
	          float gz = (norm > 0.0f) ? (mid_z / norm) : 0.0f;
	          float inc_deg = acosf(clampf(gz, -1.0f, 1.0f)) * (180.0f / M_PI);
float dip_deg = atan2f(mz, sqrtf(mx*mx + my*my)) * (180.0f / M_PI);
	          float t_c     = TEMP_PRESENT ? LMT01_PulsesToC(lmt01_edges) : NAN;

#if PRINT_TO_TERMINAL
	          printf("HDG az=%.1f inc=%.1f dip=%.1f T=%.2f C\r\n", az_deg, inc_deg, dip_deg, t_c);
HAL_Delay(1000);
	          #endif
	          #if USE_USB_CDC
	          {
	            char buf[96];
	            int len = snprintf(buf, sizeof(buf),
	                               "HDG az=%.1f inc=%.1f dip=%.1f T=%.2f C\r\n",
	                               az_deg, inc_deg, dip_deg, t_c);
	            if (len > 0) CDC_Write(buf, (uint16_t)len);
	          }
	          #endif

	          static uint32_t last2=0, last5=0, last3=0, last4=0;
	          uint32_t now2 = __HAL_TIM_GET_COUNTER(&htim2);
	          uint32_t now5 = __HAL_TIM_GET_COUNTER(&htim5);
	          uint32_t now3 = __HAL_TIM_GET_COUNTER(&htim3);
	          uint32_t now4 = __HAL_TIM_GET_COUNTER(&htim4);

	          uint32_t d2 = now2 - last2; last2 = now2;
	          uint32_t d5 = now5 - last5; last5 = now5;
	          uint32_t d3 = now3 - last3; last3 = now3;
	          uint32_t d4 = now4 - last4; last4 = now4;



	          #if LOG_TO_MEMORY
	          Log_Push(az_deg, inc_deg, dip_deg, t_c, d5, d2, d3, d4);
	          #endif

	          printf("Logged to memory \r\n");
#if RAW_DATA
{

    int16_t x312 = 0, y312 = 0, z312 = 0;

    //printf("312 A\n");                                // (1) reached branch
        ADXL312_ReadXYZ(&x312, &y312, &z312);             // (2) may hang if CS/format wrong
        //printf("312 B %d %d %d\n", x312, y312, z312);     // (3) read returned

        /* Keep your existing pretty print below (or add this single line): */
        //printf("RAW ADXL312: %d %d %d\r\n", x312, y312, z312);

    int16_t xmag = 0, ymag = 0, zmag = 0;
    ADXL356_DualRaw d356 = {0};

    if (ADXL312_PRESENT) {
        ADXL312_ReadXYZ(&x312, &y312, &z312);
        //printf("RAW ADXL312: %d %d %d\r\n", x312, y312, z312);
    }

    if (MAG_PRESENT && LIS2MDL_ReadRaw(&xmag, &ymag, &zmag)) {
        printf("RAW LIS2MDL: %d %d %d uT\r\n", xmag, ymag, zmag);
        HAL_Delay(1000);
    }

    if (ADXL356_MID_PRESENT) {
        ADXL356_ReadSixChannels(&hadc1, &d356);  // ensure hadc1 is correctly passed
        float xg = volt_to_g_mid(adc_to_volt(d356.mid_x_raw));
        float yg = volt_to_g_mid(adc_to_volt(d356.mid_y_raw));
        float zg = volt_to_g_mid(adc_to_volt(d356.mid_z_raw));


        // After you read mid_x_raw etc. and compute mid_x_g...
        printf("356MID raw=%u,%u,%u V=%.4f,%.4f,%.4f g=%.3f,%.3f,%.3f\r\n",
               r.mid_x_raw, r.mid_y_raw, r.mid_z_raw,
               adc_to_volt(r.mid_x_raw), adc_to_volt(r.mid_y_raw), adc_to_volt(r.mid_z_raw),
               mid_x, mid_y, mid_z);

        //printf("RAW ADXL356 (mid): %.2f %.2f %.2f g\r\n", xg, yg, zg);
    }
}
#endif




	      }

	      // - read sensors (ADXL312/356/LIS2MDL)
	      // - accumulate for 1 second
	      // - once per second: orientation calc and print
	      // (Whatever you have now stays right here)
	    }

	    // Add the high-power stress work (only does anything in P2_STRESS)
	    PowerProfile_Tick100Hz();

	    // Single 100 Hz pacer for the entire loop
	    uint32_t elapsed = HAL_GetTick() - t0;
	    if (elapsed < 10) HAL_Delay(10 - elapsed);
	  //Power Profile GPT Code End

	  //TESTING ACCEL_SELECTOR GPT CODE END




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_IO2;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0060112F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/* FMC initialization function */
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing = {0};

  hnor1.Instance = FMC_NORSRAM_DEVICE;
  hnor1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  hnor1.Init.NSBank = FMC_NORSRAM_BANK1;
  hnor1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hnor1.Init.MemoryType = FMC_MEMORY_TYPE_NOR;
  hnor1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hnor1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hnor1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hnor1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hnor1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hnor1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hnor1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hnor1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hnor1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hnor1.Init.WriteFifo = FMC_WRITE_FIFO_DISABLE;
  hnor1.Init.NBLSetupTime = 0;
  hnor1.Init.PageSize = FMC_PAGE_SIZE_NONE;

  /* Example conservative timings—keep yours if they worked */
  Timing.AddressSetupTime = 3;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 5;
  Timing.DataHoldTime = 1;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;

  /* NEW: don’t call Error_Handler() if this fails */
  KP("NOR",  "HAL_NOR_Init enter");
  HAL_StatusTypeDef st = HAL_NOR_Init(&hnor1, &Timing, NULL);
  KP("NOR",  "HAL_NOR_Init exit (st=%d)", (int)st);
  if (st != HAL_OK) {
    g_nor_ok = 0;
    printf("NOR init failed (st=%d). NOR logging disabled.\r\n", (int)st);
    return;
  }
  g_nor_ok = 1;
}


  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void adxl312_bootstrap_scale_from_356(float seconds)
{
    const uint32_t N = (uint32_t)(seconds * 100.0f); // 100 Hz loop
    double sum_raw_mag = 0.0, sum_356_mag = 0.0;
    uint32_t ok = 0;

    for (uint32_t i=0; i<N; ++i) {
        int16_t rx,ry,rz; float m356x,m356y,m356z;
        if (ADXL312_ReadXYZ_Filtered(&rx,&ry,&rz)) {
            // ADXL356 mid read in g (your existing API)
            ADXL356_ReadAccelG(&m356x,&m356y,&m356z);

            double rmag = sqrt((double)rx*rx + (double)ry*ry + (double)rz*rz);
            double gmag = sqrt((double)m356x*m356x + (double)m356y*m356y + (double)m356z*m356z);

            if (gmag > 0.2 && gmag < 2.0) { // sanity: we expect ~1 g at rest
                sum_raw_mag += rmag;
                sum_356_mag += gmag;
                ok++;
            }
        }
        HAL_Delay(10); // ~100 Hz
    }

    if (ok > 10) {
        float s = (float)(sum_raw_mag / sum_356_mag);
        if (s > 50.f && s < 1000.f) g_adxl312_lsb_per_g = s;
    }
}


extern SPI_HandleTypeDef hspi1;

//ADXL356 TESTING CODE START

uint16_t Read_ADC_Channel(void) {
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

//ADXL356 TESTING CODE END

//TEMP SENSOR TESTING CODE START
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hc){
  if (hc->Instance == COMP1) { lmt01_edges++; }
}

// make sure stm32l4xx_it.c calls HAL_COMP_IRQHandler(&hcomp1) in COMP_IRQHandler()

/*LMT01 pulses → Celsius */
static inline float LMT01_PulsesToC(uint32_t pulses){
  // LMT01 typical: N = 16 * (T + 50)  →  T(°C) = N/16 - 50
  return (pulses / 16.0f) - 50.0f;
}



//TEMP SENSOR TESTING CODE END

//TIMX_CH1 TESTING CODE START


  static void TIM_All_CH1_ExternalClock_Init(void)
  {
    TIM_IC_InitTypeDef ic = {0};
    ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    ic.ICSelection = TIM_ICSELECTION_DIRECTTI;   // TI1
    ic.ICPrescaler = TIM_ICPSC_DIV1;
    ic.ICFilter    = 2;                          // bump to 4 if noisy

    TIM_SlaveConfigTypeDef sl = {0};
    sl.SlaveMode    = TIM_SLAVEMODE_EXTERNAL1;   // External Clock Mode 1
    sl.InputTrigger = TIM_TS_TI1FP1;             // filtered TI1

    // TIM2 (32-bit)
    HAL_TIM_IC_Init(&htim2);
    HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_1);
    HAL_TIM_SlaveConfigSynchro(&htim2, &sl);
    __HAL_TIM_SET_PRESCALER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 0xFFFFFFFF);
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

    // TIM3 (16-bit)
    HAL_TIM_IC_Init(&htim3);
    HAL_TIM_IC_ConfigChannel(&htim3, &ic, TIM_CHANNEL_1);
    HAL_TIM_SlaveConfigSynchro(&htim3, &sl);
    __HAL_TIM_SET_PRESCALER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFF);
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);

    // TIM4 (16-bit)
    HAL_TIM_IC_Init(&htim4);
    HAL_TIM_IC_ConfigChannel(&htim4, &ic, TIM_CHANNEL_1);
    HAL_TIM_SlaveConfigSynchro(&htim4, &sl);
    __HAL_TIM_SET_PRESCALER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 0xFFFF);
    HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);

    // TIM5 (32-bit)
    HAL_TIM_IC_Init(&htim5);
    HAL_TIM_IC_ConfigChannel(&htim5, &ic, TIM_CHANNEL_1);
    HAL_TIM_SlaveConfigSynchro(&htim5, &sl);
    __HAL_TIM_SET_PRESCALER(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, 0xFFFFFFFF);
    HAL_TIM_IC_Start(&htim5, TIM_CHANNEL_1);
  }

  static void GPIO_Init_TIM_CH1_Pins(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = GPIO_PIN_15; g.Alternate = GPIO_AF1_TIM2; HAL_GPIO_Init(GPIOA, &g); // PA15 TIM2_CH1
    g.Pin = GPIO_PIN_4;  g.Alternate = GPIO_AF2_TIM3; HAL_GPIO_Init(GPIOB, &g); // PB4  TIM3_CH1
    g.Pin = GPIO_PIN_6;  g.Alternate = GPIO_AF2_TIM4; HAL_GPIO_Init(GPIOB, &g); // PB6  TIM4_CH1
    g.Pin = GPIO_PIN_6;  g.Alternate = GPIO_AF2_TIM5; HAL_GPIO_Init(GPIOF, &g); // PF6  TIM5_CH1
  }


  //COMP1 pulse counting for LMT01 */

  static void Reapply_COMP1_ForLMT01(void){
    HAL_COMP_Stop(&hcomp1);
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;   // EXTI on rising edges
    HAL_COMP_Init(&hcomp1);
    HAL_NVIC_SetPriority(COMP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(COMP_IRQn);
    HAL_COMP_Start_IT(&hcomp1);
  }

  /* USER CODE BEGIN 4 */



  static void ADXL356_ReadAccelG(float *gx, float *gy, float *gz)
  {
      ADXL356_DualRaw r;
      ADXL356_ReadSixChannels(&hadc1, &r);

  #if ADXL356_MID_PRESENT
      *gx = volt_to_g_mid(adc_to_volt(r.mid_x_raw));
      *gy = volt_to_g_mid(adc_to_volt(r.mid_y_raw));
      *gz = volt_to_g_mid(adc_to_volt(r.mid_z_raw));
  #elif ADXL356_HI_PRESENT
      *gx = volt_to_g_hi(adc_to_volt(r.hi_x_raw));
      *gy = volt_to_g_hi(adc_to_volt(r.hi_y_raw));
      *gz = volt_to_g_hi(adc_to_volt(r.hi_z_raw));
  #else
      *gx = *gy = *gz = 0.0f;
  #endif
  }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
