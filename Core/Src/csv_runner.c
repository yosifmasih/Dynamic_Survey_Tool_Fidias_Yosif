/*
 * csv_runner.c
 *
 *  Created on: Oct 17, 2025
 *      Author: fidia
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "dst_app_bridge.h"
#include <math.h>
#include "main.h"

#ifndef isnan
#define isnan(x) __builtin_isnan(x)
#endif

#define CSV_LINE_MAX 256

static char   g_line[CSV_LINE_MAX];
static size_t g_idx = 0;
static uint32_t g_sample_idx = 0;

static void csv_handle_line(char* line);

void csv_runner_init(void) {
  g_idx = 0;
  g_sample_idx = 0;
}

/* Call this from CDC RX (see step 2B) */
void csv_runner_on_rx(const uint8_t* buf, uint32_t len) {
  for (uint32_t i = 0; i < len; ++i) {
    char c = (char)buf[i];
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_idx < CSV_LINE_MAX-1 ? g_idx : CSV_LINE_MAX-1] = 0;
      csv_handle_line(g_line);
      g_idx = 0;
    } else {
      if (g_idx < CSV_LINE_MAX-1) g_line[g_idx++] = c;
      else g_idx = 0; // overflow: reset
    }
  }
}

static int is_header_or_empty(const char* s){
  while (*s && isspace((unsigned char)*s)) ++s;
  if (!*s) return 1;
  if (isalpha((unsigned char)*s)) return 1; // header
  return 0;
}

static void csv_handle_line(char* line) {
  if (is_header_or_empty(line)) return;

  // Replace commas with spaces so strtof works on either
  for (char* p = line; *p; ++p) if (*p == ',') *p = ' ';

  // Expected columns: ax ay az mx my mz [truth]
  float a[8] = {0};
  int n = 0;
  char* p = line;
  while (*p && n < 8) {
    while (*p && isspace((unsigned char)*p)) ++p;
    if (!*p) break;
    char* endp = NULL;
    a[n] = strtof(p, &endp);
    if (endp == p) break;
    n++; p = endp;
  }
  if (n < 6) return;

  DstRawSample s = {0};
  // If your CSV has only one accelerometer, mirror it to all three
  s.ax_lo = s.ax_mid = s.ax_hi = a[0];
  s.ay_lo = s.ay_mid = s.ay_hi = a[1];
  s.az_lo = s.az_mid = s.az_hi = a[2];
  s.mx = a[3]; s.my = a[4]; s.mz = a[5];
  s.temp_c = 0.0f;
  s.t_us = HAL_GetTick() * 1000U;

  DstAlgoOut out;
  dst_app_step(&s, &out);

  float truth = (n >= 7) ? a[6] : (float)NAN;

  // Print one CSV line back (capture on PC)
  // idx,incl,azim,dip,ml_pred,ml_final,rot,flow[,truth]
  printf("%lu,%.3f,%.3f,%.3f,%u,%u,%u,%u",
         (unsigned long)g_sample_idx++,
         out.incl_deg, out.azim_deg, out.dip_deg,
         (unsigned)out.ml_pred, (unsigned)out.ml_final,
         (unsigned)out.rot_flag, (unsigned)out.flow_flag);
  if (!isnan(truth)) printf(",%.3f", truth);
  printf("\r\n");
}


