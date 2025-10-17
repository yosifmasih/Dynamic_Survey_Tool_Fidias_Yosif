/*
 * usbd_cdc_if.c
 *
 *  Created on: Oct 14, 2025
 *      Author: yosif
 */

// usb_log.h
#pragma once
#include <stdint.h>
void USB_DumpLogCSV_Header(void);
void USB_DumpLogCSV_Record(uint32_t ms, float az_deg, float inc_deg, float dip_deg, float temp_c);

// usb_log.c
#include "usbd_cdc_if.h"
#include <stdio.h>

static void cdc_puts(const char* s) { if (s) CDC_Transmit_FS((uint8_t*)s, (uint16_t)strlen(s)); }

void USB_DumpLogCSV_Header(void) {
    cdc_puts("time_ms,azimuth_deg,inclination_deg,dip_deg,temp_c\r\n");
}

void USB_DumpLogCSV_Record(uint32_t ms, float az, float inc, float dip, float tc) {
    char line[128];
    int n = snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f\r\n",
                     (unsigned long)ms, az, inc, dip, tc);
    if (n > 0) CDC_Transmit_FS((uint8_t*)line, (uint16_t)n);
}


int CDC_Write(const void *buf, uint16_t len) {
  return (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_OK) ? 0 : -1;
}

