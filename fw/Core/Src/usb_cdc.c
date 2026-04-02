/*
 * usb_cdc.c
 *
 *  Created on: Feb 26, 2026
 *      Author: konstantinos
 */

#include "tusb.h"
#include "usb_cdc.h"
#include "stm32u5xx_hal.h"

/* Required by TinyUSB for internal timing. */
uint32_t tusb_time_millis_api(void)
{
  return HAL_GetTick();
}

/* CDC class callbacks */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) dtr;
  (void) rts;
}

void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;

  char buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));
  (void) count;
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
  (void) itf;
  (void) p_line_coding;
}
