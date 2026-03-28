/*
 * usb_cdc.h
 *
 *  Created on: Feb 26, 2026
 *      Author: konstantinos
 */

#ifndef INC_USB_CDC_H_
#define INC_USB_CDC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void USB_Print(const char* str);
void USB_Write(const uint8_t* buf, size_t len);

uint32_t tusb_time_millis_api(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_USB_CDC_H_ */
