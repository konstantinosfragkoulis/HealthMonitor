/*
 * usb_descriptors.c
 *
 *  Created on: Feb 26, 2026
 *      Author: konstantinos
 */

#include "tusb.h"
#include "stm32u5xx_hal.h"
#include <stdio.h>
#include <string.h>

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

/* Replace with your own VID/PID if you have one. */
#define USB_VID   0xCAFE
#define USB_PID   0x4001
#define USB_BCD   0x0200

/* Device descriptor */
static tusb_desc_device_t const desc_device =
  { .bLength = sizeof(tusb_desc_device_t), .bDescriptorType = TUSB_DESC_DEVICE,
      .bcdUSB = USB_BCD, .bDeviceClass = TUSB_CLASS_MISC, .bDeviceSubClass =
          MISC_SUBCLASS_COMMON, .bDeviceProtocol = MISC_PROTOCOL_IAD,
      .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE, .idVendor = USB_VID,
      .idProduct = USB_PID, .bcdDevice = 0x0100, .iManufacturer = 0x01,
      .iProduct = 0x02, .iSerialNumber = 0x03, .bNumConfigurations = 0x01, };

uint8_t const* tud_descriptor_device_cb(void)
{
  return (uint8_t const*) &desc_device;
}

/* Configuration descriptor */
enum
{
  ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static uint8_t const desc_configuration[] =
  { TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
      TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
                         EPNUM_CDC_IN, 64), };

uint8_t const* tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

/* String descriptors */
static char const *string_desc_arr[] =
  { (const char[]
        )
          { 0x09, 0x04 }, /* 0: English (US) */
      "HealthMonitor", /* 1: Manufacturer */
      "HealthMonitor CDC", /* 2: Product      */
      NULL, /* 3: Serial (generated from UID at runtime) */
      "HealthMonitor CDC Port", /* 4: CDC interface */
  };

static uint16_t _desc_str[32 + 1];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if (index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }
  else if (index == 3)
  {
    /* Serial number from the STM32 96-bit unique device ID */
    uint32_t *uid = (uint32_t*) UID_BASE;
    char hex[25];
    snprintf(hex, sizeof(hex), "%08lX%08lX%08lX", uid[0], uid[1], uid[2]);
    chr_count = 24;
    for (uint8_t i = 0; i < chr_count; i++)
      _desc_str[1 + i] = hex[i];
  }
  else
  {
    if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
      return NULL;

    const char *str = string_desc_arr[index];
    chr_count = (uint8_t) strlen(str);
    if (chr_count > 31)
      chr_count = 31;

    for (uint8_t i = 0; i < chr_count; i++)
      _desc_str[1 + i] = str[i];
  }

  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

  return _desc_str;
}
