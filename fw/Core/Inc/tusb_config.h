/*
 * tusb_config.h
 *
 *  Created on: Feb 25, 2026
 *      Author: konstantinos
 *
 *  TinyUSB configuration for STM32U595RJT6Q – CDC (Virtual COM Port)
 *  USB OTG HS peripheral running in Full-Speed mode with embedded HS PHY
 */

#ifndef INC_TUSB_CONFIG_H_
#define INC_TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Board / Port Configuration
//--------------------------------------------------------------------+

// STM32U595 has only USB_OTG_HS (no OTG_FS).
// In TinyUSB's dwc2_stm32.h the HS controller is the first (and only)
// entry in _dwc2_controller[], so rhport index = 0.
#define BOARD_TUD_RHPORT      0

// Force Full-Speed operation (12 Mbit/s) – plenty for CDC serial.
#define BOARD_TUD_MAX_SPEED   OPT_MODE_FULL_SPEED

// Tell TinyUSB that rhport 0 is a Device port running at Full Speed.
// This is required so tusb_init() (zero-arg form) knows which port to use.
#define CFG_TUSB_RHPORT0_MODE  (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

//--------------------------------------------------------------------+
// Common Configuration
//--------------------------------------------------------------------+

#define CFG_TUSB_MCU          OPT_MCU_STM32U5

// Bare-metal (no RTOS)
#define CFG_TUSB_OS           OPT_OS_NONE

// 0 = no debug prints from TinyUSB internals.  Bump to 1-3 while debugging.
#define CFG_TUSB_DEBUG        0

// Enable the Device stack
#define CFG_TUD_ENABLED       1

// Max speed the device will advertise
#define CFG_TUD_MAX_SPEED     BOARD_TUD_MAX_SPEED

// DWC2 transfer mode: use FIFO/slave mode (not the DWC2-internal DMA).
// This has nothing to do with GPDMA – your SPI1/ADC1 DMA channels are unaffected.
#define CFG_TUD_DWC2_DMA_ENABLE    0
#define CFG_TUD_DWC2_SLAVE_ENABLE  1

// Memory placement – not needed for STM32U5 (all SRAM is USB-accessible)
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN    __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------+
// Device Configuration
//--------------------------------------------------------------------+

#define CFG_TUD_ENDPOINT0_SIZE   64

//------------- CLASS -------------//
#define CFG_TUD_CDC              1   // One CDC interface  → Virtual COM Port
#define CFG_TUD_MSC              0
#define CFG_TUD_HID              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

// CDC FIFO sizes (internal SW buffers inside TinyUSB)
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256

// CDC endpoint HW transfer buffer (max packet size)
// For Full-Speed this is 64 bytes.
#define CFG_TUD_CDC_EP_BUFSIZE   64

#ifdef __cplusplus
 }
#endif

#endif /* INC_TUSB_CONFIG_H_ */
