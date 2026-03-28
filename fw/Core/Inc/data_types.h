/*
 * data_types.h
 *
 *  Created on: Mar 14, 2026
 *      Author: konstantinos
 */

#ifndef INC_DATA_TYPES_H_
#define INC_DATA_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t timestamp; /* TIM2 counter value (10 kHz, 0.1 ms/tick) */
  int16_t ecg; /* Differential ECG value (ADC CH15 − CH16) */
  uint16_t reserved;
} RawECG_t;

typedef struct
{
  uint32_t timestamp; /* TIM2 counter value (10 kHz, 0.1 ms/tick) */
  int16_t x, y, z;
  uint16_t reserved;
} RawIMU_t;

/*
 * Emitted when the ECG algorithm detects an R-peak.
 *
 * Packed because this struct is serialised byte-for-byte over USB CDC.
 *
 * Scaling:
 *   BPM: x10
 *   RMSSD x10
 */
typedef struct __attribute__((packed))
{
  uint16_t header;
  int16_t bpm;
  uint32_t timestamp;
  uint16_t rmssd;
  uint8_t reserved;
  uint8_t crc;
} HeartBeatEvent_t;

typedef struct
{
  uint32_t timestamp;
  int16_t rr;
  uint8_t reserved;
  uint8_t crc;
} BreathEvent_t;

/* Packed to guarantee a 16 byte wire format. */
typedef struct __attribute__((packed))
{
  uint16_t header;
  int16_t ecg;
  uint32_t timestamp;
  int16_t x, y, z;
  uint8_t r_peak;
  uint8_t crc;
} TelemetryPacket_t;

/*
 * One slot in the lock-free sensor FIFO
 *
 * Status: 0=free, 1=DMA in progress, 2=ready to consume.
 *
 * adc[] is uint16_t even though HAL_ADC_Start_DMA() takes uint32_t*.
 * This works because GPDMA1 CH0 is configured for halfword transfers.
 */
typedef struct
{
  volatile uint32_t timestamp;
  uint16_t adc[3]; /**< ADC CH7, CH15, CH16 (halfword DMA)  */
  uint8_t imu[7]; /**< SPI RX: [0]=dummy, [1..6]=Z/Y/X     */
  volatile uint8_t status;
} DataSample_t;

#ifdef __cplusplus
}
#endif

#endif /* INC_DATA_TYPES_H_ */
