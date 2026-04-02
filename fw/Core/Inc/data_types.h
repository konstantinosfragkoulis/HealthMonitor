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
  int16_t ax, ay, az; /* Accelerometer */
  int16_t gx, gy, gz; /* Gyroscope */
} RawIMU_t;

/* Heartbeat event, packed for USB CDC serialisation. BPM is x10. */
typedef struct __attribute__((packed))
{
  uint16_t header;    /* 0xCCDD */
  int16_t bpm;        /* ×10 */
  uint8_t source;     /* 0 = ECG (R-peak), 1 = SCG (AO) */
  uint8_t reserved[3];
  uint32_t timestamp; /* TIM2 ticks of the event (R-peak or AO) */
  uint32_t crc;       /* Full 32-bit hardware CRC */
} HeartBeatEvent_t;   /* 16 bytes */

/* HRV report, emitted every 32 beats. Intervals in TIM2 ticks. */
typedef struct __attribute__((packed))
{
  uint16_t header;    /* 0xFFAA */
  int16_t bpm;        /* ×10 */
  uint16_t rmssd;     /* TIM2 ticks */
  uint16_t sdnn;      /* TIM2 ticks */
  uint16_t sd1;       /* TIM2 ticks */
  uint16_t sd2;       /* TIM2 ticks */
  uint8_t pnn50;      /* Percentage (0–100) */
  uint8_t n_beats;    /* Number of beats in the analysis window */
  uint8_t source;     /* 0 = ECG, 1 = SCG */
  uint8_t reserved;
  uint32_t timestamp; /* TIM2 ticks at time of report */
  uint32_t crc;       /* Full 32-bit hardware CRC */
} HRVReport_t;        /* 24 bytes */

typedef struct __attribute__((packed))
{
  uint16_t header;    /* 0xEEFF */
  int16_t rr;         /* Respiratory rate, ×10 (e.g. 162 = 16.2 BrPM) */
  uint32_t timestamp; /* TIM2 ticks */
  uint32_t crc;       /* Full 32-bit hardware CRC */
} BreathEvent_t;      /* 12 bytes */

typedef struct __attribute__((packed))
{
  uint16_t header;      /* 0xAABB */
  int16_t ecg;
  uint32_t timestamp;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint32_t crc;
} TelemetryPacket_t;    /* 24 bytes */

/* One FIFO slot. status: 0=free, 1=DMA busy, 2=ready */
typedef struct
{
  volatile uint32_t timestamp;
  uint16_t adc[3]; /* ADC CH7, CH15, CH16 (halfword DMA)  */
  uint8_t imu[13]; /* SPI RX: [0]=dummy, [1..6]=gyro, [7..12]=accel */
  volatile uint8_t status;
} DataSample_t;

#ifdef __cplusplus
}
#endif

#endif /* INC_DATA_TYPES_H_ */
