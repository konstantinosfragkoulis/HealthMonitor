/*
 * hrv.h
 *
 * Used by both ECG (R-R intervals) and SCG (AO-AO intervals) pipelines.
 *
 *  Created on: Apr 2, 2026
 *      Author: konstantinos
 */

#ifndef INC_HRV_H_
#define INC_HRV_H_

#include <stdint.h>

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Compute HRV from the most recent n_use intervals in a circular buffer.
 * Iterates backward from head. buf_size must be a power of 2.
 * Returns 0 if n_use < 2.
 */
uint8_t HRV_Compute(const uint32_t *rr_buf, uint8_t buf_size,
                    uint8_t head, uint8_t n_use,
                    uint8_t source, uint32_t timestamp,
                    HRVReport_t *report);

/*
 * Shared inter-beat interval tracker used by both ECG (R-R) and SCG (AO-AO).
 * Manages the circular RR buffer, BPM calculation, and HRV report triggering.
 */

#define BEAT_TRACKER_SIZE    128U
#define BEAT_TRACKER_MASK    (BEAT_TRACKER_SIZE - 1U)
#define BEAT_TRACKER_MAX_RR  20000U    /* 2000 ms -> 30 BPM min              */

#define HRV_TRIGGER_BEATS    32U       /* Emit HRV report every 32 new beats */
#define HRV_MIN_BEATS        20U       /* Minimum beats for meaningful HRV   */
#define HRV_MIN_WINDOW_TICKS 600000U   /* 60 s in TIM2 ticks (10 kHz)        */
#define BPM_WINDOW           16U       /* Recent beats for responsive BPM    */

typedef struct
{
  uint32_t rr_buf[BEAT_TRACKER_SIZE];
  uint8_t head;
  uint8_t count;
  uint32_t last_valid;
  uint8_t hrv_beat_count;
  uint32_t min_rr;
  uint8_t source;
  HRVReport_t hrv_report;
  uint8_t hrv_ready;
} BeatTracker_t;

/* Initialise (or fully reset) a beat tracker instance. */
void BeatTracker_Init(BeatTracker_t *bt, uint32_t min_rr, uint8_t source);

/*
 * Record a beat at the given timestamp. Validates the interval, stores it,
 * triggers HRV when the window is full, and computes BPM.
 * Returns 1 and writes *bpm if a valid interval was recorded, 0 otherwise.
 */
uint8_t BeatTracker_Update(BeatTracker_t *bt, uint32_t timestamp, int16_t *bpm);

/* Retrieve the latest HRV report. Returns 1 if ready, 0 otherwise. */
uint8_t BeatTracker_GetHRV(BeatTracker_t *bt, HRVReport_t *report);

/* Compute HRV on demand (for synchronized reporting). */
uint8_t BeatTracker_ComputeHRV(BeatTracker_t *bt, uint32_t timestamp,
                               HRVReport_t *report);

/* Mean of the most recent 32 intervals in the buffer. */
float BeatTracker_MeanRR(const BeatTracker_t *bt);

/* Mean of the N most recent intervals (walking backward from head). */
float BeatTracker_RecentMeanRR(const BeatTracker_t *bt, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* INC_HRV_H_ */
