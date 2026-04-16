/*
 * hrv.c
 *
 * Computes time-domain and Poincare plot HRV metrics from a circular
 * buffer of inter-beat intervals. Used by both ECG (R-R intervals)
 * and SCG (AO-AO intervals) pipelines to produce identical metrics.
 *
 *  Created on: Apr 2, 2026
 *      Author: konstantinos
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "data_types.h"
#include "hrv.h"

#define NN50_THRESHOLD  500U  /* 50 ms in TIM2 ticks (10 kHz) */

uint8_t HRV_Compute(const uint32_t *rr_buf, uint8_t buf_size,
                    uint8_t head, uint8_t n_use,
                    uint8_t source, uint32_t timestamp,
                    HRVReport_t *report)
{
  uint8_t mask = buf_size - 1U;

  if (n_use < 2U)
    return 0U;

  /* Mean RR interval - iterate backward from head over n_use intervals */
  uint32_t rr_sum = 0U;
  for (uint8_t i = 0U; i < n_use; ++i)
  {
    uint8_t idx = (uint8_t) ((head - 1U - i + buf_size) & mask);
    rr_sum += rr_buf[idx];
  }
  float mean_rr = (float) rr_sum / (float) n_use;

  /* BPM x10: 1 minute = 600,000 TIM2 ticks at 10 kHz */
  report->bpm = (int16_t) (6000000.0f / mean_rr);

  /* SDNN: standard deviation of NN intervals (Bessel's correction) */
  float var_sum = 0.0f;
  for (uint8_t i = 0U; i < n_use; ++i)
  {
    uint8_t idx = (uint8_t) ((head - 1U - i + buf_size) & mask);
    float dev = (float) rr_buf[idx] - mean_rr;
    var_sum += dev * dev;
  }
  float sdnn = sqrtf(var_sum / (float) (n_use - 1U));
  report->sdnn = (uint16_t) sdnn;

  /* RMSSD + pNN50: from successive differences */
  float sq_sum = 0.0f;
  uint16_t nn50_count = 0U;
  uint8_t diff_n = 0U;
  report->rmssd = 0U;
  report->pnn50 = 0U;
  for (uint8_t i = 0U; i < n_use - 1U; ++i)
  {
    uint8_t a = (uint8_t) ((head - 1U - i + buf_size) & mask);
    uint8_t b = (uint8_t) ((head - 2U - i + buf_size) & mask);
    int32_t d = (int32_t) rr_buf[a] - (int32_t) rr_buf[b];
    sq_sum += (float) (d * d);
    if (d > (int32_t) NN50_THRESHOLD || d < -(int32_t) NN50_THRESHOLD)
      nn50_count++;
    diff_n++;
  }

  float rmssd = 0.0f;
  if (diff_n > 0U)
  {
    rmssd = sqrtf(sq_sum / (float) diff_n);
    report->rmssd = (uint16_t) rmssd;
    report->pnn50 = (uint8_t) ((nn50_count * 100U) / diff_n);
  }

  /* SD1 = RMSSD / sqrt(2) -- Shaffer & Ginsberg 2017 */
  float sd1 = rmssd * 0.7071068f;
  report->sd1 = (uint16_t) sd1;

  /* SD2 = sqrt(2 * SDNN^2 - SD1^2) */
  float sd2_sq = 2.0f * sdnn * sdnn - sd1 * sd1;
  report->sd2 = (sd2_sq > 0.0f) ? (uint16_t) sqrtf(sd2_sq) : 0U;

  report->timestamp = timestamp;
  report->n_beats = n_use;
  report->source = source;
  report->reserved = 0U;

  return 1U;
}

void BeatTracker_Init(BeatTracker_t *bt, uint32_t min_rr, uint8_t source)
{
  memset(bt->rr_buf, 0, sizeof(bt->rr_buf));
  bt->head = 0U;
  bt->count = 0U;
  bt->last_valid = 0U;
  bt->hrv_beat_count = 0U;
  bt->min_rr = min_rr;
  bt->source = source;
  bt->hrv_ready = 0U;
}

uint8_t BeatTracker_Update(BeatTracker_t *bt, uint32_t timestamp, int16_t *bpm)
{
  if (bt->last_valid == 0U)
  {
    bt->last_valid = timestamp;
    return 0U;
  }

  uint32_t curr_rr = timestamp - bt->last_valid;

  if (curr_rr > BEAT_TRACKER_MAX_RR)
  {
    bt->last_valid = timestamp;
    return 0U;
  }
  if (curr_rr < bt->min_rr)
    return 0U;

  bt->last_valid = timestamp;

  bt->rr_buf[bt->head] = curr_rr;
  bt->head = (uint8_t) ((bt->head + 1U) & BEAT_TRACKER_MASK);
  if (bt->count < BEAT_TRACKER_SIZE)
    bt->count++;

  bt->hrv_beat_count++;
  if (bt->hrv_beat_count >= HRV_TRIGGER_BEATS)
  {
    if (BeatTracker_ComputeHRV(bt, timestamp, &bt->hrv_report))
      bt->hrv_ready = 1U;
    bt->hrv_beat_count = 0U;
  }

  /* BPM from most recent BPM_WINDOW intervals for responsive display */
  uint8_t bpm_n = (bt->count < BPM_WINDOW) ? bt->count : BPM_WINDOW;
  uint32_t rr_sum = 0U;
  for (uint8_t i = 0U; i < bpm_n; ++i)
  {
    uint8_t idx = (uint8_t) ((bt->head - 1U - i + BEAT_TRACKER_SIZE)
                              & BEAT_TRACKER_MASK);
    rr_sum += bt->rr_buf[idx];
  }
  if (bpm_n > 0U)
    *bpm = (int16_t) (6000000.0f / ((float) rr_sum / (float) bpm_n));

  return 1U;
}

uint8_t BeatTracker_GetHRV(BeatTracker_t *bt, HRVReport_t *report)
{
  if (!bt->hrv_ready)
    return 0U;
  *report = bt->hrv_report;
  bt->hrv_ready = 0U;
  return 1U;
}

uint8_t BeatTracker_ComputeHRV(BeatTracker_t *bt, uint32_t timestamp,
                               HRVReport_t *report)
{
  if (bt->count < HRV_MIN_BEATS)
    return 0U;

  uint32_t window_dur = 0U;
  uint8_t n_use = 0U;
  for (uint8_t i = 0U; i < bt->count; ++i)
  {
    uint8_t idx = (uint8_t) ((bt->head - 1U - i + BEAT_TRACKER_SIZE)
                              & BEAT_TRACKER_MASK);
    window_dur += bt->rr_buf[idx];
    n_use++;
    if (window_dur >= HRV_MIN_WINDOW_TICKS)
      break;
  }

  if (window_dur < HRV_MIN_WINDOW_TICKS)
    return 0U;

  return HRV_Compute(bt->rr_buf, BEAT_TRACKER_SIZE, bt->head, n_use,
                     bt->source, timestamp, report);
}

float BeatTracker_MeanRR(const BeatTracker_t *bt)
{
  uint32_t sum = 0U;
  /* Use most recent 32 intervals for responsive mean (search-back, etc.) */
  uint8_t window = (bt->count < 32U) ? bt->count : 32U;
  for (uint8_t i = 0U; i < window; ++i)
  {
    uint8_t idx = (uint8_t) ((bt->head - 1U - i + BEAT_TRACKER_SIZE)
                              & BEAT_TRACKER_MASK);
    sum += bt->rr_buf[idx];
  }
  return (window > 0U) ? ((float) sum / (float) window) : 0.0f;
}

float BeatTracker_RecentMeanRR(const BeatTracker_t *bt, uint8_t n)
{
  uint32_t sum = 0U;
  uint8_t cnt = 0U;
  for (uint8_t i = 0U; i < n; ++i)
  {
    uint8_t idx = (uint8_t) ((bt->head - 1U - i + BEAT_TRACKER_SIZE)
                              & BEAT_TRACKER_MASK);
    if (bt->rr_buf[idx] != 0U)
    {
      sum += bt->rr_buf[idx];
      cnt++;
    }
  }
  return (cnt > 0U) ? ((float) sum / (float) cnt) : 0.0f;
}
