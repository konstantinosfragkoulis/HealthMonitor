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

#include "data_types.h"
#include "hrv.h"

#define NN50_THRESHOLD  500U  /* 50 ms in TIM2 ticks (10 kHz) */

uint8_t HRV_Compute(const uint32_t *rr_buf, uint8_t buf_size,
                    uint8_t head, uint8_t count,
                    uint8_t source, uint32_t timestamp,
                    HRVReport_t *report)
{
  uint8_t mask = buf_size - 1U;

  /* Mean RR interval */
  uint32_t rr_sum = 0U;
  uint8_t valid = 0U;
  for (uint8_t i = 0U; i < buf_size; ++i)
  {
    if (rr_buf[i] != 0U)
    {
      rr_sum += rr_buf[i];
      valid++;
    }
  }
  if (valid < 2U)
    return 0U;

  float mean_rr = (float) rr_sum / (float) valid;

  /* BPM x10: 1 minute = 600,000 TIM2 ticks at 10 kHz */
  report->bpm = (int16_t) (6000000.0f / mean_rr);

  /* SDNN: standard deviation of NN intervals */
  float var_sum = 0.0f;
  for (uint8_t i = 0U; i < buf_size; ++i)
  {
    if (rr_buf[i] != 0U)
    {
      float dev = (float) rr_buf[i] - mean_rr;
      var_sum += dev * dev;
    }
  }
  float sdnn = sqrtf(var_sum / (float) valid);
  report->sdnn = (uint16_t) sdnn;

  /* RMSSD + pNN50: from successive differences */
  float sq_sum = 0.0f;
  uint16_t nn50_count = 0U;
  uint8_t diff_n = 0U;
  report->rmssd = 0U;
  report->pnn50 = 0U;
  for (uint8_t i = 0U; i < buf_size - 1U; ++i)
  {
    uint8_t a = (uint8_t) ((head - 1U - i + buf_size) & mask);
    uint8_t b = (uint8_t) ((head - 2U - i + buf_size) & mask);
    if (rr_buf[a] != 0U && rr_buf[b] != 0U)
    {
      int32_t d = (int32_t) rr_buf[a] - (int32_t) rr_buf[b];
      sq_sum += (float) (d * d);
      if (d > (int32_t) NN50_THRESHOLD || d < -(int32_t) NN50_THRESHOLD)
        nn50_count++;
      diff_n++;
    }
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
  report->n_beats = valid;
  report->source = source;
  report->reserved = 0U;

  return 1U;
}
