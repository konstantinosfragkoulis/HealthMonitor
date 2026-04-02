/*
 * ecg_dsp.c
 *
 * Reference:
 *   Imtiaz & Khan, "Pan-Tompkins++: A Robust Approach to Detect R-peaks
 *   in ECG Signals," Toronto Metropolitan University, 2022.
 *   https://github.com/Niaz-Imtiaz/Pan-Tompkins-Plus-Plus
 *
 * Hardware mapping:
 *   Fs   = 480 Hz  (ST1VAFE6AX ODR, also drives ECG ADC via shared EXTI)
 *   TIM2 = 10 kHz  (1 tick = 0.1 ms, 10 ticks = 1 ms)
 *
 * All timing constants are derived from these two values.
 *
 * Pipeline: bandpass 5-18 Hz -> 5pt derivative -> squaring -> flattop FIR
 *           -> MWI -> PT++ decision logic + search-back
 *
 * ~9 KB .bss (mostly the MWI history ring buffers)
 *
 *  Created on: Mar 14, 2026
 *      Author: konstantinos
 */

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "data_types.h"
#include "ecg_dsp.h"
#include "hrv.h"

#define FS                  480.0f          /* Sample rate (Hz)                  */
#define SAMPLES_PER_TICK    (FS / 10000.0f) /* 0.048 samples per TIM2 tick       */

/* Window sizes in samples — ceil(ms × Fs / 1000) */
#define FLATTOP_SIZE        29U     /* 60 ms  — PT++ flattop smoothing FIR       */
#define MWI_SIZE            72U     /* 150 ms — moving window integration        */
#define SLOPE_WIN_SIZE      34U     /* 70 ms  — T-wave slope comparison window   */

/* Timing thresholds in TIM2 ticks (10 kHz -> 1 tick = 0.1 ms) */
#define REFRACTORY_TICKS    2310U   /* 231 ms — max 260 BPM (PT++ refractory)    */
#define T_WAVE_TICKS        3600U   /* 360 ms — T-wave discrimination window     */
#define SEARCH_10S_TICKS    10000U  /* 1.0 s  — short search-back threshold      */
#define SEARCH_14S_TICKS    14000U  /* 1.4 s  — long search-back threshold       */
#define SB_COOLDOWN_TICKS   REFRACTORY_TICKS /* Min gap between search-back runs */

#define MIN_RR_TICKS        2310U   /* 231 ms -> 260 BPM max                      */
#define MAX_RR_TICKS        20000U  /* 2000 ms -> 30 BPM min                      */

#define RR_HISTORY_SIZE     32U
#define RR_MASK             (RR_HISTORY_SIZE - 1U)
#define T_WAVE_RR_COUNT     8U      /* PT++ specifies 8 most recent beats for T-wave */

#define HRV_REPORT_INTERVAL RR_HISTORY_SIZE  /* Emit HRV report every N beats */

/* MWI output history for search-back. Must be a power of 2 >= 1.4 s × 480 Hz = 672 samples */
#define MWI_HIST_SIZE       1024U
#define MWI_HIST_MASK       (MWI_HIST_SIZE - 1U)

/* Bandpass filter — cascaded HP + LP biquad sections (2nd-order Butterworth) */
#define HP_B0   ( 0.95474f)
#define HP_B1   (-1.90948f)
#define HP_B2   ( 0.95474f)
#define HP_A1   (-1.90752f)
#define HP_A2   ( 0.91154f)

#define LP_B0   ( 0.011857f)
#define LP_B1   ( 0.023714f)
#define LP_B2   ( 0.011857f)
#define LP_A1   (-1.66907f)
#define LP_A2   ( 0.71659f)

#define RULE1_ALPHA         0.125f
#define RULE1_BETA          0.875f
#define RULE2_ALPHA         0.75f
#define RULE2_BETA          0.25f

#define T_WAVE_SLOPE_RATIO  0.60f

static float hp_w1, hp_w2;
static float lp_w1, lp_w2;

static float der_x1, der_x2, der_x3, der_x4;

static float flattop_w[FLATTOP_SIZE];
static float flattop_buf[FLATTOP_SIZE];
static uint8_t flattop_head;

static float mwi_buf[MWI_SIZE];
static uint8_t mwi_head;
static float mwi_sum;

static float mwi_history[MWI_HIST_SIZE];
static uint32_t mwi_ts_history[MWI_HIST_SIZE];
static uint32_t sample_count;

static float slope_buf[SLOPE_WIN_SIZE];
static uint8_t slope_head;
static float slope_sum;
static float last_peak_slope;

static float signal_peak;
static float noise_peak;
static float threshold1;
static float threshold2;

static float prev_mwi;
static float local_max;
static float local_max_slope;
static uint32_t local_max_ts;
static int8_t peak_dir;

static uint32_t last_peak_time;
static uint32_t last_sb_time;

static uint32_t rr_history[RR_HISTORY_SIZE];
static uint8_t rr_head;
static uint8_t rr_count;
static uint32_t last_valid_peak;

static uint8_t initialized;

static uint8_t hrv_beat_count;
static uint8_t hrv_report_ready;
static HRVReport_t hrv_report;

static void init_flattop(void)
{
  /* Coefficients from the PT++ paper */
  const float a0 = 0.2155789f;
  const float a1 = 0.4166316f;
  const float a2 = 0.27726316f;
  const float a3 = 0.08357895f;
  const float a4 = 0.00694737f;
  float norm = 0.0f;

  for (uint8_t i = 0U; i < FLATTOP_SIZE; ++i)
  {
    float psi = (2.0f * (float) M_PI * (float) i) / (float) (FLATTOP_SIZE - 1U);
    flattop_w[i] = a0 - a1 * cosf(psi) + a2 * cosf(2.0f * psi)
        - a3 * cosf(3.0f * psi) + a4 * cosf(4.0f * psi);
    norm += flattop_w[i];
  }

  float inv_norm = 1.0f / norm;
  for (uint8_t i = 0U; i < FLATTOP_SIZE; ++i)
  {
    flattop_w[i] *= inv_norm;
  }
}

static float compute_mean_rr(void)
{
  uint32_t sum = 0U;
  uint8_t cnt = 0U;

  for (uint8_t i = 0U; i < RR_HISTORY_SIZE; ++i)
  {
    if (rr_history[i] != 0U)
    {
      sum += rr_history[i];
      cnt++;
    }
  }
  return (cnt > 0U) ? ((float) sum / (float) cnt) : 0.0f;
}

/* Mean of the N most recent RR intervals */
static float compute_recent_mean_rr(uint8_t n)
{
  uint32_t sum = 0U;
  uint8_t cnt = 0U;

  for (uint8_t i = 0U; i < n; ++i)
  {
    uint8_t idx = (uint8_t) ((rr_head - 1U - i + RR_HISTORY_SIZE) & RR_MASK);
    if (rr_history[idx] != 0U)
    {
      sum += rr_history[idx];
      cnt++;
    }
  }
  return (cnt > 0U) ? ((float) sum / (float) cnt) : 0.0f;
}

/* Update BPM and RR history after a confirmed R-peak */
static uint8_t calculate_biometrics(HeartBeatEvent_t *event)
{
  if (last_valid_peak == 0U)
  {
    last_valid_peak = event->timestamp;
    return 0U;
  }

  uint32_t curr_rr = event->timestamp - last_valid_peak;

  if (curr_rr > MAX_RR_TICKS)
  {
    /* Gap too long, reset baseline */
    last_valid_peak = event->timestamp;
    return 0U;
  }
  if (curr_rr < MIN_RR_TICKS)
    return 0U;

  last_valid_peak = event->timestamp;

  rr_history[rr_head] = curr_rr;
  rr_head = (uint8_t) ((rr_head + 1U) & RR_MASK);
  if (rr_count < RR_HISTORY_SIZE)
    rr_count++;

  /* HRV report every N beats */
  hrv_beat_count++;
  if (hrv_beat_count >= HRV_REPORT_INTERVAL && rr_count >= RR_HISTORY_SIZE)
  {
    if (HRV_Compute(rr_history, RR_HISTORY_SIZE, rr_head, rr_count,
                    0U, event->timestamp, &hrv_report))
    {
      hrv_report_ready = 1U;
    }
    hrv_beat_count = 0U;
  }

  /* BPM: 1 min = 60 000 ms = 600 000 ticks at 10 kHz */
  uint32_t rr_sum = 0U;
  uint8_t valid = 0U;
  for (uint8_t i = 0U; i < RR_HISTORY_SIZE; ++i)
  {
    if (rr_history[i] != 0U)
    {
      rr_sum += rr_history[i];
      valid++;
    }
  }
  if (valid > 0U)
  {
    float avg_rr = (float) rr_sum / (float) valid;
    event->bpm = (int16_t) (6000000.0f / avg_rr);
  }

  return 1U;
}

/* Scan MWI history for the highest peak in [start_ts, end_ts] */
static uint8_t search_back(uint32_t start_ts, uint32_t end_ts, float thresh,
                           HeartBeatEvent_t *event)
{
  float best_val = 0.0f;
  uint32_t best_ts = 0U;
  uint32_t window = end_ts - start_ts;

  /* Search at most 1.4 s of history plus some margin */
  uint32_t max_lookback = (uint32_t) ((float) SEARCH_14S_TICKS * SAMPLES_PER_TICK)
      + 10U;
  if (max_lookback > MWI_HIST_SIZE - 1U)
    max_lookback = MWI_HIST_SIZE - 1U;

  for (uint32_t i = 1U; i <= max_lookback; ++i)
  {
    uint32_t idx = (sample_count - i) & MWI_HIST_MASK;
    uint32_t ts = mwi_ts_history[idx];

    if ((ts - start_ts) <= window)
    {
      if (mwi_history[idx] > best_val)
      {
        best_val = mwi_history[idx];
        best_ts = ts;
      }
    }
  }

  if (best_val > thresh)
  {
    signal_peak = RULE2_ALPHA * best_val + RULE2_BETA * signal_peak;
    noise_peak = RULE2_ALPHA * best_val + RULE2_BETA * noise_peak;
    threshold1 = noise_peak + 0.25f * (signal_peak - noise_peak);
    threshold2 = 0.4f * threshold1;

    last_peak_time = best_ts;
    event->timestamp = best_ts;
    return 1U;
  }
  return 0U;
}

uint8_t ECG_Process_Sample(RawECG_t sample, HeartBeatEvent_t *out_event)
{
  if (!initialized)
  {
    init_flattop();
    signal_peak = 50.0f;
    noise_peak = 10.0f;
    threshold1 = noise_peak + 0.25f * (signal_peak - noise_peak);
    threshold2 = 0.4f * threshold1;
    last_peak_time = sample.timestamp;
    last_sb_time = sample.timestamp;
    initialized = 1U;
  }

  ++sample_count;

  /* Bandpass filter */
  float x = (float) sample.ecg;

  float hp_y = HP_B0 * x + hp_w1;
  hp_w1 = HP_B1 * x - HP_A1 * hp_y + hp_w2;
  hp_w2 = HP_B2 * x - HP_A2 * hp_y;

  float bp = LP_B0 * hp_y + lp_w1;
  lp_w1 = LP_B1 * hp_y - LP_A1 * bp + lp_w2;
  lp_w2 = LP_B2 * hp_y - LP_A2 * bp;

  /* Five-point derivative */
  float d = 0.125f * (bp + 2.0f * der_x1 - 2.0f * der_x3 - der_x4);
  der_x4 = der_x3;
  der_x3 = der_x2;
  der_x2 = der_x1;
  der_x1 = bp;

  float d_sq = d * d;

  float abs_d = fabsf(d);
  slope_sum -= slope_buf[slope_head];
  slope_sum += abs_d;
  slope_buf[slope_head] = abs_d;
  slope_head = (uint8_t) ((slope_head + 1U) % SLOPE_WIN_SIZE);

  /* Flattop window FIR smoothing */
  flattop_buf[flattop_head] = d_sq;
  flattop_head = (uint8_t) ((flattop_head + 1U) % FLATTOP_SIZE);

  float smoothed = 0.0f;
  for (uint8_t i = 0U; i < FLATTOP_SIZE; ++i)
  {
    uint8_t idx = (uint8_t) ((flattop_head + i) % FLATTOP_SIZE);
    smoothed += flattop_w[i] * flattop_buf[idx];
  }

  /* Moving window integration */
  mwi_sum += smoothed - mwi_buf[mwi_head];
  mwi_buf[mwi_head] = smoothed;
  mwi_head = (uint8_t) ((mwi_head + 1U) % MWI_SIZE);
  float mwi_out = mwi_sum / (float) MWI_SIZE;

  /* Store in history ring for search-back */
  uint32_t h_idx = sample_count & MWI_HIST_MASK;
  mwi_history[h_idx] = mwi_out;
  mwi_ts_history[h_idx] = sample.timestamp;

  /* Local maximum tracking */
  if (mwi_out > prev_mwi)
  {
    if (peak_dir != 1)
    {
      local_max = mwi_out;
      local_max_slope = slope_sum / (float) SLOPE_WIN_SIZE;
      local_max_ts = sample.timestamp;
      peak_dir = 1;
    }
    else if (mwi_out > local_max)
    {
      local_max = mwi_out;
      local_max_slope = slope_sum / (float) SLOPE_WIN_SIZE;
      local_max_ts = sample.timestamp;
    }
  }
  else if (peak_dir == 1 && mwi_out < prev_mwi)
  {
    /* Decision logic */
    peak_dir = -1;

    uint32_t time_since_last = sample.timestamp - last_peak_time;

    if (local_max > threshold1 && time_since_last > REFRACTORY_TICKS)
    {
      /* Candidate peak */
      uint8_t is_r_peak = 1U;

      if (rr_count >= 1U)
      {
        float mean_rr_val = compute_recent_mean_rr(T_WAVE_RR_COUNT);
        if (time_since_last <= T_WAVE_TICKS
            || (mean_rr_val > 0.0f
                && (float) time_since_last <= 0.5f * mean_rr_val))
        {
          if (local_max_slope < T_WAVE_SLOPE_RATIO * last_peak_slope)
          {
            noise_peak = RULE1_ALPHA * local_max + RULE1_BETA * noise_peak;
            is_r_peak = 0U;
          }
        }
      }

      if (is_r_peak)
      {
        signal_peak = RULE1_ALPHA * local_max + RULE1_BETA * signal_peak;
        last_peak_slope = local_max_slope;
        last_peak_time = local_max_ts;

        out_event->timestamp = local_max_ts;
        uint8_t biometrics_ok = calculate_biometrics(out_event);

        threshold1 = noise_peak + 0.25f * (signal_peak - noise_peak);
        threshold2 = 0.4f * threshold1;

        prev_mwi = mwi_out;
        return biometrics_ok;
      }

    }
    else if (local_max <= threshold1)
    {
      noise_peak = RULE1_ALPHA * local_max + RULE1_BETA * noise_peak;
    }

    threshold1 = noise_peak + 0.25f * (signal_peak - noise_peak);
    threshold2 = 0.4f * threshold1;
  }

  prev_mwi = mwi_out;

  /* Search-back */
  uint32_t gap = sample.timestamp - last_peak_time;
  uint32_t sb_cooldown = sample.timestamp - last_sb_time;

  if (sb_cooldown >= SB_COOLDOWN_TICKS)
  {
    float mean_rr = compute_mean_rr();

    uint8_t cond_a = (gap > SEARCH_10S_TICKS)
        || (mean_rr > 0.0f && (float) gap > 1.66f * mean_rr);
    uint8_t cond_b = (gap > SEARCH_14S_TICKS);

    if (cond_a || cond_b)
    {
      last_sb_time = sample.timestamp;

      uint32_t sb_start = last_peak_time + T_WAVE_TICKS;
      uint32_t sb_end = sample.timestamp;

      float thresh_sb;
      if (cond_b)
      {
        thresh_sb = 0.2f * threshold2;
      }
      else
      {
        thresh_sb = 0.5f * threshold2 + 0.5f * noise_peak;
      }

      if (search_back(sb_start, sb_end, thresh_sb, out_event))
      {
        if (calculate_biometrics(out_event))
          return 1U;
      }
    }
  }

  return 0U;
}

uint8_t ECG_Get_HRV_Report(HRVReport_t *report)
{
  if (!hrv_report_ready)
    return 0U;
  *report = hrv_report;
  hrv_report_ready = 0U;
  return 1U;
}

void ECG_Reset(void)
{
  hp_w1 = 0.0f;
  hp_w2 = 0.0f;
  lp_w1 = 0.0f;
  lp_w2 = 0.0f;
  der_x1 = 0.0f;
  der_x2 = 0.0f;
  der_x3 = 0.0f;
  der_x4 = 0.0f;

  memset(flattop_buf, 0, sizeof(flattop_buf));
  flattop_head = 0U;

  memset(mwi_buf, 0, sizeof(mwi_buf));
  mwi_head = 0U;
  mwi_sum = 0.0f;

  memset(mwi_history, 0, sizeof(mwi_history));
  memset(mwi_ts_history, 0, sizeof(mwi_ts_history));
  sample_count = 0U;

  memset(slope_buf, 0, sizeof(slope_buf));
  slope_head = 0U;
  slope_sum = 0.0f;
  last_peak_slope = 0.0f;

  signal_peak = 0.0f;
  noise_peak = 0.0f;
  threshold1 = 0.0f;
  threshold2 = 0.0f;

  prev_mwi = 0.0f;
  local_max = 0.0f;
  local_max_slope = 0.0f;
  local_max_ts = 0U;
  peak_dir = 0;

  last_peak_time = 0U;
  last_sb_time = 0U;

  memset(rr_history, 0, sizeof(rr_history));
  rr_head = 0U;
  rr_count = 0U;
  last_valid_peak = 0U;

  hrv_beat_count = 0U;
  hrv_report_ready = 0U;

  initialized = 0U;
}
