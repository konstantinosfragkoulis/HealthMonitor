/*
 * scg_dsp.c
 *
 * Reference:
 *   Parlato et al. 2025, "Fully automated template matching method for
 *   ECG-free heartbeat detection in cardiomechanical signals"
 *
 * Pipeline: bandpass 7-30 Hz -> NCC against template -> adaptive threshold
 *
 * State machine: BUFFERING (10s) -> SELECTING -> DETECTING
 *
 * ~52 KB .bss (two 4800-sample cal buffers dominate)
 *
 *  Created on: Mar 31, 2026
 *      Author: konstantinos
 */

#include <stdint.h>
#include <string.h>
#include <math.h>

#include "data_types.h"
#include "scg_dsp.h"
#include "hrv.h"

#define SCG_FS              480U

#define TEMPLATE_LEN        336U     /* 700 ms at 480 Hz (200ms pre + 500ms post) */
#define TEMPLATE_PRE        96U      /* 200 ms × 480 Hz                           */
#define TEMPLATE_POST       240U     /* 500 ms × 480 Hz                           */

#define CAL_BUF_SIZE        4800U    /* 10 s at 480 Hz                            */
#define CAL_MIN_PEAKS       5U       /* Minimum heartbeats in 10 s window         */
#define CAL_MAD_THRESH      161U     /* 335 ms × 480/1000 ≈ 161 samples           */
#define CAL_MAX_PEAKS       20U      /* Max peaks we'll track during selection     */
#define CAL_PEAK_DIST       168U     /* 350 ms min distance between envelope peaks */
#define CAL_PROM_THRESH     0.25f    /* Min prominence on normalised envelope      */
#define NCC_LAG_RANGE       50U      /* ±50 samples for pairwise NCC alignment    */

#define REFRACTORY_SAMPLES  168U     /* 350 ms -> 171 BPM max                      */
#define REFRACTORY_TICKS    (REFRACTORY_SAMPLES * 10000U / SCG_FS) /* 3500 TIM2    */
#define NCC_ALPHA           0.4f     /* Adaptive threshold coefficient             */
#define RULE1_A             0.125f   /* EMA fast coefficient (Rule-1)              */
#define RULE1_B             0.875f   /* EMA slow coefficient (Rule-1)              */
#define RULE2_A             0.75f    /* EMA fast coefficient (Rule-2, search-back) */
#define RULE2_B             0.25f    /* EMA slow coefficient (Rule-2, search-back) */

#define RR_HISTORY_SIZE     32U
#define RR_MASK             (RR_HISTORY_SIZE - 1U)
#define HRV_REPORT_INTERVAL RR_HISTORY_SIZE
#define MIN_RR_TICKS        3500U    /* ~350 ms -> 171 BPM                         */
#define MAX_RR_TICKS        20000U   /* 2000 ms -> 30 BPM                          */

#define REFRESH_INTERVAL    (60U * SCG_FS)  /* 60 s between template refreshes    */
#define REFRESH_NCC_THRESH  0.7f     /* Min NCC to accept refreshed template       */
#define RECOMPUTE_INTERVAL  (30U * SCG_FS)  /* 30 s running sum recomputation     */

/* NCC history for search-back (must be power of 2) */
#define NCC_HIST_SIZE       1024U
#define NCC_HIST_MASK       (NCC_HIST_SIZE - 1U)
#define SB_TRIGGER_RATIO    1.5f     /* Search-back if gap > 1.5 × mean AO-AO     */
#define SB_MAX_GAP_TICKS    30000U   /* 3 s absolute max before search-back        */
#define SB_COOLDOWN_TICKS   REFRACTORY_TICKS /* Min gap between search-back runs   */
#define SB_THRESH_RATIO     0.5f     /* Search-back threshold = 0.5 × ncc_thresh   */

/* Bandpass: 4th-order Butterworth 7-30 Hz, Fs=480 Hz */

/* HPF at 7 Hz */
#define SHP_B0  ( 0.9372603903f)
#define SHP_B1  (-1.8745207805f)
#define SHP_B2  ( 0.9372603903f)
#define SHP_A1  (-1.8705806407f)
#define SHP_A2  ( 0.8784609203f)

/* LPF at 30 Hz */
#define SLP_B0  ( 0.0299545822f)
#define SLP_B1  ( 0.0599091644f)
#define SLP_B2  ( 0.0299545822f)
#define SLP_A1  (-1.4542435863f)
#define SLP_A2  ( 0.5740619151f)

/* Envelope LP at 3 Hz */
#define ELP_B0  ( 0.0003750696f)
#define ELP_B1  ( 0.0007501392f)
#define ELP_B2  ( 0.0003750696f)
#define ELP_A1  (-1.9444776578f)
#define ELP_A2  ( 0.9459779362f)

/* State machine phases */

typedef enum
{
  SCG_PHASE_BUFFERING,
  SCG_PHASE_SELECTING,
  SCG_PHASE_DETECTING
} scg_phase_t;

/* Static state */

/* Bandpass filter */
static float shp_w1, shp_w2;
static float slp_w1, slp_w2;

/* Phase */
static scg_phase_t phase;
static uint32_t sample_idx;          /* Total samples since init */

/* Calibration buffer (also used for template refresh) */
static float cal_buf[CAL_BUF_SIZE];
static uint16_t cal_head;
static uint16_t cal_count;           /* Samples written since last selection */

/* Active template */
static float scg_template[TEMPLATE_LEN];
static float template_mean;
static float template_denom;         /* sqrt(SUM(t[k]-μ)^2) */
static uint16_t template_ao_offset;  /* Index of AO peak within template*/

static float pending_template[TEMPLATE_LEN];

static float scg_ring[TEMPLATE_LEN];
static uint32_t scg_ts_ring[TEMPLATE_LEN];
static uint16_t ring_head;
static float running_sum;
static float running_sum2;

static float ncc_signal_peak;
static float ncc_noise_peak;
static float ncc_threshold;
static float prev_ncc;
static float local_max_ncc;
static uint32_t local_max_ncc_ts;
static uint16_t local_max_ncc_ring_head;
static int8_t ncc_peak_dir;
static uint32_t last_ao_time;

static uint32_t refresh_counter;
static uint32_t recompute_counter;

static float ncc_history[NCC_HIST_SIZE];
static uint32_t ncc_ao_ts_history[NCC_HIST_SIZE];
static uint32_t ncc_sample_count;
static uint32_t last_sb_time;

static uint32_t ao_rr_history[RR_HISTORY_SIZE];
static uint8_t ao_rr_head;
static uint8_t ao_rr_count;
static uint32_t last_valid_ao;
static uint8_t ao_hrv_beat_count;

static HeartBeatEvent_t ao_beat_event;
static uint8_t ao_beat_ready;
static HRVReport_t ao_hrv_report;
static uint8_t ao_hrv_ready;

static float bandpass(float x)
{
  /* HPF at 7 Hz */
  float hp = SHP_B0 * x + shp_w1;
  shp_w1 = SHP_B1 * x - SHP_A1 * hp + shp_w2;
  shp_w2 = SHP_B2 * x - SHP_A2 * hp;

  /* LPF at 30 Hz */
  float lp = SLP_B0 * hp + slp_w1;
  slp_w1 = SLP_B1 * hp - SLP_A1 * lp + slp_w2;
  slp_w2 = SLP_B2 * hp - SLP_A2 * lp;

  return lp;
}

static void precompute_template_stats(const float *tmpl, uint16_t len)
{
  float sum = 0.0f;
  for (uint16_t i = 0U; i < len; ++i)
    sum += tmpl[i];
  template_mean = sum / (float) len;

  float sq_dev = 0.0f;
  uint16_t max_idx = 0U;
  float max_val = tmpl[0];
  for (uint16_t i = 0U; i < len; ++i)
  {
    float d = tmpl[i] - template_mean;
    sq_dev += d * d;
    if (tmpl[i] > max_val)
    {
      max_val = tmpl[i];
      max_idx = i;
    }
  }
  template_denom = sqrtf(sq_dev);
  template_ao_offset = max_idx;
}

static float zero_lag_ncc(const float *a, const float *b, uint16_t len)
{
  float sum_a = 0.0f, sum_b = 0.0f;
  for (uint16_t i = 0U; i < len; ++i)
  {
    sum_a += a[i];
    sum_b += b[i];
  }
  float mean_a = sum_a / (float) len;
  float mean_b = sum_b / (float) len;

  float cross = 0.0f, var_a = 0.0f, var_b = 0.0f;
  for (uint16_t i = 0U; i < len; ++i)
  {
    float da = a[i] - mean_a;
    float db = b[i] - mean_b;
    cross += da * db;
    var_a += da * da;
    var_b += db * db;
  }
  float denom = sqrtf(var_a * var_b);
  if (denom < 1e-12f)
    return 0.0f;
  return cross / denom;
}

/* Max NCC over +-lag_range between two segments */
static float max_ncc_pair(const float *a, const float *b, uint16_t len,
                          uint16_t lag_range)
{
  float best = -2.0f;

  for (int16_t lag = -(int16_t) lag_range; lag <= (int16_t) lag_range; ++lag)
  {
    uint16_t start_a = (lag >= 0) ? (uint16_t) lag : 0U;
    uint16_t start_b = (lag < 0) ? (uint16_t) (-lag) : 0U;
    uint16_t overlap = len - ((lag >= 0) ? (uint16_t) lag : (uint16_t) (-lag));
    if (overlap < len / 2U)
      continue;

    float sum_a = 0.0f, sum_b = 0.0f;
    for (uint16_t i = 0U; i < overlap; ++i)
    {
      sum_a += a[start_a + i];
      sum_b += b[start_b + i];
    }
    float mean_a = sum_a / (float) overlap;
    float mean_b = sum_b / (float) overlap;

    float cross = 0.0f, var_a = 0.0f, var_b = 0.0f;
    for (uint16_t i = 0U; i < overlap; ++i)
    {
      float da = a[start_a + i] - mean_a;
      float db = b[start_b + i] - mean_b;
      cross += da * db;
      var_a += da * da;
      var_b += db * db;
    }
    float denom = sqrtf(var_a * var_b);
    if (denom < 1e-12f)
      continue;

    float ncc = cross / denom;
    if (ncc > best)
      best = ncc;
  }

  return best;
}

/* Simple insertion sort for small arrays */
static void sort_u16(uint16_t *arr, uint8_t n)
{
  for (uint8_t i = 1U; i < n; ++i)
  {
    uint16_t key = arr[i];
    int8_t j = (int8_t) i - 1;
    while (j >= 0 && arr[j] > key)
    {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

/* Extract segments around envelope peaks, pairwise NCC, pick best template. */
static uint8_t select_template_from_peaks(const float *filtered,
                                          const uint16_t *peak_locs,
                                          uint8_t n_peaks,
                                          float *out_template)
{
  /* Clamp peaks that are too close to buffer edges */
  uint8_t valid_peaks[CAL_MAX_PEAKS];
  uint8_t n_valid = 0U;
  for (uint8_t i = 0U; i < n_peaks; ++i)
  {
    if (peak_locs[i] >= TEMPLATE_PRE
        && peak_locs[i] + TEMPLATE_POST < CAL_BUF_SIZE)
    {
      valid_peaks[n_valid++] = i;
    }
  }
  if (n_valid < CAL_MIN_PEAKS)
    return 0U;

  /* Pairwise max NCC */
  float ncc_matrix[CAL_MAX_PEAKS][CAL_MAX_PEAKS];
  memset(ncc_matrix, 0, sizeof(ncc_matrix));

  for (uint8_t i = 0U; i < n_valid; ++i)
  {
    ncc_matrix[i][i] = 1.0f;
    const float *seg_i = &filtered[peak_locs[valid_peaks[i]] - TEMPLATE_PRE];

    for (uint8_t j = i + 1U; j < n_valid; ++j)
    {
      const float *seg_j = &filtered[peak_locs[valid_peaks[j]] - TEMPLATE_PRE];
      float ncc = max_ncc_pair(seg_i, seg_j, TEMPLATE_LEN, NCC_LAG_RANGE);
      ncc_matrix[i][j] = ncc;
      ncc_matrix[j][i] = ncc;
    }
  }

  /* Select template with highest mean/std ratio */
  float best_ratio = -1.0f;
  uint8_t best_idx = 0U;

  for (uint8_t i = 0U; i < n_valid; ++i)
  {
    float sum = 0.0f;
    for (uint8_t j = 0U; j < n_valid; ++j)
      sum += ncc_matrix[i][j];
    float mean = sum / (float) n_valid;

    float var = 0.0f;
    for (uint8_t j = 0U; j < n_valid; ++j)
    {
      float d = ncc_matrix[i][j] - mean;
      var += d * d;
    }
    float sd = sqrtf(var / (float) n_valid);

    float ratio = (sd > 1e-6f) ? (mean / sd) : mean * 1e6f;
    if (ratio > best_ratio)
    {
      best_ratio = ratio;
      best_idx = i;
    }
  }

  /* Copy the best segment as the template */
  uint16_t start = peak_locs[valid_peaks[best_idx]] - TEMPLATE_PRE;
  memcpy(out_template, &filtered[start], TEMPLATE_LEN * sizeof(float));

  return 1U;
}

/* Copy of cal_buf taken before envelope computation overwrites it.
 * We still need the original filtered signal for segment extraction. */
static float cal_buf_filtered[CAL_BUF_SIZE];

/* Run the full template selection pipeline. Overwrites cal_buf with envelope. */
static uint8_t run_template_selection(float *dest_template)
{
  /* Save the filtered signal before we overwrite cal_buf with the envelope */
  memcpy(cal_buf_filtered, cal_buf, CAL_BUF_SIZE * sizeof(float));

  /* Compute 4th-power envelope + LP filter at 3 Hz */
  float env_w1 = 0.0f, env_w2 = 0.0f;
  float env_max = 0.0f;

  for (uint16_t i = 0U; i < CAL_BUF_SIZE; ++i)
  {
    float x = cal_buf[i];
    float x2 = x * x;
    float x4 = x2 * x2;

    float y = ELP_B0 * x4 + env_w1;
    env_w1 = ELP_B1 * x4 - ELP_A1 * y + env_w2;
    env_w2 = ELP_B2 * x4 - ELP_A2 * y;

    cal_buf[i] = y; /* cal_buf now holds envelope */
    if (y > env_max)
      env_max = y;
  }

  if (env_max < 1e-12f)
    return 0U;

  /* Normalise envelope */
  float inv_max = 1.0f / env_max;
  for (uint16_t i = 0U; i < CAL_BUF_SIZE; ++i)
    cal_buf[i] *= inv_max;

  /* Detect envelope peaks */
  uint16_t peak_locs[CAL_MAX_PEAKS];
  uint8_t n_peaks = 0U;

  for (uint16_t i = 1U; i < CAL_BUF_SIZE - 1U; ++i)
  {
    if (cal_buf[i] > CAL_PROM_THRESH && cal_buf[i] > cal_buf[i - 1U]
        && cal_buf[i] > cal_buf[i + 1U])
    {
      if (n_peaks > 0U && (i - peak_locs[n_peaks - 1U]) < CAL_PEAK_DIST)
      {
        if (cal_buf[i] > cal_buf[peak_locs[n_peaks - 1U]])
          peak_locs[n_peaks - 1U] = i;
        continue;
      }
      if (n_peaks < CAL_MAX_PEAKS)
        peak_locs[n_peaks++] = i;
    }
  }

  if (n_peaks < CAL_MIN_PEAKS)
    return 0U;

  /* Compute IBIs and MAD */
  uint16_t ibis[CAL_MAX_PEAKS - 1U];
  for (uint8_t i = 0U; i < n_peaks - 1U; ++i)
    ibis[i] = peak_locs[i + 1U] - peak_locs[i];

  uint8_t n_ibis = n_peaks - 1U;

  uint32_t ibi_sum = 0U;
  for (uint8_t i = 0U; i < n_ibis; ++i)
    ibi_sum += ibis[i];
  uint16_t mean_ibi = (uint16_t) (ibi_sum / n_ibis);

  uint16_t abs_devs[CAL_MAX_PEAKS - 1U];
  for (uint8_t i = 0U; i < n_ibis; ++i)
  {
    int16_t dev = (int16_t) ibis[i] - (int16_t) mean_ibi;
    abs_devs[i] = (dev >= 0) ? (uint16_t) dev : (uint16_t) (-dev);
  }
  sort_u16(abs_devs, n_ibis);
  uint16_t mad = abs_devs[n_ibis / 2U];

  if (mad > CAL_MAD_THRESH)
    return 0U;

  /* Select template from filtered signal using detected peaks */
  return select_template_from_peaks(cal_buf_filtered, peak_locs, n_peaks,
                                    dest_template);
}

static float compute_ncc(void)
{
  float mean_s = running_sum / (float) TEMPLATE_LEN;
  float var_s = running_sum2 / (float) TEMPLATE_LEN - mean_s * mean_s;

  if (var_s < 1e-12f || template_denom < 1e-6f)
    return 0.0f;

  float denom_s = sqrtf(var_s * (float) TEMPLATE_LEN);

  float cross = 0.0f;
  for (uint16_t k = 0U; k < TEMPLATE_LEN; ++k)
  {
    uint16_t idx = (ring_head + k) % TEMPLATE_LEN;
    cross += (scg_ring[idx] - mean_s) * (scg_template[k] - template_mean);
  }

  float ncc = cross / (denom_s * template_denom);
  return (ncc > 0.0f) ? ncc : 0.0f;
}

/* Returns 1 if biometrics were populated, 0 if not enough data */
static uint8_t ao_calculate_biometrics(uint32_t ao_ts)
{
  if (last_valid_ao == 0U)
  {
    last_valid_ao = ao_ts;
    return 0U;
  }

  uint32_t curr_rr = ao_ts - last_valid_ao;

  if (curr_rr > MAX_RR_TICKS)
  {
    /* Gap too long, reset baseline */
    last_valid_ao = ao_ts;
    return 0U;
  }
  if (curr_rr < MIN_RR_TICKS)
    return 0U;

  last_valid_ao = ao_ts;

  ao_rr_history[ao_rr_head] = curr_rr;
  ao_rr_head = (uint8_t) ((ao_rr_head + 1U) & RR_MASK);
  if (ao_rr_count < RR_HISTORY_SIZE)
    ao_rr_count++;

  ao_hrv_beat_count++;
  if (ao_hrv_beat_count >= HRV_REPORT_INTERVAL && ao_rr_count >= RR_HISTORY_SIZE)
  {
    if (HRV_Compute(ao_rr_history, RR_HISTORY_SIZE, ao_rr_head, ao_rr_count,
                    1U, ao_ts, &ao_hrv_report))
    {
      ao_hrv_ready = 1U;
    }
    ao_hrv_beat_count = 0U;
  }

  /* BPM */
  uint32_t rr_sum = 0U;
  uint8_t valid = 0U;
  for (uint8_t i = 0U; i < RR_HISTORY_SIZE; ++i)
  {
    if (ao_rr_history[i] != 0U)
    {
      rr_sum += ao_rr_history[i];
      valid++;
    }
  }
  if (valid > 0U)
  {
    float avg_rr = (float) rr_sum / (float) valid;
    ao_beat_event.bpm = (int16_t) (6000000.0f / avg_rr);
  }

  return 1U;
}

static float scg_compute_mean_aoao(void)
{
  uint32_t sum = 0U;
  uint8_t cnt = 0U;
  for (uint8_t i = 0U; i < RR_HISTORY_SIZE; ++i)
  {
    if (ao_rr_history[i] != 0U)
    {
      sum += ao_rr_history[i];
      cnt++;
    }
  }
  return (cnt > 0U) ? ((float) sum / (float) cnt) : 0.0f;
}

/* Search NCC history for missed beats. Returns 1 if a peak was found. */
static uint8_t scg_search_back(uint32_t current_ts, float thresh,
                               uint32_t *out_ts, float *out_ncc)
{
  float best_ncc = 0.0f;
  uint32_t best_ts = 0U;
  uint32_t earliest = last_ao_time + REFRACTORY_TICKS;

  uint32_t max_look = (ncc_sample_count < NCC_HIST_SIZE)
                        ? ncc_sample_count : NCC_HIST_SIZE;
  /* Don't search further than ~2.5 s (1200 samples at 480 Hz) */
  if (max_look > 1200U)
    max_look = 1200U;

  for (uint32_t i = 1U; i <= max_look; ++i)
  {
    uint32_t idx = (ncc_sample_count - i) & NCC_HIST_MASK;
    uint32_t ts = ncc_ao_ts_history[idx];

    if (ts >= earliest && ts <= current_ts)
    {
      if (ncc_history[idx] > best_ncc && ncc_history[idx] > thresh)
      {
        best_ncc = ncc_history[idx];
        best_ts = ts;
      }
    }
  }

  if (best_ts != 0U)
  {
    *out_ts = best_ts;
    *out_ncc = best_ncc;
    return 1U;
  }
  return 0U;
}

/* Public API */

void SCG_Init(void)
{
  phase = SCG_PHASE_BUFFERING;
  sample_idx = 0U;
  cal_head = 0U;
  cal_count = 0U;
  ring_head = 0U;
  running_sum = 0.0f;
  running_sum2 = 0.0f;

  ncc_signal_peak = 0.3f;
  ncc_noise_peak = 0.1f;
  ncc_threshold = ncc_noise_peak + NCC_ALPHA * (ncc_signal_peak - ncc_noise_peak);
  prev_ncc = 0.0f;
  local_max_ncc = 0.0f;
  local_max_ncc_ts = 0U;
  local_max_ncc_ring_head = 0U;
  ncc_peak_dir = 0;
  last_ao_time = 0U;

  refresh_counter = 0U;
  recompute_counter = 0U;

  ncc_sample_count = 0U;
  last_sb_time = 0U;

  ao_beat_ready = 0U;
  ao_hrv_ready = 0U;
}

void SCG_Process_Sample(int16_t az, uint32_t timestamp)
{
  float filtered = bandpass((float) az);
  sample_idx++;

  float old = scg_ring[ring_head];
  scg_ring[ring_head] = filtered;
  scg_ts_ring[ring_head] = timestamp;
  running_sum += filtered - old;
  running_sum2 += filtered * filtered - old * old;
  ring_head = (ring_head + 1U) % TEMPLATE_LEN;

  cal_buf[cal_head] = filtered;
  cal_head = (cal_head + 1U) % CAL_BUF_SIZE;
  if (cal_count < CAL_BUF_SIZE)
    cal_count++;

  switch (phase)
  {
    case SCG_PHASE_BUFFERING:
      if (cal_count >= CAL_BUF_SIZE)
        phase = SCG_PHASE_SELECTING;
      break;

    case SCG_PHASE_SELECTING:
    {
      /* Linearise circular buffer if needed */
      if (cal_head != 0U)
      {
        memcpy(cal_buf_filtered, &cal_buf[cal_head],
               (CAL_BUF_SIZE - cal_head) * sizeof(float));
        memcpy(&cal_buf_filtered[CAL_BUF_SIZE - cal_head], cal_buf,
               cal_head * sizeof(float));
        memcpy(cal_buf, cal_buf_filtered, CAL_BUF_SIZE * sizeof(float));
      }

      if (run_template_selection(scg_template))
      {
        precompute_template_stats(scg_template, TEMPLATE_LEN);
        last_ao_time = timestamp;
        phase = SCG_PHASE_DETECTING;
      }
      else
      {
        /* Template selection failed — retry after another 10 s */
        cal_count = 0U;
        cal_head = 0U;
        phase = SCG_PHASE_BUFFERING;
      }
      break;
    }

    case SCG_PHASE_DETECTING:
    {
      /* NCC */
      float ncc = compute_ncc();

      /* Store in history for search-back */
      {
        uint32_t h = ncc_sample_count & NCC_HIST_MASK;
        ncc_history[h] = ncc;
        uint16_t ao_pos = (ring_head + template_ao_offset) % TEMPLATE_LEN;
        ncc_ao_ts_history[h] = scg_ts_ring[ao_pos];
        ncc_sample_count++;
      }

      /* Peak detection */
      uint32_t time_since_ao = timestamp - last_ao_time;

      if (ncc > prev_ncc)
      {
        if (ncc_peak_dir != 1)
        {
          local_max_ncc = ncc;
          local_max_ncc_ts = timestamp;
          local_max_ncc_ring_head = ring_head;
          ncc_peak_dir = 1;
        }
        else if (ncc > local_max_ncc)
        {
          local_max_ncc = ncc;
          local_max_ncc_ts = timestamp;
          local_max_ncc_ring_head = ring_head;
        }
      }
      else if (ncc_peak_dir == 1 && ncc < prev_ncc)
      {
        ncc_peak_dir = -1;

        if (local_max_ncc > ncc_threshold
            && time_since_ao > REFRACTORY_TICKS)
        {
          /* AO detected — update adaptive threshold */
          ncc_signal_peak = RULE1_A * local_max_ncc + RULE1_B * ncc_signal_peak;

          /* Look up AO timestamp from ring buffer */
          uint16_t ao_idx = (local_max_ncc_ring_head + template_ao_offset)
                            % TEMPLATE_LEN;
          uint32_t ao_ts = scg_ts_ring[ao_idx];
          last_ao_time = ao_ts;

          ao_beat_event.timestamp = ao_ts;
          ao_beat_event.source = 1U; /* SCG */
          if (ao_calculate_biometrics(ao_ts))
            ao_beat_ready = 1U;
        }
        else if (local_max_ncc <= ncc_threshold)
        {
          ncc_noise_peak = RULE1_A * local_max_ncc + RULE1_B * ncc_noise_peak;
        }

        ncc_threshold = ncc_noise_peak
                        + NCC_ALPHA * (ncc_signal_peak - ncc_noise_peak);
      }

      prev_ncc = ncc;

      /* Search-back */
      {
        uint32_t ao_gap = timestamp - last_ao_time;
        uint32_t sb_cd = timestamp - last_sb_time;

        if (sb_cd >= SB_COOLDOWN_TICKS)
        {
          float mean_aoao = scg_compute_mean_aoao();

          uint8_t should_search =
              (mean_aoao > 0.0f && (float) ao_gap > SB_TRIGGER_RATIO * mean_aoao)
              || (ao_gap > SB_MAX_GAP_TICKS);

          if (should_search)
          {
            last_sb_time = timestamp;
            float sb_thresh = SB_THRESH_RATIO * ncc_threshold;
            uint32_t found_ts;
            float found_ncc;

            if (scg_search_back(timestamp, sb_thresh, &found_ts, &found_ncc))
            {
              ncc_signal_peak = RULE2_A * found_ncc
                                + RULE2_B * ncc_signal_peak;
              ncc_noise_peak = RULE2_A * found_ncc
                               + RULE2_B * ncc_noise_peak;
              ncc_threshold = ncc_noise_peak
                              + NCC_ALPHA * (ncc_signal_peak - ncc_noise_peak);

              last_ao_time = found_ts;
              ao_beat_event.timestamp = found_ts;
              ao_beat_event.source = 1U;
              if (ao_calculate_biometrics(found_ts))
                ao_beat_ready = 1U;
            }
          }
        }
      }

      /* Template refresh (every 60s) */
      refresh_counter++;
      if (refresh_counter >= REFRESH_INTERVAL && cal_count >= CAL_BUF_SIZE)
      {
        refresh_counter = 0U;

        /* Linearise the circular cal_buf */
        if (cal_head != 0U)
        {
          memcpy(cal_buf_filtered, &cal_buf[cal_head],
                 (CAL_BUF_SIZE - cal_head) * sizeof(float));
          memcpy(&cal_buf_filtered[CAL_BUF_SIZE - cal_head], cal_buf,
                 cal_head * sizeof(float));
          memcpy(cal_buf, cal_buf_filtered, CAL_BUF_SIZE * sizeof(float));
          cal_head = 0U;
        }

        if (run_template_selection(pending_template))
        {
          /* Only swap if the new template is morphologically similar */
          float compat = zero_lag_ncc(pending_template, scg_template,
                                      TEMPLATE_LEN);
          if (compat > REFRESH_NCC_THRESH)
          {
            memcpy(scg_template, pending_template,
                   TEMPLATE_LEN * sizeof(float));
            precompute_template_stats(scg_template, TEMPLATE_LEN);
            /* Recompute running sums after swap */
            running_sum = 0.0f;
            running_sum2 = 0.0f;
            for (uint16_t k = 0U; k < TEMPLATE_LEN; ++k)
            {
              running_sum += scg_ring[k];
              running_sum2 += scg_ring[k] * scg_ring[k];
            }
          }
          /* else keep old template */
        }
      }

      /* Recompute running sums periodically to avoid drift */
      recompute_counter++;
      if (recompute_counter >= RECOMPUTE_INTERVAL)
      {
        recompute_counter = 0U;
        running_sum = 0.0f;
        running_sum2 = 0.0f;
        for (uint16_t k = 0U; k < TEMPLATE_LEN; ++k)
        {
          running_sum += scg_ring[k];
          running_sum2 += scg_ring[k] * scg_ring[k];
        }
      }
      break;
    }
  }
}

uint8_t SCG_Get_Beat(HeartBeatEvent_t *event)
{
  if (!ao_beat_ready)
    return 0U;
  *event = ao_beat_event;
  ao_beat_ready = 0U;
  return 1U;
}

uint8_t SCG_Get_HRV_Report(HRVReport_t *report)
{
  if (!ao_hrv_ready)
    return 0U;
  *report = ao_hrv_report;
  ao_hrv_ready = 0U;
  return 1U;
}

void SCG_Reset(void)
{
  shp_w1 = 0.0f; shp_w2 = 0.0f;
  slp_w1 = 0.0f; slp_w2 = 0.0f;

  memset(cal_buf, 0, sizeof(cal_buf));
  memset(cal_buf_filtered, 0, sizeof(cal_buf_filtered));
  memset(scg_template, 0, sizeof(scg_template));
  memset(pending_template, 0, sizeof(pending_template));
  memset(scg_ring, 0, sizeof(scg_ring));
  memset(scg_ts_ring, 0, sizeof(scg_ts_ring));
  memset(ao_rr_history, 0, sizeof(ao_rr_history));
  memset(ncc_history, 0, sizeof(ncc_history));
  memset(ncc_ao_ts_history, 0, sizeof(ncc_ao_ts_history));

  ao_rr_head = 0U;
  ao_rr_count = 0U;
  last_valid_ao = 0U;
  ao_hrv_beat_count = 0U;
  ao_beat_ready = 0U;
  ao_hrv_ready = 0U;

  SCG_Init();
}
