/*
 * resp_dsp.c
 *
 * Reference:
 *   Khreis et al. 2016: autocorrelation outperforms zero-crossing and FFT
 *   Lauteslager et al. 2021: PCA + autocorrelation achieves LOA [-1.9, +1.9] BrPM
 *
 * Pipeline: LP anti-alias 1Hz @ 480Hz -> decimate 48:1 -> BP 0.1-0.6Hz @ 10Hz
 *           -> 30s ring buffer -> autocorrelation every 5s
 *
 *  Created on: Mar 31, 2026
 *      Author: konstantinos
 */

#include <math.h>

#include "data_types.h"
#include "resp_dsp.h"

#define RESP_FS             10U       /* Decimated sample rate (Hz)           */
#define DECIMATE_RATIO      48U       /* 480 / 10                             */
#define RESP_BUF_SIZE       300U      /* 30 s at 10 Hz                        */
#define RESP_UPDATE_PERIOD  50U       /* 5 s at 10 Hz — rate update interval  */
#define RESP_MIN_SAMPLES    300U      /* 30 s minimum before first estimate   */

/*
 *  Autocorrelation lag bounds (in samples at 10 Hz):
 * 0.6 Hz -> period 1.667 s -> 16.7 samples -> 17
 * 0.1 Hz -> period 10.0 s  -> 100 samples  -> 100
 */
#define LAG_MIN             17U
#define LAG_MAX             100U

/* Anti-aliasing filter: 4th-order Butterworth LP, fc=1.0 Hz, Fs=480 Hz */
/* Two cascaded biquad sections, transposed direct-form II */

/* Section 0 */
#define AA0_B0  ( 0.000000001804f)
#define AA0_B1  ( 0.000000003608f)
#define AA0_B2  ( 0.000000001804f)
#define AA0_A1  (-1.975933280157f)
#define AA0_A2  ( 0.976102577659f)

/* Section 1 */
#define AA1_B0  ( 1.000000000000f)
#define AA1_B1  ( 2.000000000000f)
#define AA1_B2  ( 1.000000000000f)
#define AA1_A1  (-1.989861099913f)
#define AA1_A2  ( 0.990031590747f)

/* Respiratory bandpass: 2nd-order Butterworth, Fs=10 Hz */

/* HPF at 0.1 Hz */
#define RHP_B0  ( 0.9565432256f)
#define RHP_B1  (-1.9130864511f)
#define RHP_B2  ( 0.9565432256f)
#define RHP_A1  (-1.9111970674f)
#define RHP_A2  ( 0.9149758348f)

/* LPF at 0.6 Hz */
#define RLP_B0  ( 0.0278597661f)
#define RLP_B1  ( 0.0557195322f)
#define RLP_B2  ( 0.0278597661f)
#define RLP_A1  (-1.4754804436f)
#define RLP_A2  ( 0.5869195081f)

/* Anti-aliasing (2 sections at 480 Hz) */
static float aa0_w1, aa0_w2;
static float aa1_w1, aa1_w2;

/* Respiratory bandpass (at 10 Hz) */
static float rhp_w1, rhp_w2;
static float rlp_w1, rlp_w2;

static uint8_t decim_count;

static float resp_buf[RESP_BUF_SIZE];
static uint16_t resp_head;
static uint16_t resp_sample_count; /* Total decimated samples since reset    */
static uint16_t resp_update_count; /* Decimated samples since last estimate  */

static BreathEvent_t resp_event;
static uint8_t resp_event_ready;

/* Latest timestamp passed through anti-alias (used for event timestamping) */
static uint32_t resp_last_ts;

void RESP_Process_Sample(int16_t az, uint32_t timestamp)
{
  resp_last_ts = timestamp;
  float x = (float) az;

  /* Anti-aliasing LP filter (4th-order, two biquad sections) at 480 Hz */

  float y0 = AA0_B0 * x + aa0_w1;
  aa0_w1 = AA0_B1 * x - AA0_A1 * y0 + aa0_w2;
  aa0_w2 = AA0_B2 * x - AA0_A2 * y0;

  float y1 = AA1_B0 * y0 + aa1_w1;
  aa1_w1 = AA1_B1 * y0 - AA1_A1 * y1 + aa1_w2;
  aa1_w2 = AA1_B2 * y0 - AA1_A2 * y1;

  /* Decimation: keep every 48th sample */

  decim_count++;
  if (decim_count < DECIMATE_RATIO)
    return;
  decim_count = 0U;

  /* Respiratory bandpass filter at 10 Hz */

  /* HPF at 0.1 Hz */
  float hp = RHP_B0 * y1 + rhp_w1;
  rhp_w1 = RHP_B1 * y1 - RHP_A1 * hp + rhp_w2;
  rhp_w2 = RHP_B2 * y1 - RHP_A2 * hp;

  /* LPF at 0.6 Hz */
  float bp = RLP_B0 * hp + rlp_w1;
  rlp_w1 = RLP_B1 * hp - RLP_A1 * bp + rlp_w2;
  rlp_w2 = RLP_B2 * hp - RLP_A2 * bp;

  resp_buf[resp_head] = bp;
  resp_head = (resp_head + 1U) % RESP_BUF_SIZE;

  if (resp_sample_count < RESP_BUF_SIZE)
    resp_sample_count++;
  resp_update_count++;

  if (resp_update_count < RESP_UPDATE_PERIOD)
    return;
  resp_update_count = 0U;

  if (resp_sample_count < RESP_MIN_SAMPLES)
    return;

  /* Compute autocorrelation for lags in [LAG_MIN, LAG_MAX] */
  uint16_t n = (resp_sample_count < RESP_BUF_SIZE)
                   ? resp_sample_count
                   : RESP_BUF_SIZE;

  uint16_t best_lag = 0U;
  float r0 = 0.0f;

  /* R(0) for normalisation */
  for (uint16_t i = 0U; i < n; ++i)
  {
    uint16_t idx = (resp_head + RESP_BUF_SIZE - n + i) % RESP_BUF_SIZE;
    r0 += resp_buf[idx] * resp_buf[idx];
  }

  if (r0 < 1e-12f)
    return; /* Flat signal */

  float r_vals[3] = { 0.0f, 0.0f, 0.0f }; /* Sliding window: [lag-2, lag-1, lag] */
  uint8_t r_primed = 0U; /* Need 3 values before checking for a peak */

  for (uint16_t lag = LAG_MIN; lag <= LAG_MAX && lag < n; ++lag)
  {
    float r = 0.0f;
    for (uint16_t i = 0U; i < n - lag; ++i)
    {
      uint16_t idx_a = (resp_head + RESP_BUF_SIZE - n + i) % RESP_BUF_SIZE;
      uint16_t idx_b = (idx_a + lag) % RESP_BUF_SIZE;
      r += resp_buf[idx_a] * resp_buf[idx_b];
    }
    r /= r0; /* Normalised autocorrelation */

    /* Shift the sliding window */
    r_vals[0] = r_vals[1];
    r_vals[1] = r_vals[2];
    r_vals[2] = r;

    if (r_primed < 2U)
    {
      r_primed++;
      continue; /* Need at least 3 real R values before checking */
    }

    /* Detect the first local maximum: r_vals[1] > both its neighbours */
    if (r_vals[1] > r_vals[0] && r_vals[1] > r_vals[2] && r_vals[1] > 0.2f)
    {
      best_lag = lag - 1U;
      break; /* First peak found — this is the fundamental period */
    }
  }

  if (best_lag == 0U)
    return; /* No clear periodicity */

  /* Respiratory rate in tenths of BrPM: rate = 60 / (lag / Fs) × 10 */
  int16_t rate = (int16_t) ((600U * RESP_FS + best_lag / 2U) / best_lag);

  resp_event.rr = rate;
  resp_event.timestamp = resp_last_ts;
  resp_event_ready = 1U;
}

uint8_t RESP_Get_Event(BreathEvent_t *event)
{
  if (!resp_event_ready)
    return 0U;
  *event = resp_event;
  resp_event_ready = 0U;
  return 1U;
}

void RESP_Reset(void)
{
  aa0_w1 = 0.0f; aa0_w2 = 0.0f;
  aa1_w1 = 0.0f; aa1_w2 = 0.0f;
  rhp_w1 = 0.0f; rhp_w2 = 0.0f;
  rlp_w1 = 0.0f; rlp_w2 = 0.0f;

  decim_count = 0U;

  for (uint16_t i = 0U; i < RESP_BUF_SIZE; ++i)
    resp_buf[i] = 0.0f;
  resp_head = 0U;
  resp_sample_count = 0U;
  resp_update_count = 0U;

  resp_event_ready = 0U;
  resp_last_ts = 0U;
}
