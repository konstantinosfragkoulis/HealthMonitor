/*
 * ecg_dsp.h
 *
 *  Created on: Mar 14, 2026
 *      Author: konstantinos
 */

#ifndef INC_ECG_DSP_H_
#define INC_ECG_DSP_H_

#include <stdint.h>

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Run one sample through the PT++ pipeline at 480 Hz. Returns 1 on R-peak. */
uint8_t ECG_Process_Sample(RawECG_t sample, HeartBeatEvent_t *out_event);

/* Returns 1 if a new HRV report is ready (every 32 beats). */
uint8_t ECG_Get_HRV_Report(HRVReport_t *report);

/* Reset all filter and threshold state. */
void ECG_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ECG_DSP_H_ */
