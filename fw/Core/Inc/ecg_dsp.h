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

/*
 * Process one ECG sample through the Pan-Tompkins++ pipeline.
 * Call once per sample at 480 Hz from the main loop (not from ISR).
 * Returns 1 if an R-peak was detected, 0 otherwise.
 * out_event is only valid when the return value is 1.
 */
uint8_t ECG_Process_Sample(RawECG_t sample, HeartBeatEvent_t *out_event);

/* Reset all filter/threshold state. Next call to ECG_Process_Sample()
 * will behave as if the device just booted. */
void ECG_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_ECG_DSP_H_ */
