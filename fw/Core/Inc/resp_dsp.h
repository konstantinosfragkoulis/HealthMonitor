/*
 * resp_dsp.h
 *
 * Pipeline: Z-axis @ 480 Hz -> LP anti-alias (1 Hz) -> decimate 48:1
 *           -> bandpass 0.1-0.6 Hz @ 10 Hz -> autocorrelation -> rate
 *
 *  Created on: Mar 31, 2026
 *      Author: konstantinos
 */

#ifndef INC_RESP_DSP_H_
#define INC_RESP_DSP_H_

#include <stdint.h>

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Process one accelerometer Z-axis sample at 480 Hz.
 * Internally decimates to 10 Hz and runs the respiratory pipeline.
 */
void RESP_Process_Sample(int16_t az, uint32_t timestamp);

/*
 * Retrieve the latest respiratory rate estimate.
 * Returns 1 and populates *event if a new estimate is ready, 0 otherwise.
 * A new estimate is produced every 5 seconds (every 50 decimated samples).
 */
uint8_t RESP_Get_Event(BreathEvent_t *event);

/* Reset all filter and buffer state. */
void RESP_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_RESP_DSP_H_ */
