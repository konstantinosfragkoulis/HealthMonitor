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
 * Compute HRV from a circular buffer of inter-beat intervals (TIM2 ticks).
 * buf_size must be a power of 2. Returns 0 if < 2 valid intervals.
 */
uint8_t HRV_Compute(const uint32_t *rr_buf, uint8_t buf_size,
                    uint8_t head, uint8_t count,
                    uint8_t source, uint32_t timestamp,
                    HRVReport_t *report);

#ifdef __cplusplus
}
#endif

#endif /* INC_HRV_H_ */
