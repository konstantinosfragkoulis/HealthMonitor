/*
 * scg_dsp.h
 *
 * Reference:
 *   Parlato et al. 2025, "Fully automated template matching method for
 *   ECG-free heartbeat detection in cardiomechanical signals"
 *   Physical and Engineering Sciences in Medicine, vol. 48, pp. 649-664.
 *
 *  Created on: Mar 31, 2026
 *      Author: konstantinos
 */

#ifndef INC_SCG_DSP_H_
#define INC_SCG_DSP_H_

#include <stdint.h>

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise SCG pipeline. Must be called once before SCG_Process_Sample. */
void SCG_Init(void);

/*
 * Process one accelerometer Z-axis sample at 480 Hz.
 * Runs the bandpass filter, template selection (first 10 s), and NCC detection.
 */
void SCG_Process_Sample(int16_t az, uint32_t timestamp);

/*
 * Retrieve the latest detected AO beat event.
 * Returns 1 and populates *event if a new AO beat was detected, 0 otherwise.
 */
uint8_t SCG_Get_Beat(HeartBeatEvent_t *event);

/*
 * Retrieve the latest SCG-derived HRV report.
 * Returns 1 if a new report is ready (every 32 beats), 0 otherwise.
 */
uint8_t SCG_Get_HRV_Report(HRVReport_t *report);

/* Compute SCG HRV on demand (synchronized with ECG). */
uint8_t SCG_Compute_HRV(uint32_t timestamp, HRVReport_t *report);

/* Reset all state. Next call enters buffering phase. */
void SCG_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_SCG_DSP_H_ */
