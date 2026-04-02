/*
 * imu_dsp.h
 *
 *  Created on: Mar 14, 2026
 *      Author: konstantinos
 */

#ifndef INC_IMU_DSP_H_
#define INC_IMU_DSP_H_

#include <stdint.h>

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise all IMU DSP sub-pipelines. Call once before IMU_Process_Sample. */
void IMU_Init(void);

/* Process one IMU sample at 480 Hz. Feeds both respiratory and SCG pipelines */
void IMU_Process_Sample(RawIMU_t sample);

/* Poll for SCG heartbeat event. Returns 1 if an AO beat was detected. */
uint8_t IMU_Get_Beat(HeartBeatEvent_t *event);

/* Poll for respiratory rate event. Returns 1 if a new estimate is ready. */
uint8_t IMU_Get_Breath(BreathEvent_t *event);

/* Poll for SCG-derived HRV report. Returns 1 if a new report is ready. */
uint8_t IMU_Get_HRV_Report(HRVReport_t *report);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_DSP_H_ */
