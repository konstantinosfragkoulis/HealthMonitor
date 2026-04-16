/*
 * imu_dsp.c
 *
 * Routes RawIMU_t samples to the respiratory and SCG sub-pipelines.
 * Both pipelines use only the accelerometer Z-axis (dorso-ventral) for now.
 * Gyroscope data is streamed in telemetry but not processed on-device yet.
 *
 *  Created on: Mar 14, 2026
 *      Author: konstantinos
 */

#include "data_types.h"
#include "imu_dsp.h"
#include "resp_dsp.h"
#include "scg_dsp.h"

void IMU_Init(void)
{
  SCG_Init();
  /* RESP has no init */
}

void IMU_Process_Sample(RawIMU_t sample)
{
  RESP_Process_Sample(sample.az, sample.timestamp);
  SCG_Process_Sample(sample.az, sample.timestamp);
}

uint8_t IMU_Get_Beat(HeartBeatEvent_t *event)
{
  return SCG_Get_Beat(event);
}

uint8_t IMU_Get_Breath(BreathEvent_t *event)
{
  return RESP_Get_Event(event);
}

uint8_t IMU_Get_HRV_Report(HRVReport_t *report)
{
  return SCG_Get_HRV_Report(report);
}

uint8_t IMU_Compute_HRV(uint32_t timestamp, HRVReport_t *report)
{
  return SCG_Compute_HRV(timestamp, report);
}

void IMU_Reset(void)
{
  SCG_Reset();
  RESP_Reset();
}
