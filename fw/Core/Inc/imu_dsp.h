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

void IMU_Process_Sample(RawIMU_t sample);

#endif /* INC_IMU_DSP_H_ */
