/*
 * pid.h
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */

#ifndef SRC_COMMON_CORE_PID_H_
#define SRC_COMMON_CORE_PID_H_

#include "hw.h"


typedef struct _PIDSingle
{
	float kp;
	float ki;
	float kd;

	float reference;
	float meas_value;
	float meas_value_prev;
	float error;
	float error_sum;
	float error_deriv;
	float error_deriv_filt;

	float p_result;
	float i_result;
	float d_result;

	float pid_result;
}PIDSingle;

typedef struct _PIDDouble
{
	PIDSingle in;
	PIDSingle out;
}PIDDouble;


extern PIDDouble roll;
extern PIDDouble pitch;
extern PIDSingle yaw_heading;
extern PIDSingle yaw_rate;


void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle, float rate);
void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point, float value);
void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point, float angle, float rate);
void Reset_PID_Integrator(PIDSingle* axis);
void Reset_All_PID_Integrator(void);
void pidUpdate(void);

#endif /* SRC_COMMON_CORE_PID_H_ */
