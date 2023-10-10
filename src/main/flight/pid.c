/*
 * pid.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */
#include "accgyro/bmi270.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/runtime_config.h"
#include "flight/mixer.h"
#include "flight/core.h"
#include "rx/rx.h"


PIDDouble roll;
PIDDouble pitch;
PIDSingle yaw_heading;
PIDSingle yaw_rate;

unsigned short ccr1, ccr2, ccr3, ccr4;

float yaw_heading_reference;

#define DT 0.000312f
#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1

void Double_Roll_Pitch_PID_Calculation(PIDDouble* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Double PID Outer Begin (Roll and Pitch Angular Position Control) *************/
	axis->out.reference = set_point_angle;	//Set point of outer PID control
	axis->out.meas_value = angle;			//BNO080 rotation angle

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop
	axis->out.p_result = axis->out.error * axis->out.kp;			//Calculate P result of outer loop

	axis->out.error_sum = axis->out.error_sum + axis->out.error * DT;	//Define summation of outer loop
#define OUT_ERR_SUM_MAX 500
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
	else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Calculate I result of outer loop

	axis->out.error_deriv = -rate;										//Define derivative of outer loop (rate = ICM-20602 Angular Rate)

#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Calculate D result of inner loop
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop
	/****************************************************************************************/

	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) **************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = rate;					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;	//Define summation of inner loop
#define IN_ERR_SUM_MAX 500
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->out.error_sum > IN_ERR_SUM_MAX) axis->out.error_sum = IN_ERR_SUM_MAX;
	else if(axis->out.error_sum < IN_I_ERR_MIN) axis->out.error_sum = IN_I_ERR_MIN;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Calculate I result of inner loop

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Define derivative of inner loop
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif

	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
}

void Single_Yaw_Heading_PID_Calculation(PIDSingle* axis, float set_point_angle, float angle/*Rotation Angle*/, float rate/*Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Position) *************/
	axis->reference = set_point_angle;	//Set point of yaw heading @ yaw stick is center.
	axis->meas_value = angle;			//Current BNO080_Yaw angle @ yaw stick is center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw angle control

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;

	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw angle control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw angle control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw angle control

	axis->error_deriv = -rate;						//Define differentiation of yaw angle control
	axis->d_result = axis->error_deriv * axis->kd;	//Calculate D result of yaw angle control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw angle control
	/***************************************************************/
}

void Single_Yaw_Rate_PID_Calculation(PIDSingle* axis, float set_point_rate, float rate/*Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_rate;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = rate;			//Current ICM20602.gyro_z @ yaw stick is not center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw rate control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;	//Define differentiation of yaw rate control
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
}

void Reset_PID_Integrator(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_All_PID_Integrator(void)
{
	Reset_PID_Integrator(&roll.in);
	Reset_PID_Integrator(&roll.out);
	Reset_PID_Integrator(&pitch.in);
	Reset_PID_Integrator(&pitch.out);
	Reset_PID_Integrator(&yaw_heading);
	Reset_PID_Integrator(&yaw_rate);
}

void pidUpdate(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);

    Double_Roll_Pitch_PID_Calculation(&pitch, (rcData[1] - 1500) * 0.1f, (attitude.values.pitch/10), mpu.gyro.gyroADC[X]);
	Double_Roll_Pitch_PID_Calculation(&roll, (rcData[0] - 1500) * 0.1f, (attitude.values.roll/10), mpu.gyro.gyroADC[Y]);

	if(rcData[2] < 1030 || !ARMING_FLAG(ARMED))
	{
	  Reset_All_PID_Integrator();
	}

	if(rcData[3] < 1485 || rcData[3] > 1515)
	{
	  yaw_heading_reference = (attitude.values.yaw/10);
	  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (rcData[3] - 1500), mpu.gyro.gyroADC[Z]);

	  ccr1 = 10500 + 500 + (rcData[2] - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
	  ccr2 = 10500 + 500 + (rcData[2] - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
	  ccr3 = 10500 + 500 + (rcData[2] - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
	  ccr4 = 10500 + 500 + (rcData[2] - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
	}
	else
	{
	  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, (attitude.values.yaw/10), mpu.gyro.gyroADC[Z]);

	  ccr1 = 10500 + 500 + (rcData[2] - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result;
	  ccr2 = 10500 + 500 + (rcData[2] - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
	  ccr3 = 10500 + 500 + (rcData[2] - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result;
	  ccr4 = 10500 + 500 + (rcData[2] - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
	}
}
